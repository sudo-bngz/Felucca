/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Nicolas Bounoughaz */
/* ALEA (engine 15, FELUCCA_ALEA): plays patches written in alea, the generative-art language
 * (https://github.com/sudo-bngz/alea): a work's `synth { … }`, compiled by `alea work.alea --fm1 bank.syx`, is a small
 * program of oscillators, filters, envelopes, delays and arithmetic that alea/alea_vm.h (MIT) runs one voice per note
 * in fixed point. A bank of up to 16 patches comes over USB from the editor protocol (editor_alea.c) and is kept in
 * flash (alea_bank.c); without one the engine plays its four factory patches (alea/alea_factory.h).
 *
 * EDIT 1: PATCH (the bank's patch, its name on the card), LEVL. The patch's own envelopes are the voice's amplitude
 * (ownenv): ATK DEC SUS REL are not applied; LFO -> AMP, the matrix's AMP and the fades are. The pitch is the host's
 * (glide, bend, unison detune, TUNE), read once a block.
 *
 * Memory: a part keeps the patch it plays (the interpreter's form of it), its eight voices' state words and as many
 * voices' delay and chorus lines as fit in the part's engine memory (engines.c eng_mem); one more region, ALEA_SHARED
 * bytes, is for the first ALEA part (in track order) whose patch wants more lines than that. A voice gets lines at its
 * note-on when a slab is free (one patch's lines, ALEA_SLABS at most a part), else it plays its delays and choruses
 * dry: what fits is printed by `alea --fm1` (src/fm1.rs ALEA_PART_LINES). */

#define ALEA_NO_STRING_H                 /* (the firmware is freestanding: bsp/chip/libc.c) */
#include "alea/alea_vm.h"

#ifndef ALEA_SHARED
#define ALEA_SHARED 16384u              /* bytes of lines one part can borrow (.pool) */
#endif
#define ALEA_VOICE_WORDS 128u           /* a voice's state: ops + state words (src/fm1.rs checks a patch fits) */
#define ALEA_SLABS NVOICE
#define ALEA_BANK_MAX 3584u             /* a bank's bytes: one flash sector's payload (alea_bank.c) */
#define ALEA_NPATCH 16u

/* ---- the bank the engine plays: the flash's (copied into img by alea_bank_set, main loop) or the factory one (read where
 * it is). The audio side reads it only in alea_load, and not while busy: the main loop never interrupts the audio, so a
 * load that finds busy 0 sees a whole bank; a part reloads its patch when gen moves on ---- */
typedef struct {
    uint16_t at[ALEA_NPATCH];            /* a patch's bytes: src + at[k], size[k] */
    uint16_t size[ALEA_NPATCH];
    char name[ALEA_NPATCH][16];
    uint8_t n;
} alea_table_t;
static struct {
    const uint8_t *src;
    uint32_t len;
    alea_table_t tab;
    uint8_t user;                        /* 1 = the bank came over USB (flash), 0 = the factory bank */
    uint8_t gen;
    volatile uint8_t busy;
    uint8_t img[ALEA_BANK_MAX];
} alea_bank __attribute__((section(".pool")));

/* an ALB1 image -> its patch table; 0 if it is not a bank */
static int alea_bank_parse(const uint8_t *b, uint32_t len, alea_table_t *t)
{
    uint32_t p = 5, k, cnt;
    if (len < 5u || len > ALEA_BANK_MAX || memcmp(b, "ALB1", 4) || !b[4] || b[4] > ALEA_NPATCH)
        return 0;
    cnt = b[4];
    for (k = 0; k < cnt; k++) {
        uint32_t sz, i;
        if (p + 20u > len)
            return 0;
        sz = alea_rd32(b + p + 16);
        if (sz > len - p - 20u)
            return 0;
        for (i = 0; i < 15u; i++)
            t->name[k][i] = b[p + i] >= 32u && b[p + i] < 127u ? (char)b[p + i] : 0;
        t->name[k][15] = 0;
        t->at[k] = (uint16_t)(p + 20u);
        t->size[k] = (uint16_t)sz;
        p += 20u + sz;
    }
    t->n = (uint8_t)cnt;
    return p == len;
}

#include "alea/alea_factory.h"           /* ALEA_FACTORY[]: the bank when flash holds none */

/* main loop: the engine plays b (len bytes, ALB1) from now on, or the factory bank (b = 0). 0 when b is not a bank */
static int alea_bank_set(const uint8_t *b, uint32_t len)
{
    alea_table_t t;
    const uint8_t *src = b ? alea_bank.img : ALEA_FACTORY;
    if (!b) {
        b = ALEA_FACTORY;
        len = sizeof ALEA_FACTORY;
    }
    memset(&t, 0, sizeof t);
    if (!alea_bank_parse(b, len, &t))
        return 0;
    alea_bank.busy = 1;
    RING_PUBLISH();
    if (src == alea_bank.img && b != alea_bank.img)
        memcpy(alea_bank.img, b, len);
    alea_bank.src = src;
    alea_bank.len = len;
    alea_bank.tab = t;
    alea_bank.user = src == alea_bank.img;
    alea_bank.gen++;
    RING_PUBLISH();
    alea_bank.busy = 0;
    return 1;
}

/* ---- a part's state, in its engine memory (zeroed when another engine had it: loaded 0 = no patch) ---- */
typedef struct {
    uint8_t loaded;                      /* patch + 1 */
    uint8_t gen;                         /* alea_bank.gen it was loaded from */
    uint8_t ok;                          /* the patch loaded (else the part is silent) */
    uint8_t nslab;                       /* slabs of lines: in lines[] first, then the shared region's */
    uint8_t slab_in_part;                /* .. how many of them are in lines[] */
    uint8_t wants;                       /* 1: the patch's lines want the shared region too (alea_shared_owner) */
    uint8_t slab_of[NVOICE];             /* a voice's slab + 1, 0 = none (dry) */
    uint8_t slab_by[ALEA_SLABS];         /* a slab's voice + 1, 0 = free */
    uint32_t need;                       /* int16s of lines a voice needs */
    alea_vm_t vm;
    alea_voice_t v[NVOICE];
    int32_t words[NVOICE][ALEA_VOICE_WORDS];
    int16_t lines[];                     /* the rest of the region */
} alea_part_t;
#define ALEA_PART_LINES ((ENG_MEM_PART - sizeof(alea_part_t)) / sizeof(int16_t))
ENG_MEM_FITS("ALEA", sizeof(alea_part_t) + 1024u);

static int16_t alea_shared[ALEA_SHARED / 2u] __attribute__((section(".pool")));

static alea_part_t *alea_part(const track_t *t) { return (alea_part_t *)ENG_MEM((uint32_t)(t - trk) % NPART); }

/* the part the shared lines are for: the first playing ALEA whose region holds ALEA's state (eng_mem_owner) and whose
 * patch wants more lines than the region has. Worked out from the parts' state, never stored: the parts may render on
 * two cores */
static uint32_t alea_shared_owner(void)
{
    uint32_t q;
    for (q = 0; q < NPART; q++)
        if (trk[q].engine == ENGI_ALEA && eng_mem_owner[q] == ENGI_ALEA + 1u && ((alea_part_t *)ENG_MEM(q))->wants)
            return q;
    return NPART;
}

/* once a block: the part takes the shared lines when they are its, gives them back when they are another's (its
 * voices that had them play their lines dry from then on) */
static void alea_lines_fit(uint32_t part, alea_part_t *a)
{
    uint32_t mine = a->wants && alea_shared_owner() == part, i;
    if (mine && a->nslab == a->slab_in_part) {
        uint32_t shared = (ALEA_SHARED / 2u) / a->need;
        a->nslab = (uint8_t)(a->slab_in_part + (shared > ALEA_SLABS - a->slab_in_part ? ALEA_SLABS - a->slab_in_part : shared));
    } else if (!mine && a->nslab > a->slab_in_part) {
        for (i = 0; i < NVOICE; i++)
            if (a->slab_of[i] > a->slab_in_part) {
                a->v[i].pool = 0;
                a->v[i].pool_len = 0;
                a->slab_of[i] = 0;
            }
        memset(a->slab_by + a->slab_in_part, 0, ALEA_SLABS - a->slab_in_part);
        a->nslab = a->slab_in_part;
    }
}

/* the part's patch: EDIT 1 PATCH of the bank, laid out again when the patch or the bank changed */
static void alea_load(track_t *t, alea_part_t *a)
{
    uint32_t k = (uint32_t)t->p[P_E0], i, w;
    if (alea_bank.busy)                  /* (a new bank half written: the next block) */
        return;
    if (!alea_bank.src) {                /* none set yet (a host without the store): the factory bank */
        if (!alea_bank_parse(ALEA_FACTORY, sizeof ALEA_FACTORY, &alea_bank.tab))
            return;
        alea_bank.src = ALEA_FACTORY;
        alea_bank.len = sizeof ALEA_FACTORY;
        alea_bank.gen++;
    }
    if (k >= alea_bank.tab.n)
        k = alea_bank.tab.n - 1u;
    if (a->loaded == k + 1u && a->gen == alea_bank.gen)
        return;
    memset(a, 0, sizeof *a);
    a->loaded = (uint8_t)(k + 1u);
    a->gen = alea_bank.gen;
    if (alea_vm_load(&a->vm, alea_bank.src + alea_bank.tab.at[k], alea_bank.tab.size[k]))
        return;
    w = alea_voice_words(&a->vm);
    if (w > ALEA_VOICE_WORDS)
        return;
    a->ok = 1;
    a->need = a->vm.pool;
    if (a->need) {
        uint32_t in_part = ALEA_PART_LINES / a->need;
        a->slab_in_part = a->nslab = (uint8_t)(in_part > ALEA_SLABS ? ALEA_SLABS : in_part);
        a->wants = a->slab_in_part < ALEA_SLABS && (ALEA_SHARED / 2u) >= a->need;
    }
    for (i = 0; i < NVOICE; i++)
        alea_voice_init(&a->vm, &a->v[i], a->words[i], 0, 0);
}

static int16_t *alea_slab(alea_part_t *a, uint32_t s)
{
    return s < a->slab_in_part ? a->lines + s * a->need : alea_shared + (s - a->slab_in_part) * a->need;
}

static void alea_block(track_t *t)
{
    alea_part_t *a = alea_part(t);
    alea_load(t, a);
    if (a->ok && a->need)
        alea_lines_fit((uint32_t)(t - trk) % NPART, a);
}

/* the note's frequency (Hz, Q16) from a phase increment at FS: f = inc fs / 2^32 */
static int32_t alea_freq(uint32_t inc)
{
    uint64_t f = ((uint64_t)inc * FS) >> 16;
    return (int32_t)(f > (20000u << ALEA_Q) ? (20000u << ALEA_Q) : f);
}

static void alea_note_on(track_t *t, voice_t *v)
{
    alea_part_t *a = alea_part(t);
    uint32_t i = (uint32_t)(v - t->v) % NVOICE, s;
    alea_voice_t *av = &a->v[i];
    alea_load(t, a);
    if (!a->ok)
        return;
    if (a->need && !a->slab_of[i])       /* lines: a free slab, else dry */
        for (s = 0; s < a->nslab; s++)
            if (!a->slab_by[s]) {
                a->slab_by[s] = (uint8_t)(i + 1u);
                a->slab_of[i] = (uint8_t)(s + 1u);
                break;
            }
    av->pool = a->slab_of[i] ? alea_slab(a, a->slab_of[i] - 1u) : 0;
    av->pool_len = a->slab_of[i] ? a->need : 0;
    alea_voice_on(&a->vm, av, alea_freq(pitch_inc((uint32_t)clamp(v->pitch_cur, 0, 2047))),
                  ((int32_t)v->vel << ALEA_Q) / 127, ALEA_ONE);
}

/* the patch's output (Q16, ±8; 1.0 a full-scale voice), through LEVL and a soft knee, to Q15 and the voice level */
static void alea_render(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m)
{
    int32_t buf[CTL];
    alea_part_t *a = alea_part(t);
    alea_voice_t *av = &a->v[(uint32_t)(v - t->v) % NVOICE];
    int32_t lev = (int32_t)t->p[P_E1] * 328;          /* 100 = 1.0 in Q15 (32800) */
    uint32_t i;
    if (!a->ok || !av->active || n > CTL)
        return;
    if (av->gate != v->gate)
        alea_voice_gate(av, v->gate);
    av->freq = alea_freq(m->inc);
    memset(buf, 0, n * sizeof buf[0]);
    alea_voice_render(&a->vm, av, buf, n);
    for (i = 0; i < n; i++) {
        int32_t s = (int32_t)(((int64_t)buf[i] * lev) >> 16);   /* Q16 * Q15 >> 16: Q15 */
        s = soft_knee(s, 24576);
        out[i] += voice_amp(clamp(s, -32767, 32767), m, i);
    }
}

static int alea_done(track_t *t, voice_t *v)
{
    alea_part_t *a = alea_part(t);
    uint32_t i = (uint32_t)(v - t->v) % NVOICE;
    if (!a->ok || alea_voice_silent(&a->vm, &a->v[i])) {
        a->v[i].active = 0;
        if (a->slab_of[i]) {                           /* its lines back */
            a->slab_by[a->slab_of[i] - 1u] = 0;
            a->slab_of[i] = 0;
        }
        return 1;
    }
    return 0;
}

/* EDIT 1 PATCH: the bank's names */
static const char *alea_names[ALEA_NPATCH + 1];
static const param_desc_t *alea_desc(const track_t *t, uint32_t k)
{
    static param_desc_t d;
    uint32_t i;
    (void)t;
    if (k != 0)
        return 0;
    for (i = 0; i < ALEA_NPATCH; i++)
        alea_names[i] = i < alea_bank.tab.n ? alea_bank.tab.name[i] : "-";
    alea_names[ALEA_NPATCH] = 0;
    d = (param_desc_t){"PATCH", F_ENUM, 0, ALEA_NPATCH - 1, 0, alea_names, 0};
    return &d;
}

static const preset_t ALEA_PRESETS[] = {
    {"ALEA 1", {0, 100, 0, 0, 0, 0, 0, 0}, {0, 40, 127, 40}, 0, 1, FX(0, 0, 0, 0)},
    {"ALEA 2", {1, 100, 0, 0, 0, 0, 0, 0}, {0, 40, 127, 40}, 0, 0, FX(0, 20, 0, 30)},
    {"ALEA 3", {2, 100, 0, 0, 0, 0, 0, 0}, {0, 40, 127, 40}, 0, 0, FX(0, 0, 25, 30)},
    {"ALEA 4", {3, 100, 0, 0, 0, 0, 0, 0}, {0, 40, 127, 40}, 0, 0, FX(0, 0, 0, 20)},
};

static const engine_t ENG_ALEA = {
    .name = "ALEA",
    .page_title = {"PATCH", "-"},
    .edit = {
        {"PATCH", F_INT, 0, ALEA_NPATCH - 1, 0, 0, 0},
        {"LEVL", F_PCT, 0, 127, 100, 0, 0},
        {"-", F_INT, 0, 0, 0, 0, 0}, {"-", F_INT, 0, 0, 0, 0, 0}, {"-", F_INT, 0, 0, 0, 0, 0},
        {"-", F_INT, 0, 0, 0, 0, 0}, {"-", F_INT, 0, 0, 0, 0, 0}, {"-", F_INT, 0, 0, 0, 0, 0},
    },
    .presets = ALEA_PRESETS,
    .npresets = NELEM(ALEA_PRESETS),
    .note_on = alea_note_on,
    .render = alea_render,
    .knob = {P_E0, P_E1, P_LEVEL, P_PAN},
    .desc = alea_desc,
    .block = alea_block,
    .ownenv = 1,
    .done = alea_done,
    .mem = (uint16_t)ENG_MEM_PART,
};
