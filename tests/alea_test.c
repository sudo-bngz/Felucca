/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Nicolas Bounoughaz */
/* ALEA (src/eng_alea.c, its bank src/alea_bank.c, the editor's ALEA_* in src/editor_alea.c) against simulated NOR
 * flash: the factory bank with an empty flash; a bank sent over the editor (BEGIN, WRITEs, END) stored at 0xE8000 and
 * nothing else touched, the tail erased, played at once and again after a power-on; WRITEs out of order, a wrong CRC,
 * a bank that does not load, no transfer: refused, nothing committed; a power cut after any programmed byte: the new
 * bank or none (the factory one), never half; BEGIN 0 erases it. The engine: PATCH picks a patch, the voice is the
 * interpreter's (alea/alea_vm.h) note for note through the host's amplitude, it ends after its release; a new bank
 * while a note rings; the lines (a chorus): the slabs a part has, the shared region going to one part at a time. */
#define main hostsim_main
#include "hostsim.c"
#undef main

static int32_t fm1_enc_take(uint32_t e) { (void)e; return 0; }
static void fm1_irq_off(void) {}
static void fm1_irq_on(void) {}
static void lcd_sync(void) {}
static void lcd_power(uint32_t s) { (void)s; }
static void lcd_wake_now(void) {}
static void lcd_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint16_t *p)
{ (void)x; (void)y; (void)w; (void)h; (void)p; }
#define FELUCCA_FLASH 1
#include "../firmware/src/gfx.c"
#include "../firmware/src/panel.c"
#include "../firmware/src/ui.c"
#include "../firmware/src/menu_items.c"
static void panel_setup(void) {}

static uint8_t nor[0x100000], flash_ok = 1;
static long cut_bytes = -1;                      /* the power goes after that many programmed bytes */
static int dead;
static int st_read(uint32_t off, void *dst, uint32_t n) { memcpy(dst, nor + off, n); return 0; }
static int st_erase(uint32_t off)
{
    if (dead)
        return -8;
    if (off & 0xFFFu) {
        printf("erase off a sector at %#x\n", off);
        exit(1);
    }
    memset(nor + off, 0xFF, 4096);
    return 0;
}
static int st_prog(uint32_t off, const void *src, uint32_t n)
{
    const uint8_t *s = src;
    uint32_t i;
    if ((off & 0xFFu) + n > 256u) {
        printf("page wrap at %#x\n", off);
        exit(1);
    }
    for (i = 0; i < n; i++) {
        if (dead || (cut_bytes >= 0 && cut_bytes-- == 0)) {
            dead = 1;
            return -9;
        }
        nor[off + i] &= s[i];
    }
    return 0;
}
static uint32_t irq_save(void) { return 0; }
static void irq_restore(uint32_t f) { (void)f; }
static uint32_t fl_jedec_ram(void) { return 0; }
static void fl_plain_window_init(void) {}
#define FL_FAR(fn) (fn)
#include "../firmware/src/storage.c"
#include "../firmware/src/upreset.c"
#include "../firmware/src/project.c"

static uint8_t rep[1024];                         /* the editor's reply */
static uint32_t rep_n;
static void ed_b(uint32_t v) { if (rep_n < sizeof rep) rep[rep_n++] = (uint8_t)(v & 127u); }
static void ed_str(const char *s, uint32_t max)
{
    uint32_t i;
    for (i = 0; s && s[i] && i < max; i++)
        ed_b((uint8_t)s[i]);
    ed_b(0);
}
static int ed_flash_stop(void) { return transport_busy(); }
#include "../firmware/src/editor_alea.c"

static int check(const char *what, int ok)
{
    printf("alea: %-100s %s\n", what, ok ? "ok" : "FAIL");
    return ok ? 0 : 1;
}

static uint32_t request(uint32_t cmd, const uint8_t *a, uint32_t n)
{
    rep_n = 0;
    if (!ed_alea_handle(cmd, a, n))
        return 0;
    return rep_n;
}

/* ---- banks: the factory bank's patches, in another order and with other names ---- */
static uint8_t bank_a[ALEA_BANK_MAX], bank_b[ALEA_BANK_MAX];
static uint32_t len_a, len_b;
static uint32_t make_bank(uint8_t *out, const uint32_t *pick, uint32_t n, const char *tag)
{
    alea_table_t t;
    uint32_t k, p = 5;
    memset(&t, 0, sizeof t);
    alea_bank_parse(ALEA_FACTORY, sizeof ALEA_FACTORY, &t);
    memcpy(out, "ALB1", 4);
    out[4] = (uint8_t)n;
    for (k = 0; k < n; k++) {
        uint32_t s = t.size[pick[k]];
        memset(out + p, 0, 16);
        snprintf((char *)out + p, 16, "%s %s", tag, t.name[pick[k]]);
        memcpy(out + p + 16, &s, 4);
        memcpy(out + p + 20, ALEA_FACTORY + t.at[pick[k]], s);
        p += 20u + s;
    }
    return p;
}

static uint32_t crc_of(const uint8_t *b, uint32_t n) { return st_crc32(b, n); }

/* BEGIN, the WRITEs (chunk bytes each), END with crc ^ crc_xor -> END's rc, or the first rc that was not 0 (+ 100 for a
 * WRITE's) */
static uint32_t send_bank(const uint8_t *b, uint32_t n, uint32_t chunk, uint32_t crc_xor)
{
    uint8_t a[8 + 300];
    uint32_t off, k, c = crc_of(b, n) ^ crc_xor;
    a[0] = (uint8_t)(n & 127u); a[1] = (uint8_t)((n >> 7) & 127u); a[2] = (uint8_t)(n >> 14);
    request(ED_ALEA_BEGIN, a, 3);
    if (rep[0])
        return rep[0];
    for (off = 0; off < n; off += chunk) {
        uint32_t m = n - off < chunk ? n - off : chunk, na = 3, j;
        a[0] = (uint8_t)(off & 127u); a[1] = (uint8_t)((off >> 7) & 127u); a[2] = (uint8_t)(off >> 14);
        for (k = 0; k < m; k += 7u) {                     /* pack7 */
            uint32_t g = m - k < 7u ? m - k : 7u, hi = 0;
            for (j = 0; j < g; j++)
                hi |= (uint32_t)(b[off + k + j] >> 7) << j;
            a[na++] = (uint8_t)hi;
            for (j = 0; j < g; j++)
                a[na++] = b[off + k + j] & 127u;
        }
        request(ED_ALEA_WRITE, a, na);
        if (rep[3])
            return 100u + rep[3];
    }
    for (k = 0; k < 5u; k++)
        a[k] = (uint8_t)((c >> (7u * k)) & 127u);
    request(ED_ALEA_END, a, 5);
    return rep[0];
}

/* LIST says: stored, n patches, these names */
static int list_is(uint32_t stored, uint32_t n, const char *const *names)
{
    uint32_t k, p = 2;
    request(ED_ALEA_LIST, rep, 0);
    if (rep_n < 2u || rep[0] != stored || rep[1] != n)
        return 0;
    for (k = 0; k < n; k++) {
        if (strcmp((const char *)rep + p, names[k]))
            return 0;
        p += (uint32_t)strlen(names[k]) + 1u;
    }
    return p == rep_n;
}

static void reset(void)
{
    memset(nor, 0xFF, sizeof nor);
    memset(&song, 0, sizeof song);
    memset(trk, 0, sizeof trk);
    memset(&chain, 0, sizeof chain);
    chain_defaults(&chain_config);
    memset(proj_slot, 0, sizeof proj_slot);
    memset(up_bank, 0, sizeof up_bank);
    memset(&upf, 0, sizeof upf);
    memset(&ui, 0, sizeof ui);
    memset(&alea_tx, 0, sizeof alea_tx);
    host_tracks_init();
    flash_ok = 1;
    dead = 0;
    cut_bytes = -1;
    transport_req = 0;
    up_boot();
}
static void power_on(void) { dead = 0; cut_bytes = -1; up_boot(); }

static const char *const FACTORY_NAMES[] = {"BASS", "PAD", "BELL", "STAB"};
static const char *const A_NAMES[] = {"A STAB", "A BASS", "A BELL"};
static const char *const B_NAMES[] = {"B PAD", "B PAD", "B BELL", "B BASS", "B STAB"};

static int store_test(void)
{
    static uint8_t snap[0x100000];
    static const uint32_t PA[] = {3, 0, 2}, PB[] = {1, 1, 2, 0, 3};
    uint8_t a[16];
    uint32_t k, rc, ok, kept, bad = 0, cuts, news, olds;
    reset();
    len_a = make_bank(bank_a, PA, 3, "A");
    len_b = make_bank(bank_b, PB, 5, "B");
    bad += check("an empty flash: the factory bank (BASS PAD BELL STAB); LIST: not stored, 4 patches, their names",
                 !alea_bank.user && list_is(0, 4, FACTORY_NAMES));
    memcpy(snap, nor, sizeof nor);
    rc = send_bank(bank_a, len_a, 256, 0);
    ok = rc == 0 && rep[1] == 3 && list_is(1, 3, A_NAMES) && alea_bank.len == len_a && !memcmp(alea_bank.src, bank_a, len_a);
    bad += check("a bank over the editor (BEGIN, WRITEs of 256 bytes, END): rc 0, 3 patches, played at once (LIST)", ok);
    for (k = 0, kept = 1; k < sizeof nor; k++)
        if (nor[k] != snap[k] && !(k >= 0xE8000u && k < 0xE8000u + ALEA_DATA + len_a))
            kept = 0;
    for (k = 0xE8F00u; k < 0xE9000u; k++)
        kept &= nor[k] == 0xFFu;
    bad += check("stored at 0xE8000 (its record, the bytes from 0xE8100), nothing else written, the tail 0xF00.. erased",
                 kept && !memcmp(nor + 0xE8100u, bank_a, len_a));
    memset(&alea_bank, 0, sizeof alea_bank);
    power_on();
    bad += check("power-on: the stored bank", alea_bank.user && list_is(1, 3, A_NAMES));
    rc = send_bank(bank_b, len_b, 100, 0);
    bad += check("another (WRITEs of 100 bytes): replaces it, 5 patches", rc == 0 && list_is(1, 5, B_NAMES));
    rc = send_bank(bank_a, len_a, 256, 0x40u);
    power_on();
    bad += check("a wrong CRC: rc 2, nothing committed (B plays on until the power-on; then the factory bank)",
                 rc == 2 && list_is(0, 4, FACTORY_NAMES));
    send_bank(bank_a, len_a, 256, 0);
    a[0] = 10; a[1] = a[2] = 0; a[3] = 0; a[4] = 1;
    request(ED_ALEA_WRITE, a, 5);
    ok = rep[3] == 5;
    a[0] = (uint8_t)(len_a & 127u); a[1] = (uint8_t)(len_a >> 7); a[2] = 0;
    request(ED_ALEA_BEGIN, a, 3);
    a[0] = 10; a[1] = a[2] = 0; a[3] = 0; a[4] = 1;
    request(ED_ALEA_WRITE, a, 5);
    ok &= rep[3] == 1;
    a[0] = 0x7F; a[1] = 0x7F; a[2] = 0;
    request(ED_ALEA_BEGIN, a, 3);
    ok &= rep[0] == 1;
    power_on();
    bad += check("refused: a WRITE with no transfer (5), out of order (1), BEGIN over 3584 bytes (1); after it no bank",
                 ok && list_is(0, 4, FACTORY_NAMES));
    {
        static uint8_t junk[400];
        for (k = 0; k < sizeof junk; k++)
            junk[k] = (uint8_t)(k * 37u);
        memcpy(junk, bank_a, 25);                          /* ALB1, a name and a size: then no ALP1 */
        junk[4] = 1;
        rc = send_bank(junk, sizeof junk, 256, 0);
        power_on();
        bad += check("a bank whose patch does not load (the CRC right): rc 3, nothing committed", rc == 3 &&
                     list_is(0, 4, FACTORY_NAMES));
    }
    send_bank(bank_a, len_a, 256, 0);
    a[0] = a[1] = a[2] = 0;
    request(ED_ALEA_BEGIN, a, 3);
    ok = rep[0] == 0 && list_is(0, 4, FACTORY_NAMES);
    power_on();
    bad += check("BEGIN 0: the stored bank erased, the factory one at once and after a power-on",
                 ok && list_is(0, 4, FACTORY_NAMES));
    /* a power cut after each programmed byte of a transfer of B over A */
    for (cuts = 0, news = 0, olds = 0, ok = 1;; cuts++) {
        reset();
        send_bank(bank_a, len_a, 256, 0);
        cut_bytes = (long)cuts;
        rc = send_bank(bank_b, len_b, 256, 0);
        if (!dead) {                                       /* (no cut: every byte written) */
            memset(&alea_bank, 0, sizeof alea_bank);
            power_on();
            news += rc == 0 && list_is(1, 5, B_NAMES);
            break;
        }
        memset(&alea_bank, 0, sizeof alea_bank);
        power_on();
        if (list_is(1, 5, B_NAMES))
            news++;
        else if (list_is(0, 4, FACTORY_NAMES))
            olds++;
        else
            ok = 0;
    }
    bad += check("a power cut after any programmed byte (each of them): the new bank or the factory one, never half", ok &&
                 cuts > len_b && news == 1u && olds == cuts);
    printf("alea:   (%u cuts: %u the new bank, %u the factory one)\n", cuts, news, olds);
    return bad;
}

/* ---- the engine ---- */
static int32_t blocks(uint32_t nb, double *rms)  /* nb blocks of the mix: its peak; *rms of the left */
{
    int32_t o[2 * CTL], peak = 0;
    double s = 0;
    uint32_t b, i;
    for (b = 0; b < nb; b++) {
        mix_block(o, CTL);
        for (i = 0; i < 2u * CTL; i += 2u) {
            int32_t v = o[i] < 0 ? -o[i] : o[i];
            peak = v > peak ? v : peak;
            s += (double)o[i] * o[i];
        }
    }
    if (rms)
        *rms = sqrt(s / (nb * CTL));
    return peak;
}
static uint32_t sounding(const track_t *t)
{
    uint32_t i, n = 0;
    for (i = 0; i < NVOICE; i++)
        n += t->v[i].active;
    return n;
}

/* the engine's render against the interpreter's: a track's voice by hand (note_on, render with the amplitude at 1.0)
 * and a voice of the same patch played by alea_vm.h alone, the same frequency and velocity */
static int same_as_vm(uint32_t patch, uint32_t note, uint32_t vel, double *worst)
{
    static alea_vm_t vm;
    static alea_voice_t vv;
    static int32_t words[ALEA_VOICE_WORDS];
    static int16_t lines[8192];
    track_t *t = &trk[0];
    voice_t *v = &t->v[0];
    vmod_t m;
    uint32_t b, i, frames = FS / CTL;
    *worst = 0;
    host_preset(t, ENGI_ALEA, 0);
    t->p[P_E0] = (int16_t)patch;
    alea_block(t);
    memset(v, 0, sizeof *v);
    v->note = (uint8_t)note; v->vel = (uint8_t)vel; v->gate = 1; v->active = 1;
    v->pitch16 = v->pitch_cur = (int32_t)note * 16;
    ENG_ALEA.note_on(t, v);
    memset(&m, 0, sizeof m);
    m.inc = pitch_inc((uint32_t)v->pitch_cur);
    m.amp0 = m.amp1 = 32767;
    alea_vm_load(&vm, alea_bank.src + alea_bank.tab.at[patch], alea_bank.tab.size[patch]);
    alea_voice_init(&vm, &vv, words, lines, sizeof lines / sizeof lines[0]);
    alea_voice_on(&vm, &vv, alea_freq(m.inc), ((int32_t)vel << ALEA_Q) / 127, ALEA_ONE);
    for (b = 0; b < frames; b++) {
        int32_t o[CTL] = {0}, r[CTL] = {0};
        if (b == frames / 2u)
            v->gate = 0;
        alea_voice_gate(&vv, v->gate);
        ENG_ALEA.render(t, v, o, CTL, &m);
        alea_voice_render(&vm, &vv, r, CTL);
        for (i = 0; i < CTL; i++) {
            int32_t s = (int32_t)(((int64_t)r[i] * (100 * 328)) >> 16);
            double d;
            s = voice_amp(clamp(soft_knee(s, 24576), -32767, 32767), &m, i);
            d = fabs((double)o[i] - s);
            *worst = d > *worst ? d : *worst;
        }
    }
    return *worst < 1.0;
}

static int engine_test(void)
{
    uint32_t bad = 0, k, ok;
    double rms[4], w;
    track_t *t = &trk[0];
    reset();
    for (k = 0, ok = 1; k < 4u; k++) {
        ok &= same_as_vm(k, 48u + 12u * (k & 1u), 100, &w);
        if (w >= 1.0)
            printf("alea:   patch %u: off by %.1f\n", k, w);
    }
    bad += check("the voice is the interpreter's, sample for sample (BASS, PAD, BELL, STAB; the release half way)", ok);
    for (k = 0, ok = 1; k < 4u; k++) {
        host_tracks_init();
        host_preset(t, ENGI_ALEA, k);
        t->p[P_DIST] = t->p[P_CHOR] = t->p[P_DLY] = t->p[P_REV] = 0;
        trk_note_on(t, 48, 100);
        blocks(FS / 2u / CTL, &rms[k]);
        trk_note_off(t, 48);
        blocks(4u * FS / CTL, 0);
        ok &= rms[k] > 300.0 && !sounding(t);
        printf("alea:   preset ALEA %u (%s): rms %.0f\n", k + 1u, alea_bank.tab.name[k], rms[k]);
    }
    bad += check("each factory preset plays its patch (PATCH = its number) and its voice ends after the release", ok &&
                 rms[0] != rms[1] && rms[1] != rms[2] && rms[2] != rms[3]);
    host_tracks_init();
    host_preset(t, ENGI_ALEA, 1);
    trk_note_on(t, 60, 100);
    blocks(FS / 4u / CTL, 0);
    ok = sounding(t) == 1;
    send_bank(bank_a, len_a, 256, 0);                     /* a new bank while PAD rings: patch 1 is A BASS now */
    blocks(4, 0);
    ok &= alea_part(t)->gen == alea_bank.gen && !strcmp(alea_bank.tab.name[alea_part(t)->loaded - 1u], "A BASS");
    trk_note_off(t, 60);
    blocks(4u * FS / CTL, 0);
    bad += check("a new bank while a note rings: the part loads its patch from it at the next block; the voice ends",
                 ok && !sounding(t));
    {   /* the pitch: BELL's tail (its index decayed: a sine) at A4 */
        int32_t o[2 * CTL], prev = 0;
        uint32_t b, i, ups = 0, first = 0, last = 0, f = 0;
        host_tracks_init();
        host_preset(t, ENGI_ALEA, 2);
        t->p[P_DIST] = t->p[P_CHOR] = t->p[P_DLY] = t->p[P_REV] = 0;
        trk_note_on(t, 69, 100);
        for (b = 0; b < FS / CTL; b++) {
            mix_block(o, CTL);
            for (i = 0; i < 2u * CTL; i += 2u, f++) {
                if (f > FS / 2u && prev < 0 && o[i] >= 0) {
                    if (!ups++)
                        first = f;
                    last = f;
                }
                prev = o[i];
            }
        }
        trk_note_off(t, 69);
        w = (ups - 1u) * (double)FS / (last - first);
        printf("alea:   A4: %.2f Hz\n", w);
        bad += check("the note's pitch is the host's (A4: 440 Hz within 0.5)", fabs(w - 440.0) < 0.5);
        blocks(4u * FS / CTL, 0);
    }
    t->p[P_E0] = 15;
    alea_block(t);
    bad += check("PATCH past the bank's patches: its last one", alea_part(t)->loaded == 3u);
    return bad;
}

/* ---- lines: PAD has a chorus ---- */
static int lines_test(void)
{
    uint32_t bad = 0, k, slabs[NPART];
    reset();
    printf("alea:   a part has %u int16s of lines (src/fm1.rs PART_LINES), the shared region %u\n",
           (unsigned)ALEA_PART_LINES, ALEA_SHARED / 2u);
    bad += check("the part's region leaves at least the lines alea's compiler counts on (2860)", ALEA_PART_LINES >= 2860u);
    for (k = 0; k < 2u; k++) {
        host_preset(&trk[k], ENGI_ALEA, 1);
        blocks(2, 0);                                      /* (the part's engine memory claimed, the patch loaded) */
        slabs[k] = alea_part(&trk[k])->nslab;
    }
    bad += check("PAD on two parts: the first gets the shared region (8 voices with lines), the other its own (2)",
                 slabs[0] == 8u && slabs[1] == ALEA_PART_LINES / alea_part(&trk[1])->need && alea_shared_owner() == 0u);
    for (k = 0; k < NVOICE + 2u; k++)
        trk_note_on(&trk[1], 40u + k, 100);
    blocks(8, 0);
    {
        uint32_t with = 0;
        for (k = 0; k < NVOICE; k++)
            with += alea_part(&trk[1])->slab_of[k] != 0;
        bad += check("eight notes on the second: two voices with lines, the others dry, all sounding",
                     with == slabs[1] && sounding(&trk[1]) == NVOICE);
    }
    for (k = 0; k < NVOICE + 2u; k++)
        trk_note_off(&trk[1], 40u + k);
    blocks(6u * FS / CTL, 0);
    bad += check("after the release every slab is free again", !sounding(&trk[1]) && !alea_part(&trk[1])->slab_by[0] &&
                 !alea_part(&trk[1])->slab_by[1]);
    host_preset(&trk[0], 0, 0);                    /* (ANALOG) */
    blocks(64, 0);
    bad += check("the first part on another engine: the next ALEA part playing PAD takes the shared region",
                 alea_shared_owner() == 1u && alea_part(&trk[1])->nslab == 8u);
    host_preset(&trk[0], ENGI_ALEA, 1);
    trk_note_on(&trk[1], 50, 100);
    blocks(4, 0);
    bad += check("  and gives it back when the first plays PAD again (the second's voices keep their own lines, dry beyond)",
                 alea_shared_owner() == 0u && alea_part(&trk[0])->nslab == 8u && alea_part(&trk[1])->nslab == slabs[1] &&
                 sounding(&trk[1]) == 1u);
    trk_note_off(&trk[1], 50);
    blocks(6u * FS / CTL, 0);
    return bad;
}

/* the four factory presets, a phrase each, with their sends: build/host/alea_factory.wav (to listen to) */
static void factory_wav(const char *path)
{
    static const uint8_t PH[4][8] = {{36, 36, 48, 36, 39, 36, 43, 34}, {60, 0, 0, 0, 63, 0, 0, 0},
                                     {72, 0, 79, 0, 76, 0, 84, 0}, {0, 63, 0, 0, 0, 63, 0, 65}};
    const uint32_t step = FS / 8u / CTL, frames = 4u * 16u * 2u * step * CTL + 2u * FS / CTL * CTL;
    FILE *w = fopen(path, "wb");
    uint32_t k, st, b;
    if (!w)
        return;
    wav_hdr(w, frames);
    host_tracks_init();
    for (k = 0; k < 4u; k++)
        for (st = 0; st < 16u; st++) {
            uint32_t n = PH[k][st % 8u];
            if (st == 0)
                host_preset(&trk[0], ENGI_ALEA, k);
            if (n)
                trk_note_on(&trk[0], n, 100);
            for (b = 0; b < 2u * step; b++) {
                int32_t o[2 * CTL];
                uint32_t i;
                if (b == step && n && k != 1)
                    trk_note_off(&trk[0], n);
                if (b == 2u * step - 1u && n && k == 1)
                    trk_note_off(&trk[0], n);
                mix_block(o, CTL);
                for (i = 0; i < 2u * CTL; i += 2u)
                    wav_put(w, o[i], o[i + 1]);
            }
        }
    for (b = 0; b < 2u * FS / CTL; b++) {
        int32_t o[2 * CTL];
        uint32_t i;
        mix_block(o, CTL);
        for (i = 0; i < 2u * CTL; i += 2u)
            wav_put(w, o[i], o[i + 1]);
    }
    fclose(w);
    printf("alea:   the factory presets: %s\n", path);
}

int main(void)
{
    uint32_t bad = 0;
    if (getenv("ALEA_WAV")) {
        reset();
        factory_wav(getenv("ALEA_WAV"));
        return 0;
    }
    bad += store_test();
    bad += engine_test();
    bad += lines_test();
    printf("alea: %s\n", bad ? "FAILED" : "all ok");
    return bad ? 1 : 0;
}
