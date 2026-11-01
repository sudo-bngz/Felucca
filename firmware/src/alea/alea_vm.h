/* SPDX-License-Identifier: MIT
 * alea_vm.h — the ALEA engine's interpreter: plays a patch compiled by
 * `alea work.alea --fm1` (src/fm1.rs; the format in FORMAT.md), one voice at
 * a time, in fixed point, with no float and no division per sample.
 *
 * Signals are Q16.16 (1.0 = 65536): the same slot can hold an audio sample,
 * an envelope or a frequency in Hz (up to 32767). Oscillator phases are 32-bit
 * accumulators; delay and chorus lines are Q15 in a pool the host provides.
 *
 * A host (fm1/eng_alea.c for Felucca, fm1/test/vm_main.c on a computer):
 *   alea_vm_load(&vm, bytes, size)             once per patch: checks it, lays out the state
 *   alea_voice_init(&vm, &v, words, pool, pool_size)
 *                                              once per voice: its state (alea_voice_words(&vm) int32s at
 *                                              words) and lines (pool_size int16s at pool; pool may be 0)
 *   alea_voice_on(&vm, &v, freq_q16, vel_q16, level_q16)
 *   alea_voice_gate(&v, 0)                     the key is released
 *   alea_voice_render(&vm, &v, out, n)         n samples, Q16, added to out[] (never overwritten)
 *   alea_voice_silent(&vm, &v)                 1 once the note has died away (the host may free it)
 * Everything is static; include this header in the one file that plays patches. */
#ifndef ALEA_VM_H
#define ALEA_VM_H
#include <stdint.h>
#include <string.h>

#ifndef ALEA_FS
#define ALEA_FS 44100u
#endif
#define ALEA_MAX_OPS 64
#define ALEA_MAX_STATES 32
#define ALEA_Q 16
#define ALEA_ONE (1 << ALEA_Q)

enum { AOP_CONST, AOP_NOTE, AOP_OSC, AOP_NOISE, AOP_ADSR, AOP_PERC, AOP_FILTER, AOP_DELAY, AOP_DRIVE, AOP_CHORUS,
       AOP_ADD, AOP_SUB, AOP_MUL, AOP_DIV, AOP_NEG };
enum { AST_PHASE, AST_ENV, AST_SVF, AST_LINE, AST_CHORUS };

typedef struct {
    uint8_t op, aux;             /* aux: which note value / wave / pass */
    uint16_t a, b, c, state;     /* operand slots; state: index into the state table */
    int32_t k[4];                /* constants, Q16 (times, levels); DELAY: k[0] = length in samples */
} alea_op_t;

typedef struct {
    uint8_t kind;
    uint32_t len;                /* LINE / CHORUS: samples */
    uint32_t at;                 /* LINE / CHORUS: offset in the pool (int16s) */
    uint32_t w;                  /* offset of its words in a voice's state (ints) */
} alea_state_t;

typedef struct {
    uint32_t nops, nstates;
    alea_op_t ops[ALEA_MAX_OPS];
    alea_state_t st[ALEA_MAX_STATES];
    uint32_t words;              /* int32 words of state a voice needs */
    uint32_t pool;               /* int16 samples of lines a voice needs */
    uint32_t release_q16;        /* the longest adsr release (seconds, Q16): how long a note rings after its key */
    uint32_t struck_q16;         /* the longest perc (attack + release) */
    uint32_t echo_q16;           /* how long echoes go on */
    uint8_t gated;
} alea_vm_t;

typedef struct {
    int32_t *slots;              /* the ops' outputs: vm->nops words */
    int32_t *st;                 /* the states' words (vm->words): phase 1; env 3; svf 8; chorus 1 */
    int16_t *pool;
    uint32_t pool_len;
    int32_t freq, vel, level;    /* Q16 */
    uint8_t gate, active;
    uint64_t time_acc;           /* seconds, Q48 */
    uint32_t samples, off_at;    /* samples since note-on; the sample the key was released */
    uint32_t seed;
    int32_t dc_x;                /* the DC blocker: the last input, and its output in Q32 (so that it decays to 0) */
    int64_t dc_y;
    uint32_t quiet;              /* samples of silence in a row */
} alea_voice_t;

/* ---------------------------------------------------------------- tables --- */
/* the tables: a quarter sine (Q15), exp(-6.9 u) (Q16), tanh (Q15) and the filter's g = tan(pi f / fs) on 12
 * octaves above 10 Hz, 16 steps an octave (Q16); made for one sample rate by gen_tables.py */
#include "alea_vm_tables.h"
#if ALEA_TABLES_FS != ALEA_FS
#error "alea_vm_tables.h was made for another sample rate: python3 fm1/gen_tables.py FS"
#endif

/* ------------------------------------------------------------- helpers --- */
static inline int32_t alea_mul(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b + (1 << (ALEA_Q - 1))) >> ALEA_Q); }  /* rounded: a decay reaches 0 */
static inline int32_t alea_div(int32_t a, int32_t b)
{
    if (b > -16 && b < 16) return 0;
    return (int32_t)((int64_t)a * ALEA_ONE / b);
}
static inline int32_t alea_clamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }

static inline int32_t alea_sin_q16(uint32_t ph)      /* ph: a full turn is 2^32; out Q16 in [-1, 1] */
{
    uint32_t q = ph >> 30, i = (ph >> 22) & 255, f = (ph >> 14) & 255;
    int32_t a, b, s;
    if (q & 1) { a = ALEA_SINE[256 - i]; b = ALEA_SINE[255 - i]; }
    else { a = ALEA_SINE[i]; b = ALEA_SINE[i + 1]; }
    s = a + (((b - a) * (int32_t)f) >> 8);
    s = (q & 2) ? -s : s;
    return s * 2;                                     /* Q15 -> Q16 */
}

/* polyBLEP: the correction near a jump, t and dt as fractions of a cycle (2^32 = 1) */
static inline int32_t alea_blep(uint32_t t, uint32_t dt)
{
    if (dt == 0) return 0;
    if (t < dt) {
        int32_t x = (int32_t)(((uint64_t)t << ALEA_Q) / dt);          /* 0..1 Q16 */
        return x + x - alea_mul(x, x) - ALEA_ONE;
    }
    if (t > (uint32_t)(0u - dt)) {
        int32_t x = (int32_t)(((uint64_t)(t - (0u - dt)) << ALEA_Q) / dt) - ALEA_ONE;  /* -1..0 */
        return alea_mul(x, x) + x + x + ALEA_ONE;
    }
    return 0;
}

static inline uint32_t alea_rand(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return *s = x;
}

/* the filter's g for a cutoff in Hz (Q16): 16 steps an octave from 10 Hz, interpolated */
static inline int32_t alea_svf_g(int32_t fc)
{
    uint32_t r = (uint32_t)alea_clamp(fc, 10 << ALEA_Q, ALEA_G_TOP) / 10u;   /* f / 10, Q16: 1.0 .. 2^12 */
    int oct = 31 - __builtin_clz(r) - ALEA_Q;                                  /* floor(log2(f / 10)) */
    uint32_t mant, idx, frac, k;
    if (oct < 0) oct = 0;
    if (oct > 11) oct = 11;
    mant = r >> oct;                                                           /* Q16, 1.0 .. 2.0 */
    idx = (mant - ALEA_ONE) >> 12;                                             /* 0 .. 15 */
    frac = (mant - ALEA_ONE) & 4095;
    k = (uint32_t)oct * 16 + idx;
    return ALEA_G[k] + (int32_t)(((int64_t)(ALEA_G[k + 1] - ALEA_G[k]) * (int32_t)frac) >> 12);
}

/* ---------------------------------------------------------------- load --- */
static uint16_t alea_rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t alea_rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

/* 0 on success, else a reason */
static int alea_vm_load(alea_vm_t *vm, const uint8_t *b, uint32_t n)
{
    uint32_t i, p = 14;
    memset(vm, 0, sizeof *vm);
    if (n < 14 || memcmp(b, "ALP1", 4) != 0) return 1;
    if (alea_rd16(b + 4) != ALEA_FS) return 2;         /* compiled for another rate */
    vm->nops = alea_rd16(b + 6);
    vm->nstates = alea_rd16(b + 8);
    if (vm->nops == 0 || vm->nops > ALEA_MAX_OPS || vm->nstates > ALEA_MAX_STATES) return 3;
    for (i = 0; i < vm->nops; i++) {
        alea_op_t *o = &vm->ops[i];
        if (p >= n) return 4;
        o->op = b[p++];
        switch (o->op) {
        case AOP_CONST: o->k[0] = (int32_t)alea_rd32(b + p); p += 4; break;
        case AOP_NOTE: o->aux = b[p++]; break;
        case AOP_OSC: o->aux = b[p]; o->a = alea_rd16(b + p + 1); o->b = alea_rd16(b + p + 3); o->state = alea_rd16(b + p + 5); p += 7; break;
        case AOP_NOISE: break;
        case AOP_ADSR:
            o->k[0] = (int32_t)alea_rd32(b + p); o->k[1] = (int32_t)alea_rd32(b + p + 4);
            o->k[2] = (int32_t)alea_rd32(b + p + 8); o->k[3] = (int32_t)alea_rd32(b + p + 12);
            o->state = alea_rd16(b + p + 16); p += 18;
            vm->gated = 1;
            if ((uint32_t)o->k[3] > vm->release_q16) vm->release_q16 = (uint32_t)o->k[3];
            break;
        case AOP_PERC:
            o->k[0] = (int32_t)alea_rd32(b + p); o->k[1] = (int32_t)alea_rd32(b + p + 4); p += 8;
            if ((uint32_t)(o->k[0] + o->k[1]) > vm->struck_q16) vm->struck_q16 = (uint32_t)(o->k[0] + o->k[1]);
            break;
        case AOP_FILTER:
            o->aux = b[p]; o->a = alea_rd16(b + p + 1); o->b = alea_rd16(b + p + 3); o->c = alea_rd16(b + p + 5);
            o->state = alea_rd16(b + p + 7); p += 9; break;
        case AOP_DELAY:
            o->a = alea_rd16(b + p); o->k[0] = (int32_t)alea_rd32(b + p + 2); o->k[1] = (int32_t)alea_rd32(b + p + 6);
            o->k[2] = (int32_t)alea_rd32(b + p + 10); o->state = alea_rd16(b + p + 14); p += 16;
            if (vm->echo_q16 < 4u << ALEA_Q) vm->echo_q16 = 4u << ALEA_Q;
            break;
        case AOP_DRIVE: o->a = alea_rd16(b + p); o->b = alea_rd16(b + p + 2); p += 4; break;
        case AOP_CHORUS:
            o->a = alea_rd16(b + p); o->k[0] = (int32_t)alea_rd32(b + p + 2); o->k[1] = (int32_t)alea_rd32(b + p + 6);
            o->k[2] = (int32_t)alea_rd32(b + p + 10); o->state = alea_rd16(b + p + 14); p += 16; break;
        case AOP_ADD: case AOP_SUB: case AOP_MUL: case AOP_DIV:
            o->a = alea_rd16(b + p); o->b = alea_rd16(b + p + 2); p += 4; break;
        case AOP_NEG: o->a = alea_rd16(b + p); p += 2; break;
        default: return 5;
        }
        {   /* an op reads only slots before it */
            int reads = (o->op == AOP_OSC || o->op == AOP_FILTER || o->op == AOP_DELAY || o->op == AOP_DRIVE || o->op == AOP_CHORUS
                         || o->op >= AOP_ADD);
            int two = (o->op == AOP_OSC || o->op == AOP_FILTER || o->op == AOP_DRIVE || (o->op >= AOP_ADD && o->op <= AOP_DIV));
            if (reads && o->a >= i) return 6;
            if (two && o->b >= i) return 6;
            if (o->op == AOP_FILTER && o->c >= i) return 6;
            if ((o->op == AOP_OSC || o->op == AOP_ADSR || o->op == AOP_FILTER || o->op == AOP_DELAY || o->op == AOP_CHORUS)
                && o->state >= vm->nstates) return 7;
        }
    }
    for (i = 0; i < vm->nstates; i++) {
        alea_state_t *s = &vm->st[i];
        if (p + 5 > n) return 8;
        s->kind = b[p]; s->len = alea_rd32(b + p + 1); p += 5;
        switch (s->kind) {
        case AST_PHASE: s->w = vm->words; vm->words += 1; break;
        case AST_ENV: s->w = vm->words; vm->words += 3; break;
        case AST_SVF: s->w = vm->words; vm->words += 8; break;
        case AST_LINE: s->at = vm->pool; vm->pool += s->len; break;
        case AST_CHORUS: s->at = vm->pool; vm->pool += s->len; s->w = vm->words; vm->words += 1; break;
        default: return 9;
        }
    }
    if (vm->words > ALEA_MAX_STATES * 12) return 10;
    return 0;
}

/* -------------------------------------------------------------- voices --- */
/* the int32 words a voice of this patch keeps its state in (alea_voice_init's words) */
static inline uint32_t alea_voice_words(const alea_vm_t *vm) { return vm->nops + vm->words; }

static void alea_voice_init(const alea_vm_t *vm, alea_voice_t *v, int32_t *words, int16_t *pool, uint32_t pool_len)
{
    memset(v, 0, sizeof *v);
    v->slots = words;
    v->st = words + vm->nops;
    v->pool = pool;
    v->pool_len = pool_len;
    v->seed = 0xA1EA5EEDu;
}

static void alea_voice_on(const alea_vm_t *vm, alea_voice_t *v, int32_t freq_q16, int32_t vel_q16, int32_t level_q16)
{
    uint32_t i;
    v->freq = freq_q16; v->vel = vel_q16; v->level = level_q16;
    v->gate = 1; v->active = 1;
    v->time_acc = 0; v->samples = 0; v->off_at = 0; v->quiet = 0;
    v->dc_x = 0; v->dc_y = 0;
    v->seed = v->seed * 1664525u + 1013904223u + 1;
    memset(v->slots, 0, vm->nops * sizeof(int32_t));
    for (i = 0; i < vm->nstates; i++) {
        const alea_state_t *s = &vm->st[i];
        if (s->kind == AST_LINE || s->kind == AST_CHORUS) {
            if (v->pool && s->at + s->len <= v->pool_len) memset(v->pool + s->at, 0, s->len * sizeof(int16_t));
            if (s->kind == AST_CHORUS) v->st[s->w] = 0;
        } else {
            memset(v->st + s->w, 0, (s->kind == AST_SVF ? 8 : s->kind == AST_ENV ? 3 : 1) * sizeof(int32_t));
        }
    }
}

static void alea_voice_gate(alea_voice_t *v, int on)
{
    if (!on && v->gate) v->off_at = v->samples;
    v->gate = (uint8_t)(on != 0);
}

/* a line's sample `back` samples ago (Q16), or 0 when the pool can't hold it */
static inline int32_t alea_line_read(const alea_voice_t *v, const alea_state_t *s, uint32_t head, uint32_t back)
{
    uint32_t len = s->len, k;
    if (!v->pool || s->at + len > v->pool_len) return 0;
    k = (head + len - (back % len)) % len;
    return (int32_t)v->pool[s->at + k] * 8;
}

static inline int32_t alea_env_held(uint32_t t, int32_t attack, int32_t decay, int32_t sustain)
{
    if (attack < 66) attack = 66;                      /* 1 ms */
    if (decay < 66) decay = 66;
    if ((int32_t)t < attack) return alea_div((int32_t)t, attack);
    {
        int32_t u = alea_div((int32_t)t - attack, decay), w;
        if (u > ALEA_ONE) u = ALEA_ONE;
        w = ALEA_ONE - u;
        return sustain + alea_mul(ALEA_ONE - sustain, alea_mul(w, w));
    }
}

static inline int32_t alea_perc(uint32_t t, int32_t attack, int32_t release)
{
    if (attack < 33) attack = 33;                      /* 0.5 ms */
    if (release < 66) release = 66;
    if ((int32_t)t < attack) return alea_div((int32_t)t, attack);
    {
        int32_t u = alea_div((int32_t)t - attack, release);
        uint32_t i, f;
        if (u >= ALEA_ONE) return 0;
        i = (uint32_t)u >> 10; f = (uint32_t)u & 1023;  /* 64 steps */
        {
            int32_t e = ALEA_EXPD[i] + (((int32_t)ALEA_EXPD[i + 1] - (int32_t)ALEA_EXPD[i]) * (int32_t)f >> 10);
            return alea_mul(e, ALEA_ONE - u);
        }
    }
}

static inline int32_t alea_tanh(int32_t x)          /* Q16 in, Q16 out, |x| up to 8 */
{
    int32_t a = x < 0 ? -x : x, y;
    uint32_t i, f;
    if (a >= (8 << ALEA_Q)) y = ALEA_TANH[128] << 1;
    else {
        i = (uint32_t)a >> 12; f = (uint32_t)a & 4095;  /* 128 steps over 0..8 */
        y = (ALEA_TANH[i] + (((int32_t)ALEA_TANH[i + 1] - (int32_t)ALEA_TANH[i]) * (int32_t)f >> 12)) << 1;
    }
    return x < 0 ? -y : y;
}

/* the voice's next n samples, Q16, added to out[] */
static void alea_voice_render(const alea_vm_t *vm, alea_voice_t *v, int32_t *out, uint32_t n)
{
    static const uint64_t DT48 = ((uint64_t)1 << 48) / ALEA_FS;   /* a sample, in seconds Q48 */
    static const uint32_t PHK = (uint32_t)(((uint64_t)1 << 32) / ALEA_FS);  /* phase per Hz */
    uint32_t k;
    if (!v->active) return;
    for (k = 0; k < n; k++) {
        uint32_t t = (uint32_t)(v->time_acc >> 32);                 /* seconds, Q16 */
        uint32_t i;
        int32_t *s = v->slots;
        for (i = 0; i < vm->nops; i++) {
            const alea_op_t *o = &vm->ops[i];
            int32_t r = 0;
            switch (o->op) {
            case AOP_CONST: r = o->k[0]; break;
            case AOP_NOTE:
                r = o->aux == 0 ? v->freq : o->aux == 1 ? v->vel : o->aux == 2 ? (v->gate ? ALEA_ONE : 0)
                  : o->aux == 3 ? (int32_t)t : v->level;
                break;
            case AOP_OSC: {
                uint32_t *ph = (uint32_t *)&v->st[vm->st[o->state].w];
                int32_t f = s[o->a];
                uint32_t inc = (uint32_t)(((int64_t)(f < 0 ? -f : f) * PHK) >> ALEA_Q);
                uint32_t p = *ph;
                switch (o->aux) {
                case 0: r = alea_sin_q16(p); break;
                case 3: { int32_t x = (int32_t)(p >> 16) - 32768; r = ALEA_ONE - ((x < 0 ? -x : x) << 2); break; }  /* triangle */
                case 1: r = (int32_t)(p >> 15) - ALEA_ONE - alea_blep(p, inc); break;                                 /* saw */
                default: {                                                                                          /* square, pulse */
                    uint32_t w = o->aux == 2 ? 0x80000000u : (uint32_t)alea_clamp(s[o->b], 1311, 64225) << 16;
                    r = (p < w ? ALEA_ONE : -ALEA_ONE) + alea_blep(p, inc) - alea_blep(p - w, inc);
                }
                }
                *ph = p + (f < 0 ? 0u - inc : inc);
                break;
            }
            case AOP_NOISE: r = (int32_t)(alea_rand(&v->seed) >> 15) - ALEA_ONE; break;
            case AOP_ADSR: {
                int32_t *e = &v->st[vm->st[o->state].w];          /* e[0] released, e[1] level at release, e[2] time at release */
                if (v->gate) { e[0] = 0; r = alea_env_held(t, o->k[0], o->k[1], o->k[2]); }
                else {
                    int32_t u, w;
                    if (!e[0]) { e[0] = 1; e[2] = (int32_t)t; }
                    u = alea_div((int32_t)t - e[2], o->k[3] < 66 ? 66 : o->k[3]);
                    if (u > ALEA_ONE) u = ALEA_ONE;
                    w = ALEA_ONE - u;
                    r = alea_mul(e[1], alea_mul(w, w));
                }
                if (v->gate) e[1] = r;
                break;
            }
            case AOP_PERC: r = alea_perc(t, o->k[0], o->k[1]); break;
            case AOP_FILTER: {
                int32_t *f = &v->st[vm->st[o->state].w];          /* ic1 ic2 | a1 a2 a3 k | cut res */
                int32_t x = s[o->a], cut = s[o->b], res = alea_clamp(s[o->c], 0, ALEA_ONE);
                int32_t v1, v2, v3;
                if ((v->samples & 31) == 0 || f[3] == 0) {         /* the coefficients, once per block */
                    if (cut != f[6] || res != f[7] || f[3] == 0) {
                        int32_t g = alea_svf_g(cut);
                        int32_t kk = (2 << ALEA_Q) - alea_mul(127795, res);   /* 2 - 1.95 res */
                        int32_t a1 = alea_div(ALEA_ONE, ALEA_ONE + alea_mul(g, g + kk));
                        f[2] = a1; f[3] = alea_mul(g, a1); f[4] = alea_mul(g, f[3]); f[5] = kk; f[6] = cut; f[7] = res;
                    }
                }
                v3 = x - f[1];
                v1 = alea_mul(f[2], f[0]) + alea_mul(f[3], v3);
                v2 = f[1] + alea_mul(f[3], f[0]) + alea_mul(f[4], v3);
                f[0] = 2 * v1 - f[0];
                f[1] = 2 * v2 - f[1];
                f[0] = alea_clamp(f[0], -(64 << ALEA_Q), 64 << ALEA_Q);
                f[1] = alea_clamp(f[1], -(64 << ALEA_Q), 64 << ALEA_Q);
                r = o->aux == 0 ? v2 : o->aux == 2 ? v1 : x - alea_mul(f[5], v1) - v2;
                break;
            }
            case AOP_DELAY: {
                const alea_state_t *st = &vm->st[o->state];
                uint32_t len = st->len, head;
                int32_t x = s[o->a], back;
                if (!v->pool || st->at + len > v->pool_len) { r = x; break; }   /* no room: dry */
                head = v->samples % len;
                back = (int32_t)v->pool[st->at + head] * 8;                       /* lines hold Q13: ±4 */
                {
                    int32_t w = alea_clamp(x + alea_mul(back, o->k[1]), -(4 << ALEA_Q) + 8, (4 << ALEA_Q) - 8);
                    v->pool[st->at + head] = (int16_t)(w >> 3);
                }
                r = x + alea_mul(back, o->k[2]);
                break;
            }
            case AOP_DRIVE: {
                int32_t gain = ALEA_ONE + alea_mul(20 << ALEA_Q, alea_clamp(s[o->b], 0, ALEA_ONE));
                int32_t num = alea_tanh(alea_clamp(alea_mul(s[o->a], gain), -(8 << ALEA_Q), 8 << ALEA_Q));
                r = alea_div(num, alea_tanh(gain));
                break;
            }
            case AOP_CHORUS: {
                const alea_state_t *st = &vm->st[o->state];
                uint32_t len = st->len, head;
                int32_t x = s[o->a];
                uint32_t *ph = (uint32_t *)&v->st[st->w];
                int32_t depth = alea_clamp(o->k[0], 0, ALEA_ONE), lfo, late, wet;
                uint32_t back, frac;
                if (!v->pool || st->at + len > v->pool_len) { r = x; break; }
                head = v->samples % len;
                v->pool[st->at + head] = (int16_t)(alea_clamp(x, -(4 << ALEA_Q) + 8, (4 << ALEA_Q) - 8) >> 3);
                lfo = (ALEA_ONE + alea_sin_q16(*ph)) >> 1;        /* 0..1 */
                /* 7 to 25 ms late: (0.007 + 0.018 depth lfo) fs, in samples Q16 */
                late = (int32_t)((int64_t)(459 + alea_mul(1180, alea_mul(depth, lfo))) * ALEA_FS);  /* seconds Q16 × fs: samples, Q16 */
                back = (uint32_t)late >> ALEA_Q; frac = (uint32_t)late & 0xFFFF;
                {
                    int32_t a = alea_line_read(v, st, head, back), b = alea_line_read(v, st, head, back + 1);
                    wet = a + (int32_t)(((int64_t)(b - a) * (int32_t)frac) >> ALEA_Q);
                }
                *ph += (uint32_t)(((int64_t)o->k[1] * PHK) >> ALEA_Q);
                r = alea_mul(x, ALEA_ONE - (o->k[2] >> 1)) + alea_mul(wet, o->k[2]);
                break;
            }
            case AOP_ADD: r = s[o->a] + s[o->b]; break;
            case AOP_SUB: r = s[o->a] - s[o->b]; break;
            case AOP_MUL: r = alea_mul(s[o->a], s[o->b]); break;
            case AOP_DIV: r = alea_div(s[o->a], s[o->b]); break;
            case AOP_NEG: r = -s[o->a]; break;
            }
            s[i] = alea_clamp(r, -(4096 << ALEA_Q), 4096 << ALEA_Q);
        }
        {
            /* the DC blocker (15 Hz), then the output, clamped to ±8 */
            int32_t x = s[vm->nops - 1];
            /* y = x - x' + R y', R = 1 - 2 pi 15 / fs, the state in Q32 */
            int64_t y64 = ((int64_t)(x - v->dc_x) * 65536) + ((v->dc_y >> 8) * ALEA_DC_R >> 24);
            int32_t y = (int32_t)(y64 >> 16);
            v->dc_x = x; v->dc_y = y64;
            y = alea_clamp(y, -(8 << ALEA_Q), 8 << ALEA_Q);
            out[k] += y;
            if (y > -33 && y < 33) v->quiet++; else v->quiet = 0;
        }
        v->time_acc += DT48;
        v->samples++;
    }
}

/* 1 once the note has died away: silent for a tenth of a second after its envelopes and echoes are done */
static int alea_voice_silent(const alea_vm_t *vm, const alea_voice_t *v)
{
    uint32_t t;
    if (!v->active) return 1;
    if (v->gate && vm->gated) return 0;
    t = (uint32_t)(v->time_acc >> 32);
    if (!v->gate && vm->gated) {
        uint32_t off = (uint32_t)(((uint64_t)v->off_at << ALEA_Q) / ALEA_FS);
        if (t < off + vm->release_q16 + vm->echo_q16) return 0;
    } else if (t < vm->struck_q16 + vm->echo_q16) return 0;
    return v->quiet > ALEA_FS / 10;
}

#endif
