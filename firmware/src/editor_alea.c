/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Nicolas Bounoughaz */
/* Editor protocol: the ALEA bank (web/EDITOR_PROTOCOL.md "ALEA bank", cmds 96..99; alea_bank.c, eng_alea.c). `alea
 * work.alea --fm1 bank.syx` writes these frames; alea's fm1/send.html (Web MIDI) and fm1/send.py send them one at a
 * time, each after the reply to the one before. Values in 7-bit bytes, the low bits first; the bank's bytes 7 to 8. */
#if FELUCCA_ALEA
enum { ED_ALEA_BEGIN = 96, ED_ALEA_WRITE, ED_ALEA_END, ED_ALEA_LIST };

static uint32_t ed_alea_v(const uint8_t *a, uint32_t n)   /* n 7-bit bytes, the low first */
{
    uint32_t v = 0, k;
    for (k = 0; k < n; k++)
        v |= (uint32_t)a[k] << (7u * k);
    return v;
}

#define ALEA_WRITE_MAX 256u
static uint8_t ed_alea_buf[ALEA_WRITE_MAX] __attribute__((aligned(4), section(".pool")));

/* a frame's bytes 7 to 8 (groups: a byte of high bits, then up to seven) into out; 0 when malformed or over max */
static uint32_t ed_alea_unpack7(const uint8_t *a, uint32_t na, uint8_t *out, uint32_t max)
{
    uint32_t n = 0;
    while (na) {
        uint32_t m = *a++, j, k;
        na--;
        k = na > 7u ? 7u : na;
        if (!k || n + k > max || (m >> k))
            return 0;
        for (j = 0; j < k; j++, na--)
            out[n++] = (uint8_t)(*a++ | ((m >> j) & 1u) << 7);
    }
    return n;
}

static int ed_alea_handle(uint32_t cmd, const uint8_t *a, uint32_t n)
{
    uint32_t i, k, rc;
    switch (cmd) {
    case ED_ALEA_BEGIN:                                    /* size (3 x 7 bits; 0 = erase: the factory bank) -> rc */
        ed_b(n != 3u ? 1u : ed_flash_stop() ? 3u : alea_begin(ed_alea_v(a, 3)));
        return 1;
    case ED_ALEA_WRITE:                                    /* offset (3 x 7 bits), the bytes packed (at most 256) ->
                                                            * offset (3 x 7 bits), rc */
        if (n < 5u) {
            rc = 1;
        } else {
            k = ed_alea_unpack7(a + 3, n - 3u, ed_alea_buf, sizeof ed_alea_buf);
            rc = !k ? 1u : ed_flash_stop() ? 3u : alea_write(ed_alea_v(a, 3), ed_alea_buf, k);
        }
        for (i = 0; i < 3u; i++)
            ed_b(n >= 3u ? a[i] : 127u);
        ed_b(rc);
        return 1;
    case ED_ALEA_END:                                      /* CRC-32 (5 x 7 bits) -> rc, patches */
        rc = n != 5u || a[4] > 15u ? 1u : ed_flash_stop() ? 3u : alea_end(ed_alea_v(a, 5));
        ed_b(rc);
        ed_b(alea_bank.tab.n);
        return 1;
    case ED_ALEA_LIST:                                     /* -> stored (1: sent over USB, 0: the factory bank), patches,
                                                            * per patch its name */
        if (n)
            return 0;
        ed_b(alea_bank.user);
        ed_b(alea_bank.tab.n);
        for (i = 0; i < alea_bank.tab.n; i++)
            ed_str(alea_bank.tab.name[i], 15);
        return 1;
    }
    return 0;
}
#else
static int ed_alea_handle(uint32_t cmd, const uint8_t *a, uint32_t n) { (void)cmd; (void)a; (void)n; return 0; }
#endif
