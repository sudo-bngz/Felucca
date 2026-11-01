/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Nicolas Bounoughaz */
/* The ALEA engine's bank in flash (eng_alea.c): up to 16 patches compiled by alea, sent from a computer over USB with
 * the editor protocol's ALEA commands (editor_alea.c, web/EDITOR_PROTOCOL.md "ALEA bank"). Read once at power-on into
 * the engine's RAM copy (alea_bank_set); without a valid bank here the engine plays its factory one.
 *
 * Where: flash 0xE8000, the sector above the FM6 voice bank's (fm6_vbank.c), the second of felucca-bsp's FL_XDATA1
 * (a firmware's own data) and like it in the margin the stock firmware's update stages into, only while the stock
 * firmware runs: a later Felucca finds no valid bank there and plays the factory one. Sector: the commit record
 * (storage.c st_hdr_t, type OBJ_ALEA) at 0, the bank's bytes (alea's ALB1, fm1/FORMAT.md) from 256, at most 3584 of
 * them, so the tail from 0xF00 on stays erased (the record scans of the update loader and the SPL).
 *
 * A transfer: BEGIN erases the sector (the bank stored before is gone from then on; the engine plays it on until END),
 * WRITEs program the bytes, END reads them back, checks the CRC and that every patch loads, and writes the commit record
 * last. A power cut or an unfinished transfer leaves no bank (the factory one at the next power-on), never half of one.
 * Included by upreset.c, after fm6_vbank.c. */
#if FELUCCA_ALEA
#define ALEA_HOME 0xE8000u
#define ALEA_DATA 256u
#define OBJ_ALEA 33u                           /* (>= OBJ_COUNT, as OBJ_FM6VB: st_load never takes one) */
_Static_assert(ALEA_DATA + ALEA_BANK_MAX == 0xF00u, "the sector's tail (0xF00..) stays erased");

static struct {
    uint32_t size, crc;                        /* the transfer's: the bank's bytes, the CRC of what WRITE programmed */
    uint32_t written;                          /* bytes programmed, in order */
    uint8_t on;
} alea_tx __attribute__((section(".pool")));
static alea_vm_t alea_check_vm __attribute__((section(".pool")));   /* (END: each patch loads) */

/* the bank's bytes are one: its patches load in the engine (eng_alea.c alea_vm_load, a voice's words) */
static int alea_bank_ok(const uint8_t *b, uint32_t len)
{
    alea_table_t t;
    uint32_t k;
    memset(&t, 0, sizeof t);
    if (!alea_bank_parse(b, len, &t))
        return 0;
    for (k = 0; k < t.n; k++)
        if (alea_vm_load(&alea_check_vm, b + t.at[k], t.size[k]) || alea_voice_words(&alea_check_vm) > ALEA_VOICE_WORDS)
            return 0;
    return 1;
}

#if FELUCCA_FLASH
/* the bank at ALEA_HOME: its commit record and payload valid -> 0, the bytes in st_buf, h->len of them */
static int alea_flash_valid(st_hdr_t *h)
{
    if (st_read(ALEA_HOME, h, sizeof *h) || h->magic != ST_MAGIC || h->type != OBJ_ALEA || h->slot != 0u ||
        !h->len || h->len > ALEA_BANK_MAX || h->hcrc != st_crc32(h, sizeof *h - 4u) ||
        st_read(ALEA_HOME + ALEA_DATA, st_buf, h->len) || st_crc32(st_buf, h->len) != h->crc)
        return -1;
    return 0;
}

/* up_boot: the stored bank, or the factory one */
static void alea_boot(void)
{
    st_hdr_t h;
    alea_tx.on = 0;
    if (!flash_ok || alea_flash_valid(&h) || !alea_bank_ok(st_buf, h.len) || !alea_bank_set(st_buf, h.len))
        alea_bank_set(0, 0);
}

/* BEGIN: a transfer of size bytes (1..3584), the sector erased; size 0: the stored bank erased, the factory one plays.
 * rc 0, 1 size, 4 flash */
static uint32_t alea_begin(uint32_t size)
{
    alea_tx.on = 0;
    if (size > ALEA_BANK_MAX)
        return 1;
    if (!flash_ok)
        return 4;
    if (st_erase(ALEA_HOME))
        return 4;
    if (!size) {
        alea_bank_set(0, 0);
        return 0;
    }
    alea_tx.size = size;
    alea_tx.written = 0;
    alea_tx.crc = 0xFFFFFFFFu;
    alea_tx.on = 1;
    return 0;
}

/* WRITE: n bytes at off, in order (off = what was written so far). rc 0, 1 not in order / past the size, 4 flash,
 * 5 no transfer */
static uint32_t alea_write(uint32_t off, const uint8_t *d, uint32_t n)
{
    if (!alea_tx.on)
        return 5;
    if (off != alea_tx.written || !n || n > alea_tx.size - off)
        return 1;
    if (fvb_prog(ALEA_HOME + ALEA_DATA + off, d, n)) {
        alea_tx.on = 0;
        return 4;
    }
    alea_tx.crc = st_crc32_add(alea_tx.crc, d, n);
    alea_tx.written += n;
    return 0;
}

/* END: crc, zlib's CRC-32 of the whole bank. The bytes read back and checked, the commit record written, the engine
 * plays the new bank. rc 0, 2 the CRC (or bytes missing), 3 not a bank the engine can play, 4 flash, 5 no transfer */
static uint32_t alea_end(uint32_t crc)
{
    st_hdr_t h;
    if (!alea_tx.on)
        return 5;
    alea_tx.on = 0;
    if (alea_tx.written != alea_tx.size || ~alea_tx.crc != crc)
        return 2;
    if (st_read(ALEA_HOME + ALEA_DATA, st_buf, alea_tx.size) || st_crc32(st_buf, alea_tx.size) != crc)
        return 4;
    if (!alea_bank_ok(st_buf, alea_tx.size))
        return 3;
    h.magic = ST_MAGIC;
    h.type = OBJ_ALEA;
    h.slot = 0;
    h.seq = 1;
    h.len = alea_tx.size;
    h.crc = crc;
    h.rsv[0] = h.rsv[1] = 0xFFFFFFFFu;
    h.hcrc = st_crc32(&h, sizeof h - 4u);
    if (st_prog(ALEA_HOME, &h, sizeof h) || alea_flash_valid(&h))
        return 4;
    return alea_bank_set(st_buf, h.len) ? 0u : 4u;
}
#else
static void alea_boot(void) { alea_bank_set(0, 0); }
static uint32_t alea_begin(uint32_t size) { (void)size; return 4; }
static uint32_t alea_write(uint32_t off, const uint8_t *d, uint32_t n) { (void)off; (void)d; (void)n; return 5; }
static uint32_t alea_end(uint32_t crc) { (void)crc; return 5; }
#endif
#endif
