#include <string.h>

#include "gs_cobs.h"
#include "gs_crc16.h"
#include "gs_test.h"

static void test_crc(void)
{
    const uint8_t s[] = "123456789";
    CHECK_EQ(gs_crc16(s, 9), 0x29B1);
    CHECK_EQ(gs_crc16(s, 0), 0xFFFF);
    /* incremental == one shot */
    uint16_t c = gs_crc16_update(GS_CRC16_INIT, s, 4);
    c = gs_crc16_update(c, s + 4, 5);
    CHECK_EQ(c, 0x29B1);
}

static void check_vec(const uint8_t *in, size_t n, const uint8_t *exp, size_t en)
{
    uint8_t out[600];
    size_t e = gs_cobs_encode(in, n, out, sizeof(out));
    CHECK_EQ(e, en);
    CHECK(memcmp(out, exp, en) == 0);
    uint8_t dec[600];
    size_t d = gs_cobs_decode(out, e, dec, sizeof(dec));
    CHECK_EQ(d, n);
    CHECK(n == 0 || memcmp(dec, in, n) == 0);
}

static void test_cobs_vectors(void)
{
    /* Examples from the COBS paper / Wikipedia. */
    {
        const uint8_t in[] = {0x00};
        const uint8_t ex[] = {0x01, 0x01};
        check_vec(in, 1, ex, 2);
    }
    {
        const uint8_t in[] = {0x00, 0x00};
        const uint8_t ex[] = {0x01, 0x01, 0x01};
        check_vec(in, 2, ex, 3);
    }
    {
        const uint8_t in[] = {0x11, 0x22, 0x00, 0x33};
        const uint8_t ex[] = {0x03, 0x11, 0x22, 0x02, 0x33};
        check_vec(in, 4, ex, 5);
    }
    {
        const uint8_t in[] = {0x11, 0x22, 0x33, 0x44};
        const uint8_t ex[] = {0x05, 0x11, 0x22, 0x33, 0x44};
        check_vec(in, 4, ex, 5);
    }
    {
        const uint8_t in[] = {0x11, 0x00, 0x00, 0x00};
        const uint8_t ex[] = {0x02, 0x11, 0x01, 0x01, 0x01};
        check_vec(in, 4, ex, 5);
    }
    {
        /* 254 non-zero bytes 0x01..0xFE */
        uint8_t in[254], ex[256];
        for (int i = 0; i < 254; i++) in[i] = (uint8_t)(i + 1);
        ex[0] = 0xFF;
        memcpy(&ex[1], in, 254);
        ex[255] = 0x01;
        check_vec(in, 254, ex, 256);
    }
    {
        /* 255 bytes 0x00..0xFE */
        uint8_t in[255], ex[257];
        for (int i = 0; i < 255; i++) in[i] = (uint8_t)i;
        ex[0] = 0x01;
        ex[1] = 0xFF;
        memcpy(&ex[2], &in[1], 254);
        ex[256] = 0x01;
        check_vec(in, 255, ex, 257);
    }
    {
        const uint8_t ex[] = {0x01};
        check_vec(NULL, 0, ex, 1);
    }
}

static void test_cobs_roundtrip(void)
{
    static uint8_t in[2000], enc[2100], dec[2000];
    for (int iter = 0; iter < 500; iter++) {
        size_t n = gs_rand() % sizeof(in);
        int zero_bias = (int)(gs_rand() % 4);
        for (size_t i = 0; i < n; i++) {
            uint32_t r = gs_rand();
            in[i] = (zero_bias == 0 && (r & 7) == 0) ? 0 : (uint8_t)(r >> 8);
            if (zero_bias == 3) in[i] |= 1; /* long zero-free runs */
        }
        size_t e = gs_cobs_encode(in, n, enc, sizeof(enc));
        CHECK(e > 0 && e <= GS_COBS_MAX_ENCODED(n));
        CHECK(memchr(enc, 0, e) == NULL);
        size_t d = gs_cobs_decode(enc, e, dec, sizeof(dec));
        CHECK_EQ(d, n);
        CHECK(memcmp(in, dec, n) == 0);
        /* in-place */
        d = gs_cobs_decode(enc, e, enc, e);
        CHECK_EQ(d, n);
        CHECK(memcmp(in, enc, n) == 0);
    }
}

static void test_cobs_errors(void)
{
    uint8_t out[16];
    const uint8_t bad1[] = {0x05, 0x11, 0x22}; /* code promises more bytes */
    CHECK(gs_cobs_decode(bad1, 3, out, sizeof(out)) == GS_COBS_ERROR);
    const uint8_t bad2[] = {0x02, 0x00};       /* embedded zero */
    CHECK(gs_cobs_decode(bad2, 2, out, sizeof(out)) == GS_COBS_ERROR);
    const uint8_t ok[] = {0x05, 1, 2, 3, 4};
    CHECK(gs_cobs_decode(ok, 5, out, 3) == GS_COBS_ERROR); /* out too small */
    uint8_t small[2];
    const uint8_t in[] = {1, 2, 3};
    CHECK_EQ(gs_cobs_encode(in, 3, small, sizeof(small)), 0);
}

int main(void)
{
    test_crc();
    test_cobs_vectors();
    test_cobs_roundtrip();
    test_cobs_errors();
    TEST_MAIN_END();
}
