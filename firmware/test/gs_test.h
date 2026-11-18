/* Minimal test harness for the host-side unit tests. */
#ifndef GS_TEST_H
#define GS_TEST_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static int gs_test_failures = 0;
static int gs_test_checks = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        gs_test_checks++;                                                        \
        if (!(cond)) {                                                           \
            gs_test_failures++;                                                  \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                        \
    } while (0)

#define CHECK_EQ(a, b)                                                           \
    do {                                                                         \
        gs_test_checks++;                                                        \
        long long _a = (long long)(a), _b = (long long)(b);                      \
        if (_a != _b) {                                                          \
            gs_test_failures++;                                                  \
            fprintf(stderr, "%s:%d: CHECK_EQ failed: %s (%lld) != %s (%lld)\n",  \
                    __FILE__, __LINE__, #a, _a, #b, _b);                         \
        }                                                                        \
    } while (0)

#define TEST_MAIN_END()                                                          \
    do {                                                                         \
        printf("%d checks, %d failures\n", gs_test_checks, gs_test_failures);    \
        return gs_test_failures ? 1 : 0;                                         \
    } while (0)

/* Deterministic PRNG (xorshift32) so failures are reproducible. */
static uint32_t gs_rng_state = 0x12345678u;
static inline uint32_t gs_rand(void)
{
    uint32_t x = gs_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    gs_rng_state = x;
    return x;
}

#endif
