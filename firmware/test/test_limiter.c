#include "gs_limiter.h"
#include "gs_sched.h"
#include "gs_test.h"

static gs_limiter_t L;

static void test_steady_ok(void)
{
    gs_limiter_init(&L);
    /* 10 us pulse every 1 ms = 1 % duty, 1000 pps: must pass forever */
    int bad = 0;
    for (uint64_t t = 0; t < 3000000; t += 1000) bad |= gs_limiter_pulse(&L, t, 10);
    CHECK_EQ(bad, 0);
    CHECK_EQ(L.max_width_us, 10);
    CHECK(gs_limiter_duty_ppm(&L, 2999000) > 9000 && gs_limiter_duty_ppm(&L, 2999000) < 11000);
    CHECK_EQ(gs_limiter_duty_ppm(&L, 2999000 + 10000000), 0);
}

static void test_duty_trip(void)
{
    gs_limiter_init(&L);
    /* 20 us pulses every 400 us = 5 % duty: must trip on duty within the 100 ms window */
    int r = GS_LIM_OK;
    uint64_t t = 0;
    for (; t < 200000 && r == GS_LIM_OK; t += 400) r = gs_limiter_pulse(&L, t, 20);
    CHECK_EQ(r, GS_LIM_DUTY);
    CHECK(t < 100000 + 2000);
}

static void test_pps_trip(void)
{
    gs_limiter_init(&L);
    /* 5 us pulses every 250 us = 4000 pps, duty 2 % (borderline duty) -> pps limit 3000 trips */
    int r = GS_LIM_OK;
    uint64_t t = 0;
    for (; t < 2000000 && r == GS_LIM_OK; t += 250) r = gs_limiter_pulse(&L, t, 5);
    CHECK(r == GS_LIM_PPS || r == GS_LIM_DUTY);
}

static void test_burst_then_idle(void)
{
    gs_limiter_init(&L);
    int bad = 0;
    /* default burst: 10 pulses, repeated every 500 ms */
    for (int b = 0; b < 20; b++)
        for (int i = 0; i < 10; i++) bad |= gs_limiter_pulse(&L, (uint64_t)b * 500000 + (uint64_t)i * 700, 10);
    CHECK_EQ(bad, 0);
    CHECK_EQ(L.total_pulses, 200);
}

static void test_rail(void)
{
    CHECK_EQ(gs_rail_mv(0, 3300, 4300), 0);
    CHECK_EQ(gs_rail_mv(4095, 3300, 1000), 3299);
    /* 12 V rail: R7 33k/R8 10k -> 2.79 V -> raw ~ 3463 */
    uint32_t mv = gs_rail_mv(3463, 3300, 4300);
    CHECK(mv > 11950 && mv < 12050);
}

int main(void)
{
    test_steady_ok();
    test_duty_trip();
    test_pps_trip();
    test_burst_then_idle();
    test_rail();
    TEST_MAIN_END();
}
