"""ShotSource implementations: no-argument construction, async life cycle, health."""
import asyncio

import pytest
from gs_common import DeviceState, ShotSource, StrobeConfig

from gs_core.source import PipelineShotSource, SyntheticShotSource, load_object
from gs_core.synthetic.scene import PRESETS


def test_entry_point_constructs_without_arguments():
    src = load_object("gs_core.source:SyntheticShotSource")
    assert isinstance(src, ShotSource)
    assert src.status().state == DeviceState.IDLE


@pytest.mark.slow
async def test_synthetic_source_end_to_end():
    src = SyntheticShotSource(seed=3)
    await src.start()
    try:
        await src.arm()
        for _ in range(200):
            if src.status().ball_detected:
                break
            await asyncio.sleep(0.1)
        st = src.status()
        assert st.ball_detected and st.state == DeviceState.READY
        h = st.health
        assert h["calibration"]["ok"] and h["mcu"]["ok"]
        assert h["camera_cam0"]["frame_age_s"] is not None
        src.inject(PRESETS["iron7"], club="iron")
        src.inject(PRESETS["putt"], club="putter")
        it = src.shots().__aiter__()
        s1 = await asyncio.wait_for(it.__anext__(), 120)
        assert abs(s1.ball.speed_mps - 54) < 0.6 and s1.quality.spin_measured
        assert "synthetic" in s1.quality.sources
        try:
            s2 = await asyncio.wait_for(it.__anext__(), 120)
        except asyncio.TimeoutError:
            pytest.fail(f"no putt shot: {src.health()['pipeline']} {[(t.label, r and r.reason) for t, r in src.results]}")
        assert abs(s2.ball.speed_mps - 3.0) < 0.1 and not s2.quality.spin_measured
        assert src.health()["pipeline"]["shots"] == 2
        await src.disarm()
        assert src.status().state == DeviceState.IDLE
    finally:
        await src.stop()


class _NullMcu:
    """Minimal McuLink stand-in that never triggers (for wiring tests)."""
    def __init__(self):
        self.cfg = None
    def open(self): pass
    def close(self): pass
    def info(self):
        from gs_common import McuInfo
        return McuInfo("x", 0, "s", 0)
    def configure(self, c: StrobeConfig): self.cfg = c
    def arm(self, a): pass
    def fire(self): pass
    def on_trigger(self, cb): pass
    def status(self): return {"ok": True}
    def reboot_to_bootloader(self): pass


async def test_pipeline_source_without_calibration_reports_error(tmp_path):
    src = PipelineShotSource(cameras=[], mcu=_NullMcu(), calibration=str(tmp_path / "missing.json"))
    await src.start()
    try:
        st = src.status()
        assert st.state == DeviceState.ERROR and not st.health["calibration"]["ok"]
    finally:
        await src.stop()


def test_apply_config_sets_speed_hint_and_marking():
    src = SyntheticShotSource()
    src.apply_config({"club": "wedge", "ball_marking": "line"})
    assert src.speed_hint() == 40.0 and src.proc_cfg.spin.marked
