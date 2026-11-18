"""End-to-end update transactions against a local HTTP server and temp slots."""
import os
from pathlib import Path

import pytest

from conftest import Crash, read_json

from gs_ota.slots import SlotManager


def slots_of(make_engine):
    return SlotManager(make_engine.root)


def test_bootstrap_layout(bootstrapped, make_engine):
    eng, _, _ = bootstrapped
    s = slots_of(make_engine)
    assert s.current() == s.committed() == "a"
    assert os.readlink(make_engine.root / "current") == "slots/a"
    rel = read_json(make_engine.root / "slots/a/release.json")
    assert rel["version"] == "1.0.0"
    assert (make_engine.root / "slots/a/share/web/index.html").exists()
    assert (make_engine.root / "slots/a/bin/gs-python").stat().st_mode & 0o111
    assert eng.state().current == {"sensor": "1.0.0"}


def test_happy_path_update(bootstrapped, release, make_engine):
    eng, svc, hp = bootstrapped
    release.publish(sensor="1.1.0")
    events = []
    eng.on_change(lambda st: events.append(st.phase))
    st = eng.check()
    assert st.phase == "available" and st.available == {"sensor": "1.1.0"}
    assert st.notes == "notes"
    assert eng.apply() is True
    s = slots_of(make_engine)
    assert s.current() == s.committed() == "b"
    assert s.version("b") == "1.1.0" and s.version("a") == "1.0.0"   # old slot kept
    assert eng.state().phase == "up_to_date" and eng.state().current["sensor"] == "1.1.0"
    assert eng.store.txn is None and eng.store.data["installed"]["sensor"] == "1.1.0"
    assert svc.calls == ["restart"]
    for phase in ("checking", "available", "downloading", "verifying", "installing",
                  "rebooting", "up_to_date"):
        assert phase in events
    # compileall ran inside the slot while it was writable
    assert list((make_engine.root / "slots/b/lib/gs_ota").glob("__pycache__/*.pyc"))
    # next update goes into slot a again
    release.publish(sensor="1.2.0", released="2026-11-17T00:00:00Z")
    assert eng.apply() is True
    assert s.current() == s.committed() == "a" and s.version("a") == "1.2.0"
    assert eng.check().phase == "up_to_date"


def test_tampered_manifest_rejected(bootstrapped, release, make_engine):
    eng, svc, _ = bootstrapped
    d = release.publish(sensor="1.1.0")
    data = (d / "manifest.json").read_bytes()
    (d / "manifest.json").write_bytes(data.replace(b'"notes"', b'"NOTES"', 1))
    st = eng.check()
    assert st.phase == "failed" and "signature" in st.message
    assert st.available == {}
    assert eng.apply() is False
    assert slots_of(make_engine).current() == "a" and svc.calls == []


def test_wrong_key_rejected(bootstrapped, release):
    from gs_ota import keys as K
    eng, _, _ = bootstrapped
    release.publish(sensor="1.1.0", sign_key=K.generate_ed25519())
    assert eng.check().phase == "failed"


def test_tampered_artifact_rejected(bootstrapped, release, make_engine):
    eng, svc, _ = bootstrapped
    d = release.publish(sensor="1.1.0")
    f = d / "sensor-1.1.0.tar.zst"
    b = bytearray(f.read_bytes())
    b[len(b) // 2] ^= 0x55
    f.write_bytes(bytes(b))
    assert eng.apply() is False
    st = eng.state()
    assert st.phase == "failed" and "sha256" in st.message
    s = slots_of(make_engine)
    assert s.current() == s.committed() == "a" and svc.calls == []
    assert eng.store.txn is None
    assert not list((make_engine.state / "downloads").glob("sensor-1.1.0*"))


def test_truncated_artifact_on_server(bootstrapped, release, make_engine):
    eng, svc, _ = bootstrapped
    d = release.publish(sensor="1.1.0")
    f = d / "sensor-1.1.0.tar.zst"
    f.write_bytes(f.read_bytes()[:-100])
    assert eng.apply() is False
    assert "incomplete" in eng.state().message
    assert slots_of(make_engine).current() == "a" and svc.calls == []


def test_downgrade_refused(bootstrapped, release, make_engine):
    eng, svc, _ = bootstrapped
    release.publish(sensor="0.9.0")
    st = eng.check()
    assert st.phase == "up_to_date" and "downgrade" in st.message and st.available == {}
    assert eng.apply() is False
    assert slots_of(make_engine).version("a") == "1.0.0" and svc.calls == []


def test_replayed_old_manifest_refused(bootstrapped, release):
    eng, _, _ = bootstrapped
    release.publish(sensor="1.2.0", released="2026-11-20T00:00:00Z")
    assert eng.check().phase == "available"
    # attacker / stale mirror serves an older, validly signed manifest
    release.publish(sensor="1.1.0", released="2026-11-10T00:00:00Z")
    st = eng.check()
    assert st.phase == "failed" and "stale manifest" in st.message
    assert eng.store.data["highest_released"]["stable"] == "2026-11-20T00:00:00Z"
    assert eng.store.data["highest_seen"]["sensor"] == "1.2.0"


def test_min_version_gate(bootstrapped, release):
    eng, _, _ = bootstrapped
    release.publish(sensor="2.0.0", sensor_min="1.5.0")
    st = eng.check()
    assert st.phase == "up_to_date" and "1.5.0" in st.message
    release.publish(sensor="2.0.0", sensor_min="1.0.0", released="2026-11-17T00:00:00Z")
    assert eng.check().available == {"sensor": "2.0.0"}


def test_channel_mismatch_and_channels(bootstrapped, release, make_engine):
    eng, _, _ = bootstrapped
    release.publish(sensor="1.1.0", channel="stable", manifest_channel="beta")
    st = eng.check()
    assert st.phase == "failed" and "channel" in st.message
    release.publish(sensor="1.3.0-beta.1", channel="beta")
    beta, _, _ = make_engine(channel="beta")
    assert beta.check().available == {"sensor": "1.3.0-beta.1"}


def test_failed_health_rolls_back(bootstrapped, release, make_engine):
    eng, svc, hp = bootstrapped
    release.publish(sensor="1.1.0")
    hp.broken.add("1.1.0")
    assert eng.apply() is False
    s = slots_of(make_engine)
    assert s.current() == s.committed() == "a"
    st = eng.state()
    assert st.phase == "failed" and "rolled back" in st.message
    assert st.current["sensor"] == "1.0.0"
    assert svc.calls == ["restart", "restart"]           # new, then back to old
    assert eng.store.data["failed_versions"] == {"sensor": ["1.1.0"]}
    assert eng.store.txn is None
    # auto-apply does not retry a version that was rolled back; manual apply may
    assert eng.apply(auto=True) is False
    hp.broken.clear()
    assert eng.apply() is True and s.current() == "b"


def test_smoke_test_failure_never_switches(bootstrapped, release, make_engine):
    eng, svc, _ = bootstrapped
    release.publish(sensor="1.1.0")
    eng.smoke_runner = lambda cmd, t: (1, "ImportError: boom") if "-c" in cmd else (0, "")
    assert eng.apply() is False
    assert "smoke test failed" in eng.state().message
    s = slots_of(make_engine)
    assert s.current() == "a" and not (make_engine.root / "slots/b/release.json").exists()
    assert svc.calls == []


def test_unsafe_bundle_rejected(bootstrapped, release, make_engine, tmp_path):
    """A validly signed but malicious/buggy bundle with path traversal."""
    import io
    import tarfile
    import zstandard
    from gs_ota import keys as K
    from gs_ota.download import sha256_file
    from gs_ota.manifest import build_manifest_obj, dump_manifest
    eng, svc, _ = bootstrapped
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tf:
        ti = tarfile.TarInfo("../../../../escaped")
        ti.size = 1
        tf.addfile(ti, io.BytesIO(b"x"))
    d = release.srv / "stable"
    d.mkdir(parents=True, exist_ok=True)
    f = d / "sensor-1.1.0.tar.zst"
    f.write_bytes(zstandard.ZstdCompressor().compress(buf.getvalue()))
    data = dump_manifest(build_manifest_obj("stable", "2026-11-16T00:00:00Z", {"sensor": {
        "version": "1.1.0", "url": f.name, "sha256": sha256_file(f), "size": f.stat().st_size}}))
    (d / "manifest.json").write_bytes(data)
    (d / "manifest.json.sig").write_bytes(K.sign_ed25519(release.keypair[0], data))
    assert eng.apply() is False
    assert "path traversal" in eng.state().message
    assert slots_of(make_engine).current() == "a"
    assert not list(tmp_path.rglob("escaped"))


# ------------------------------------------------------- interruption recovery
def _restart(make_engine, eng, **kw):
    """Simulate the agent process being restarted (new engine, same disk)."""
    eng2, svc2, hp2 = make_engine(**kw)
    return eng2, svc2, hp2


def test_crash_during_download_then_resume(bootstrapped, release, make_engine):
    eng, _, _ = bootstrapped
    release.publish(sensor="1.1.0")
    real = eng.http

    class CrashingClient:
        def __init__(self):
            self.n = 0

        def stream(self, *a, **kw):
            return real.stream(*a, **kw)

        def __getattr__(self, k):
            return getattr(real, k)
    eng.http = CrashingClient()
    orig_set = eng._set

    def crash_mid_download(phase=None, progress=None, message=None):
        orig_set(phase, progress, message)
        if eng._state.phase == "downloading" and eng._state.progress > 0.2:
            raise Crash("power loss during download")
    eng._set = crash_mid_download
    with pytest.raises(Crash):
        eng.apply()
    assert eng.store.txn["phase"] == "download"
    part = list((make_engine.state / "downloads").glob("*.part"))
    assert part and part[0].stat().st_size > 0
    eng2, svc2, _ = _restart(make_engine, eng)
    assert eng2.recover() == "aborted pre-switch"
    assert eng2.store.txn is None and slots_of(make_engine).current() == "a"
    assert eng2.state().phase == "failed" and "interrupted" in eng2.state().message
    assert svc2.calls == ["start"]
    assert eng2.apply() is True                       # resumes the .part
    assert slots_of(make_engine).current() == "b"


@pytest.mark.parametrize("crash_at", ["compileall", "smoke"])
def test_crash_during_staging(bootstrapped, release, make_engine, crash_at):
    eng, _, _ = bootstrapped
    release.publish(sensor="1.1.0")
    real = eng.smoke_runner

    def runner(cmd, t):
        if (crash_at == "compileall") == ("compileall" in cmd):
            raise Crash(crash_at)
        return real(cmd, t)
    eng.smoke_runner = runner
    with pytest.raises(Crash):
        eng.apply()
    assert eng.store.txn["phase"] == "stage"
    assert (make_engine.root / "slots/b/release.json").exists()
    eng2, _, _ = _restart(make_engine, eng)
    eng2.recover()
    assert not (make_engine.root / "slots/b").exists()
    assert slots_of(make_engine).current() == "a" and eng2.store.txn is None


def test_crash_during_switch(bootstrapped, release, make_engine):
    eng, _, _ = bootstrapped
    release.publish(sensor="1.1.0")

    def boom(slot):
        raise Crash("before rename")
    eng.slots.switch_current = boom
    with pytest.raises(Crash):
        eng.apply()
    assert eng.store.txn["phase"] == "switch"
    eng2, svc2, _ = _restart(make_engine, eng)
    assert eng2.recover() == "aborted pre-switch"
    assert slots_of(make_engine).current() == "a" and svc2.calls == ["start"]


def test_crash_after_switch_before_restart_recovers_and_commits(bootstrapped, release, make_engine):
    eng, svc, _ = bootstrapped
    release.publish(sensor="1.1.0")
    svc.crash_on, svc.crash_count = "restart", 1
    with pytest.raises(Crash):
        eng.apply()
    s = slots_of(make_engine)
    assert eng.store.txn["phase"] == "health" and s.current() == "b" and s.committed() == "a"
    eng2, svc2, _ = _restart(make_engine, eng)
    assert eng2.recover() == "committed"
    assert s.current() == s.committed() == "b"
    assert eng2.state().current["sensor"] == "1.1.0" and svc2.calls == ["restart"]


def test_repeated_power_loss_in_health_window_rolls_back(bootstrapped, release, make_engine):
    eng, _, hp = bootstrapped
    release.publish(sensor="1.1.0")
    hp.crash_times = 1
    with pytest.raises(Crash):
        eng.apply()
    s = slots_of(make_engine)
    assert s.current() == "b"
    for attempt in range(1, 4):                       # max_boot_attempts = 3
        eng_n, _, hp_n = _restart(make_engine, eng)
        hp_n.crash_times = 1
        with pytest.raises(Crash):
            eng_n.recover()
        assert eng_n.store.txn["boot_attempts"] == attempt
    eng_last, svc_last, _ = _restart(make_engine, eng)
    assert eng_last.recover() == "rolled back (boot attempts)"
    assert s.current() == s.committed() == "a"
    assert "did not confirm health" in eng_last.state().message
    assert svc_last.calls == ["restart"]


def test_crash_during_rollback_is_completed(bootstrapped, release, make_engine):
    eng, svc, hp = bootstrapped
    release.publish(sensor="1.1.0")
    hp.broken.add("1.1.0")
    svc.crash_on, svc.crash_count = "restart", 2      # 1st restart ok? no: crash on 2nd
    calls = {"n": 0}
    orig = svc.restart

    def restart():
        calls["n"] += 1
        if calls["n"] == 2:
            raise Crash("power loss during rollback restart")
        svc.calls.append("restart")
    svc.restart = restart
    with pytest.raises(Crash):
        eng.apply()
    assert eng.store.txn["phase"] == "rollback"
    eng2, svc2, _ = _restart(make_engine, eng)
    assert eng2.recover() == "rollback completed"
    s = slots_of(make_engine)
    assert s.current() == s.committed() == "a" and eng2.store.txn is None
    assert eng2.store.data["failed_versions"]["sensor"] == ["1.1.0"]
    del orig


def test_inconsistent_links_without_txn_reverted(bootstrapped, make_engine, release):
    eng, _, _ = bootstrapped
    release.publish(sensor="1.1.0")
    assert eng.apply()
    s = slots_of(make_engine)
    s.switch_current("a")                              # e.g. manual tampering
    eng2, svc2, _ = _restart(make_engine, eng)
    assert eng2.recover() == "reverted current to committed slot"
    assert s.current() == "b" and svc2.calls == ["restart"]


def test_state_file_corruption_tolerated(bootstrapped, make_engine):
    eng, _, _ = bootstrapped
    (make_engine.state / "state.json").write_text("{not json")
    eng2, _, _ = make_engine()
    assert eng2.recover() == "nothing to recover"
    assert (make_engine.state / "state.corrupt").exists()
    assert eng2.state().current["sensor"] == "1.0.0"   # from the committed slot


def test_disk_full_before_staging(bootstrapped, release, make_engine, monkeypatch):
    import gs_ota.engine as E
    from gs_ota.download import DiskSpaceError
    eng, svc, _ = bootstrapped
    release.publish(sensor="1.1.0")

    def no_space(d, needed, margin=0):
        raise DiskSpaceError("not enough disk space (test)")
    monkeypatch.setattr(E, "check_free_space", no_space)
    assert eng.apply() is False
    assert "disk space" in eng.state().message
    assert slots_of(make_engine).current() == "a" and svc.calls == []
