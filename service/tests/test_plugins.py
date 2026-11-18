import pytest

from gs_service.mocks import MockShotSource, MockUpdater
from gs_service.plugins import load_entry_point, make_source, make_updater


def test_defaults():
    assert isinstance(make_source("mock"), MockShotSource)
    assert isinstance(make_updater("mock"), MockUpdater)


def test_entry_point():
    assert isinstance(make_source("gs_service.mocks:MockShotSource"), MockShotSource)
    assert load_entry_point("gs_service.mocks:MockShotSource") is MockShotSource
    with pytest.raises(ValueError):
        load_entry_point("nocolon")
    with pytest.raises(ImportError):
        make_source("no_such_pkg.mod:Thing")


async def test_mock_source_emits():
    src = MockShotSource()
    await src.start()
    src.emit()
    it = src.shots().__aiter__()
    shot = await it.__anext__()
    assert shot.ball.speed_mps > 0
    await src.stop()
