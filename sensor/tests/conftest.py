import pytest

from gs_core.synthetic.harness import make_world


@pytest.fixture(scope="session")
def world():
    """True scene (renderers) + the calibration the pipeline sees (with realistic errors)."""
    return make_world(seed=0, marking="logo", calib_error=1.0)


@pytest.fixture(scope="session")
def world_exact():
    return make_world(seed=0, marking="logo", calib_error=0.0)
