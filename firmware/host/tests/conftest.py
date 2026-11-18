import os
import shutil
import subprocess
from pathlib import Path

import pytest

FIRMWARE = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def vectors_tool(tmp_path_factory) -> str:
    """Path of the C test-vector tool (built with cmake + the system compiler)."""
    env = os.environ.get("GS_VECTORS")
    if env:
        return env
    if not shutil.which("cmake") or not (shutil.which("gcc") or shutil.which("cc")):
        pytest.skip("cmake/gcc not available to build the C vector tool")
    build = tmp_path_factory.mktemp("vectors-build")
    subprocess.run(["cmake", "-S", str(FIRMWARE / "test"), "-B", str(build), "-DGS_SANITIZE=OFF",
                    "-DCMAKE_BUILD_TYPE=Release"], check=True, capture_output=True)
    subprocess.run(["cmake", "--build", str(build), "--target", "gs_vectors"], check=True, capture_output=True)
    for cand in build.rglob("gs_vectors"):
        if cand.is_file():
            return str(cand)
    pytest.fail("gs_vectors not built")
