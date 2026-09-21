"""Hardware-free checks for motion uncertainty, scan timing and particle weights."""

import subprocess
import tempfile
import unittest
from pathlib import Path


class LocalisationMotionTests(unittest.TestCase):
    def run_native_case(self, name):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / name)
            subprocess.run(
                ["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                 "-pthread", str(root / f"tests/{name}_native.cpp"),
                 "-o", executable], check=True,
            )
            subprocess.run([executable], check=True, timeout=30)

    def test_native_timing_and_motion(self):
        self.run_native_case("localisation_motion")

    def test_native_particle_weights(self):
        self.run_native_case("localisation_weights")

    def test_native_deskew(self):
        self.run_native_case("localisation_deskew")
