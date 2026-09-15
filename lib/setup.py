"""
Build the LIDAR and hardware-controller pybind11 modules.

Usage (from project root):
    python lib/setup.py build_ext --inplace
"""

import json
import os
import sysconfig
from pathlib import Path

import pybind11
from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

lib_dir = Path(__file__).resolve().parent
project_root = lib_dir.parent

# Get pybind11 include path
pybind11_include = pybind11.get_include()

# SDK paths (relative to project root)
sdk_include = str(project_root / "rplidar_sdk/sdk/include")
sdk_src = str(project_root / "rplidar_sdk/sdk/src")
sdk_lib = str(project_root / "rplidar_sdk/output/Linux/Release")

# Define the extension module
lidar_module = Extension(
    'lib.lidar',
    sources=[
        str(lib_dir / 'lidar_module.cpp'),
        str(lib_dir / 'localisation.cpp'),
    ],
    include_dirs=[
        pybind11_include,
        sdk_include,
        sdk_src,
        str(lib_dir),
    ],
    library_dirs=[sdk_lib],
    libraries=['sl_lidar_sdk', 'pthread', 'rt'],
    extra_compile_args=['-std=c++11', '-O2', '-fPIC'],
    language='c++',
)


hardware_module = Extension(
    'lib.hardware_controller',
    sources=[str(lib_dir / name) for name in (
        'hardware_module.cpp', 'hardware_controller.cpp',
        'PowerfulBLDCdriver.cpp', 'linux_wire.cpp',
        'linux_kicker.cpp', 'status_display.cpp', 'imu/linux_bno08x.cpp', 'imu/sh2.c', 'imu/shtp.c',
        'imu/sh2_SensorValue.c', 'imu/sh2_util.c',
    )],
    include_dirs=[pybind11_include, str(lib_dir)],
    libraries=['pthread'],
    extra_compile_args=['-std=c++17', '-O2', '-fPIC'],
    language='c++',
)


class BuildExtWithCompileCommands(build_ext):
    """Build the extension and emit compile_commands.json for IDE/clangd."""

    def build_extensions(self):
        # The vendor SH-2 core is C, not C++; keep its C initializers and linkage.
        original_compile = self.compiler._compile

        def compile_source(obj, src, ext, cc_args, extra_postargs, pp_opts):
            flags = list(extra_postargs or [])
            if Path(src).suffix == ".c":
                flags = [flag for flag in flags if not flag.startswith("-std=")]
                flags.append("-std=c11")
            return original_compile(obj, src, ext, cc_args, flags, pp_opts)

        self.compiler._compile = compile_source
        try:
            super().build_extensions()
        finally:
            self.compiler._compile = original_compile
        python_include = sysconfig.get_path("include")
        commands = []
        for ext in self.extensions:
            compile_flags = list(ext.extra_compile_args or [])
            for include_dir in ext.include_dirs:
                compile_flags.append(f"-I{include_dir}")
            compile_flags.append(f"-I{python_include}")
            for source in ext.sources:
                source_path = Path(source)
                source_flags = list(compile_flags)
                is_c = source_path.suffix == ".c"
                if is_c:
                    source_flags = [flag for flag in source_flags if not flag.startswith("-std=")]
                    source_flags.append("-std=c11")
                commands.append(
                    {
                        "directory": str(project_root),
                        "command": " ".join(
                            ["gcc" if is_c else "g++", *source_flags, "-c", str(source_path)]
                        ),
                        "file": str(source_path),
                    }
                )
        (project_root / "compile_commands.json").write_text(
            json.dumps(commands, indent=2) + "\n"
        )


setup(
    name='soccer-hardware',
    version='1.0',
    description='Soccer LIDAR and motor hardware modules',
    ext_modules=([hardware_module] if os.environ.get("SOCCER_HARDWARE_ONLY") == "1"
                 else [lidar_module, hardware_module]),
    cmdclass={'build_ext': BuildExtWithCompileCommands},
)
