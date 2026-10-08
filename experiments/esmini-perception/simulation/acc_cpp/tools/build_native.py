#!/usr/bin/env python3
"""Build pinned esmini + the original C++ ACC/AEB + the experiment runner.

Author: Zhuo Ma
Python orchestrates compilation only. The complete simulation loop is C++.
The upstream ACC and R157 AEB files are verified byte-for-byte against the pinned commit.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
COMMIT = "19d26b68f7f473de4e55046a794b5fd2f91c58c8"


def command(args, cwd=None, logfile=None):
    """Run argument arrays without a shell; preserve compiler output on disk."""
    if logfile:
        print("Building; compiler output:", logfile, flush=True)
        with Path(logfile).open("w", encoding="utf-8") as log:
            result = subprocess.run([str(x) for x in args], cwd=cwd, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            print(Path(logfile).read_text(encoding="utf-8", errors="replace")[-8000:], file=sys.stderr)
            raise subprocess.CalledProcessError(result.returncode, args)
    else:
        subprocess.run([str(x) for x in args], cwd=cwd, check=True)


def output(args, cwd=None):
    return subprocess.check_output([str(x) for x in args], cwd=cwd)


def cmake_path(explicit):
    if explicit:
        return Path(explicit).resolve()
    located = shutil.which("cmake")
    if located:
        return Path(located)
    local = ROOT / "build/tooling/cmake/data/bin" / ("cmake.exe" if os.name == "nt" else "cmake")
    if local.is_file():
        return local
    try:
        import cmake
        return Path(cmake.CMAKE_BIN_DIR) / ("cmake.exe" if os.name == "nt" else "cmake")
    except ImportError:
        raise RuntimeError("Install CMake >=3.21 (or python -m pip install cmake), then rerun.")


def verify_controllers(source):
    """Only the CMake download guard is patched; ACC implementation is intact."""
    hashes = {}
    for name in ("ControllerACC.cpp", "ControllerACC.hpp", "ControllerALKS_R157SM.cpp", "ControllerALKS_R157SM.hpp"):
        relative = "EnvironmentSimulator/Modules/Controllers/" + name
        original = output(["git", "show", COMMIT + ":" + relative], source)
        if (source / relative).read_bytes() != original:
            raise RuntimeError("Upstream controller was modified: " + relative)
        hashes[name] = hashlib.sha256(original).hexdigest()
    return hashes


def patch_build_guard(source):
    """Avoid downloading unrelated ALKS/NCAP test assets in the slim build."""
    path = source / "CMakeLists.txt"
    text = path.read_text(encoding="utf-8")
    start = text.find('if(NOT\n   EXISTS\n   "test/OSC-ALKS-scenarios/.git"')
    if start >= 0:
        end = text.index("endif()", start) + len("endif()")
        guard = ('if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/externals/fmt/.git")\n'
                 '    message(FATAL_ERROR "Initialize pinned fmt submodule before configure")\nendif()')
        path.write_text(text[:start] + guard + text[end:], encoding="utf-8")
    elif "Initialize pinned fmt submodule before configure" not in text:
        raise RuntimeError("Unexpected upstream CMake layout; do not patch another version.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build")
    parser.add_argument("--source-dir", type=Path, help="Reuse a pinned esmini git checkout")
    parser.add_argument("--scenario", type=Path, help="Override the scenario fixture used by tests")
    parser.add_argument("--cmake", help="Explicit CMake executable")
    parser.add_argument("--jobs", type=int, default=min(6, os.cpu_count() or 2))
    parser.add_argument("--skip-tests", action="store_true")
    args = parser.parse_args()
    cmake = cmake_path(args.cmake)
    build = args.build_dir.resolve()
    build.mkdir(parents=True, exist_ok=True)
    source = args.source_dir.resolve() if args.source_dir else build / "_deps/esmini-src"
    if not source.exists():
        source.parent.mkdir(parents=True, exist_ok=True)
        command(["git", "clone", "--depth", "1", "--branch", "v3.8.1", "--filter=blob:none", "--sparse",
                 "https://github.com/esmini/esmini.git", source])
    if output(["git", "rev-parse", "HEAD"], source).decode().strip() != COMMIT:
        raise RuntimeError("Source checkout must be the pinned esmini v3.8.1 commit.")
    if not (source / "EnvironmentSimulator/Modules/Controllers/ControllerACC.cpp").exists():
        command(["git", "sparse-checkout", "set", "EnvironmentSimulator", "support", "externals"], source)
    if not (source / "externals/fmt/.git").exists():
        command(["git", "submodule", "update", "--init", "--depth", "1", "externals/fmt"], source)
    controller_hashes = verify_controllers(source)
    patch_build_guard(source)
    engine_build = build / "engine"
    flags = ["-DUSE_" + name + "=OFF" for name in ("OSG", "OSI", "SUMO", "GTEST", "IMPLOT", "PROJ")]
    command([cmake, "-S", source, "-B", engine_build, "-DCMAKE_BUILD_TYPE=Release", *flags,
             "-DDOWNLOAD_EXTERNALS=OFF", "-DBUILD_EXAMPLES=OFF", "-DBUILD_REPLAYER=OFF", "-DBUILD_ODRPLOT=OFF",
             "-DENABLE_INCLUDE_WHAT_YOU_USE=OFF", "-DCMAKE_VERBOSE_MAKEFILE=OFF",
             "-DACCSIM_BRIDGE_ROOT=" + str(ROOT),
             "-DCMAKE_PROJECT_esmini_INCLUDE=" + str(ROOT / "tools/add_bridge.cmake")], source)
    command([cmake, "--build", engine_build, "--config", "Release", "--target", "esminiLib", "--parallel", args.jobs],
            logfile=build / "engine-build.log")
    home = build / "engine-home"
    (home / "bin").mkdir(parents=True, exist_ok=True)
    pattern = "esminiLib.dll" if os.name == "nt" else "libesminiLib.dylib" if sys.platform == "darwin" else "libesminiLib.so"
    matches = list(engine_build.rglob(pattern))
    if len(matches) != 1:
        raise RuntimeError("Expected one built engine library, found: " + str(matches))
    shutil.copy2(matches[0], home / "bin" / pattern)
    if os.name == "nt":
        for file in matches[0].parent.glob("*.lib"):
            shutil.copy2(file, home / "bin" / file.name)
    header = home / "EnvironmentSimulator/Libraries/esminiLib"
    header.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source / "EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp", header / "esminiLib.hpp")
    (home / "version.txt").write_text('ESMINI_GIT_TAG="v3.8.1"\nESMINI_GIT_REV="' + COMMIT + '"\n', encoding="utf-8")
    manifest = {"commit": COMMIT, "controller_source_unchanged": True, "controller_sha256": controller_hashes,
                "abi_version": 2, "controllers": ["acc", "aeb"],
                "bridge_sha256": {name: hashlib.sha256((ROOT / "src" / name).read_bytes()).hexdigest()
                                  for name in ("native_acc_bridge.cpp", "native_aeb_bridge.cpp")}}
    (home / "accsim_bridge.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    runner = build / "runner"
    configure = [cmake, "-S", ROOT, "-B", runner, "-DCMAKE_BUILD_TYPE=Release", "-DESMINI_HOME=" + str(home)]
    if args.scenario:
        configure += ["-DACCSIM_TEST_SCENARIO=" + str(args.scenario.resolve())]
    command(configure)
    command([cmake, "--build", runner, "--config", "Release", "--parallel", args.jobs], logfile=build / "runner-build.log")
    if not args.skip_tests:
        ctest = cmake.parent / ("ctest.exe" if os.name == "nt" else "ctest")
        command([ctest, "--test-dir", runner, "-C", "Release", "--output-on-failure"])
    name = "acc_sim.exe" if os.name == "nt" else "acc_sim"
    binary = runner / "Release" / name if os.name == "nt" else runner / name
    print("Ready:", binary)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, OSError) as error:
        print("Build failed:", error, file=sys.stderr)
        sys.exit(1)
