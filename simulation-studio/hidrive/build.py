"""Build the isolated esmini 3.8.1 executable with the supplied HiDrive controller.

ControllerHiDrive.cpp/.hpp (the version ported to the esmini 3.8.1 API) are
copied as opaque compiler inputs: this script never opens, parses, hashes or
prints them. Compiler output goes to a log file only, so diagnostics quoting the
private sources are not echoed (on failure, only error lines naming
Studio-edited public files are printed). The engine is extracted from the
verified official 3.8.1 source archive; only public esmini files (registration,
option table, build configuration and the Studio perception-noise hook around
controller steps, see PerceptionNoise.hpp) are edited.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent.parent
LAB = PROJECT / 'esmini-lab'
ENGINE = LAB / 'runtime/esmini-hidrive-v3.8.1'
CURRENT = LAB / 'runtime/esmini-full-v3.8.1'
ARCHIVE = LAB / 'downloads/esmini-fullsrc-v3.8.1.zip'
# GitHub release asset digest, recorded in esmini-full-v3.8.1/installation.json.
ARCHIVE_SHA256 = 'dacb0d3c7e8919e27c456d5cbbc3458c07134fa6ce3f84800dc1f0473d77105e'
DEFAULT_SOURCE = PROJECT.parent / 'AEB Controler (esmini 3.8.1)'
TAG = 'v3.8.1'
PRIVATE_NAMES = ('ControllerHiDrive.cpp', 'ControllerHiDrive.hpp')
NOISE_HEADER = ROOT / 'PerceptionNoise.hpp'
NOISE_TARGET = 'EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/StudioPerceptionNoise.hpp'
# Public files touched by Studio; only compiler lines naming them are echoed.
PUBLIC_DIAGNOSTICS = ('StudioPerceptionNoise.hpp', 'ScenarioEngine.cpp', 'playerbase.cpp', 'EnumConfig.hpp')
HIDDEN = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0

# (file, unique upstream text, replacement) for the public engine only.
ENGINE_EDITS = [
    ('EnvironmentSimulator/Modules/Controllers/Controller.hpp',
     '            N_CONTROLLER_TYPES,',
     '            CONTROLLER_TYPE_HIDRIVE,\n            N_CONTROLLER_TYPES,'),
    ('EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioReader.cpp',
     '#include "ControllerACC.hpp"',
     '#include "ControllerACC.hpp"\n#include "ControllerHiDrive.hpp"'),
    ('EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioReader.cpp',
     'void ScenarioReader::LoadControllers()\n{',
     'void ScenarioReader::LoadControllers()\n{\n    RegisterController("HiDriveController", InstantiateControllerHiDrive);'),
    # Studio perception noise: move perceived objects only during the observer's controller step.
    ('EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioEngine.cpp',
     '#include "OSCParameterDistribution.hpp"',
     '#include "OSCParameterDistribution.hpp"\n#include "StudioPerceptionNoise.hpp"'),
    ('EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioEngine.cpp',
     '                scenarioReader->controller_[i]->Step(deltaSimTime);\n',
     '                studio::PerceptionNoise::Inst().Begin(scenarioReader->controller_[i]->GetRoadObject(), entities_.object_, simulationTime_);\n'
     '                scenarioReader->controller_[i]->Step(deltaSimTime);\n'
     '                studio::PerceptionNoise::Inst().End();\n'),
    # esmini 3.x only accepts options listed in its option enum.
    ('EnvironmentSimulator/Modules/CommonMini/EnumConfig.hpp',
     '        CONFIGS_COUNT                    // this must be the last enum value',
     '        STUDIO_PERCEPTION_NOISE,         // Studio\n'
     '        CONFIGS_COUNT                    // this must be the last enum value'),
    ('EnvironmentSimulator/Modules/CommonMini/EnumConfig.hpp',
     '        {"disable_shadows", DISABLE_SHADOWS},\n',
     '        {"disable_shadows", DISABLE_SHADOWS},\n        {"perception_noise", STUDIO_PERCEPTION_NOISE},\n'),
    ('EnvironmentSimulator/Modules/PlayerBase/playerbase.cpp',
     '    opt.AddOption("collision", "Enable global collision detection, potentially reducing performance");',
     '    opt.AddOption("collision", "Enable global collision detection, potentially reducing performance");\n'
     '    opt.AddOption("perception_noise", "Studio perceived-range noise file for the observer controllers", "filename");'),
    # The archive is not a git checkout (and sits inside the project repository).
    ('CMakeLists.txt', 'COMMAND git submodule update --init --recursive',
     'COMMAND ${CMAKE_COMMAND} -E echo "Studio: submodule update skipped (source archive)"'),
]
# Prebuilt dependency packages, reused from the installed full 3.8.1 runtime instead of downloading.
EXTERNALS = ('osg', 'osi', 'sumo', 'implot', 'proj')


def metadata(directory):
    # Filesystem metadata only.
    return {n: {'size': (directory / n).stat().st_size, 'mtime_ns': (directory / n).stat().st_mtime_ns}
            for n in PRIVATE_NAMES}


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def public_errors(log):
    """Compiler errors in Studio-edited public files only; never controller lines."""
    lines = log.read_text(encoding='utf-8', errors='replace').splitlines()
    return [line for line in lines if 'error' in line.lower() and 'ControllerHiDrive' not in line
            and any(name in line for name in PUBLIC_DIAGNOSTICS)]


def run(args, log, cwd=ENGINE):
    with log.open('a', encoding='utf-8') as stream:
        stream.write('\n$ ' + ' '.join(map(str, args)) + '\n')
        stream.flush()
        return subprocess.run([str(a) for a in args], cwd=cwd, stdout=stream,
                              stderr=subprocess.STDOUT, creationflags=HIDDEN).returncode


def find_cmake(explicit):
    cmake = explicit or shutil.which('cmake')
    if not cmake and os.name == 'nt':
        vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
        installation = subprocess.check_output([str(vswhere), '-latest', '-property', 'installationPath'],
                                               text=True, creationflags=HIDDEN).strip()
        candidate = Path(installation) / 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
        cmake = str(candidate) if candidate.is_file() else None
    if not cmake:
        raise RuntimeError('需要 CMake 与 Visual Studio C++ 工具链。')
    return cmake


def extract_engine():
    if file_sha256(ARCHIVE) != ARCHIVE_SHA256:
        raise RuntimeError('esmini 3.8.1 源码包校验失败: ' + str(ARCHIVE))
    staging = ENGINE.with_name(ENGINE.name + '.partial')
    shutil.rmtree(staging, ignore_errors=True)
    with zipfile.ZipFile(ARCHIVE) as archive:
        for member in archive.infolist():
            relative = Path(member.filename).relative_to('esmini')  # every member sits under esmini/
            if member.is_dir() or not relative.parts:
                continue
            target = staging / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(member) as source, target.open('wb') as destination:
                shutil.copyfileobj(source, destination)
    staging.rename(ENGINE)


def prepare_engine():
    if not ENGINE.exists():
        extract_engine()
    for relative, old, new in ENGINE_EDITS:
        path = ENGINE / relative
        content = path.read_text(encoding='utf-8')
        if new in content:
            continue
        if content.count(old) != 1:
            raise RuntimeError('esmini 公共源码结构与预期不同: ' + relative)
        path.write_text(content.replace(old, new), encoding='utf-8')
    shutil.copy2(NOISE_HEADER, ENGINE / NOISE_TARGET)
    for name in EXTERNALS:
        if not (ENGINE / 'externals' / name).exists():
            shutil.copytree(CURRENT / 'externals' / name, ENGINE / 'externals' / name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE,
                        help='Folder containing the 3.8.1 ControllerHiDrive.cpp/.hpp')
    parser.add_argument('--cmake')
    args = parser.parse_args()
    source = args.source.resolve()
    missing = [n for n in PRIVATE_NAMES if not (source / n).is_file()]
    if missing:
        raise SystemExit('缺少控制器文件: ' + ', '.join(missing) + ' (' + str(source) + ')')
    cmake = find_cmake(args.cmake)
    log = LAB / 'evidence/hidrive-build.log'
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text('', encoding='utf-8')
    prepare_engine()
    destination = ENGINE / 'EnvironmentSimulator/Modules/Controllers'
    for name in PRIVATE_NAMES:
        shutil.copy2(source / name, destination / name)  # OS-level copy, contents untouched
    build = ENGINE / 'build'
    if run([cmake, '-S', ENGINE, '-B', build, '-A', 'x64', '-DCMAKE_BUILD_TYPE=Release',
            '-DDOWNLOAD_EXTERNALS=OFF', '-DUSE_OSG=ON', '-DUSE_OSI=ON', '-DUSE_SUMO=ON', '-DUSE_GTEST=OFF',
            '-DUSE_IMPLOT=ON', '-DUSE_PROJ=ON', '-DENABLE_INCLUDE_WHAT_YOU_USE=OFF',
            '-DENABLE_WARNINGS_AS_ERRORS=OFF', '-DBUILD_EXAMPLES=OFF'], log):
        raise SystemExit('CMake 配置失败；日志: ' + str(log))
    if run([cmake, '--build', build, '--config', 'Release', '--target', 'esmini', '--parallel', '6',
            '--', '/verbosity:minimal', '/nologo', '/nodeReuse:false'], log):
        for line in public_errors(log):
            print(line, flush=True)
        raise SystemExit('编译失败（日志可能包含控制器源码片段，请自行查看）: ' + str(log))
    built = build / 'EnvironmentSimulator/Applications/esmini/Release/esmini.exe'
    (ENGINE / 'bin').mkdir(exist_ok=True)
    shutil.copy2(built, ENGINE / 'bin/esmini.exe')
    info = {'engine_tag': TAG, 'engine_archive': str(ARCHIVE), 'engine_archive_sha256': ARCHIVE_SHA256,
            'source_directory': str(source), 'controller_metadata': metadata(source),
            'executable': str(ENGINE / 'bin/esmini.exe'),
            'perception_noise_sha256': file_sha256(NOISE_HEADER),
            'private_sources_inspected': False}
    (ENGINE / 'hidrive-build.json').write_text(json.dumps(info, indent=2) + '\n', encoding='utf-8')
    print('HiDrive esmini ready:', ENGINE / 'bin/esmini.exe', flush=True)


if __name__ == '__main__':
    sys.exit(main())
