"""Build esmini 3.8.1 from the official source archive, with the controllers in this repository.

Works on Windows (Visual Studio) and macOS (Xcode command line tools). Controller sources are
copied as opaque compiler inputs: this script never opens, parses, hashes or prints them.
Compiler output goes to a log file only, and this script never reads that log.
Only public esmini files are edited: controller registration, the --perception_noise option and
the error hook around each controller step (see PerceptionNoise.hpp).
"""
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import urllib.request
import zipfile

from .. import controllers as controller_registry
from ..common import BUILD_INFO, ENGINE, ESMINI, EXE_SUFFIX, NO_WINDOW, NOISE_HEADER, RUNTIME, WINDOWS, sha256, write_json

TAG = 'v3.8.1'
ARCHIVE = RUNTIME / 'downloads' / f'esmini-fullsrc-{TAG}.zip'
ARCHIVE_URL = f'https://github.com/esmini/esmini/releases/download/{TAG}/esmini-fullsrc.zip'
ARCHIVE_SHA256 = 'dacb0d3c7e8919e27c456d5cbbc3458c07134fa6ce3f84800dc1f0473d77105e'
BUILD = ENGINE / 'build'
CONTROLLER_DIR = 'EnvironmentSimulator/Modules/Controllers'
NOISE_TARGET = 'EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/StudioPerceptionNoise.hpp'
SCENARIO_READER = 'EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioReader.cpp'
TARGETS = ('esmini', 'replayer')

# (file, unique upstream text, replacement). The hook wraps every controller, whichever it is.
PUBLIC_EDITS = [
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
    # The archive is not a git checkout.
    ('CMakeLists.txt', 'COMMAND git submodule update --init --recursive',
     'COMMAND ${CMAKE_COMMAND} -E echo "Submodule update skipped (source archive)"'),
]


def controller_edits(selected):
    """Registration edits for the selected controllers."""
    if not selected:
        return []
    includes = ''.join(f'\n#include "{c.header}"' for c in selected)
    registrations = ''.join(f'\n    RegisterController("{c.esmini_name}", {c.factory});' for c in selected)
    edits = [(SCENARIO_READER, '#include "ControllerACC.hpp"', '#include "ControllerACC.hpp"' + includes),
             (SCENARIO_READER, 'void ScenarioReader::LoadControllers()\n{',
              'void ScenarioReader::LoadControllers()\n{' + registrations)]
    enums = ''.join(f'            {c.type_enum},\n' for c in selected if c.type_enum)
    if enums:
        edits.append((f'{CONTROLLER_DIR}/Controller.hpp', '            N_CONTROLLER_TYPES,',
                      enums + '            N_CONTROLLER_TYPES,'))
    return edits


def apply_edits(pristine, edits, relative):
    text = pristine.replace('\r\n', '\n')
    for old, new in edits:
        if text.count(old) != 1:
            raise RuntimeError(f'esmini source differs from what this script expects: {relative}')
        text = text.replace(old, new)
    return text


def download_archive():
    if ARCHIVE.is_file() and sha256(ARCHIVE) == ARCHIVE_SHA256:
        return
    ARCHIVE.parent.mkdir(parents=True, exist_ok=True)
    print(f'Downloading {ARCHIVE_URL}', flush=True)
    temporary = ARCHIVE.with_suffix('.partial')
    with urllib.request.urlopen(ARCHIVE_URL, timeout=120) as response, temporary.open('wb') as stream:
        shutil.copyfileobj(response, stream)
    if sha256(temporary) != ARCHIVE_SHA256:
        temporary.unlink()
        raise RuntimeError('The downloaded esmini archive failed its checksum.')
    temporary.replace(ARCHIVE)


def extract_engine():
    if ENGINE.exists():
        return
    print(f'Extracting esmini to {ENGINE}', flush=True)
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


def patch_engine(selected):
    """Rewrite each edited file from the archive, so adding or removing a controller is clean."""
    by_file = {}
    for relative, old, new in PUBLIC_EDITS + controller_edits(selected):
        by_file.setdefault(relative, []).append((old, new))
    # A file that lost all its edits goes back to the archive version.
    all_controllers = list(controller_registry.all_controllers().values())
    for relative, _, _ in controller_edits(all_controllers):
        by_file.setdefault(relative, [])
    with zipfile.ZipFile(ARCHIVE) as archive:
        for relative, edits in by_file.items():
            pristine = archive.read(f'esmini/{relative}').decode('utf-8')
            text = apply_edits(pristine, edits, relative)
            path = ENGINE / relative
            if path.read_bytes() != text.encode('utf-8'):
                path.write_bytes(text.encode('utf-8'))
    shutil.copy2(NOISE_HEADER, ENGINE / NOISE_TARGET)


def install_controllers(selected):
    """Copy sources (OS-level copy, contents untouched); remove copies of controllers no longer used."""
    destination = ENGINE / CONTROLLER_DIR
    wanted = {name for c in selected for name in c.sources}
    previous = []
    if BUILD_INFO.is_file():
        previous = json.loads(BUILD_INFO.read_text(encoding='utf-8')).get('installed_sources', [])
    for name in previous:
        if name not in wanted and (destination / name).is_file():
            (destination / name).unlink()
    for controller in selected:
        for name in controller.sources:
            shutil.copy2(controller.folder / name, destination / name)
    return sorted(wanted)


def find_cmake(explicit=None):
    cmake = explicit or shutil.which('cmake')
    if not cmake and WINDOWS:
        vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
        if vswhere.is_file():
            installation = subprocess.check_output([str(vswhere), '-latest', '-property', 'installationPath'],
                                                   text=True, creationflags=NO_WINDOW).strip()
            candidate = Path(installation) / 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
            cmake = str(candidate) if candidate.is_file() else None
    if not cmake:
        raise RuntimeError('CMake not found. Windows: install Visual Studio with "Desktop development with C++". '
                           'macOS: run "xcode-select --install" and "brew install cmake".')
    return cmake


def configure_command(cmake):
    command = [cmake, '-S', ENGINE, '-B', BUILD, '-DCMAKE_BUILD_TYPE=Release', '-DDOWNLOAD_EXTERNALS=ON',
               '-DUSE_OSG=ON', '-DUSE_OSI=ON', '-DUSE_SUMO=ON', '-DUSE_GTEST=OFF', '-DUSE_IMPLOT=ON', '-DUSE_PROJ=ON',
               '-DENABLE_INCLUDE_WHAT_YOU_USE=OFF', '-DENABLE_WARNINGS_AS_ERRORS=OFF', '-DBUILD_EXAMPLES=OFF']
    if WINDOWS:
        command += ['-A', 'x64']
    elif platform.system() == 'Darwin':
        command += [f'-DCMAKE_OSX_ARCHITECTURES={platform.machine()}']
    return command


def compile_command(cmake, jobs):
    command = [cmake, '--build', BUILD, '--config', 'Release', '--target', *TARGETS, '--parallel', str(jobs), '--']
    return command + (['/verbosity:minimal', '/nologo', '/nodeReuse:false'] if WINDOWS else ['-k'])


def run_logged(command, log):
    with log.open('a', encoding='utf-8') as stream:
        stream.write('\n$ ' + ' '.join(map(str, command)) + '\n')
        stream.flush()
        return subprocess.run([str(part) for part in command], cwd=ENGINE, stdout=stream, stderr=subprocess.STDOUT,
                              creationflags=NO_WINDOW).returncode


def install_binaries():
    bin_folder = ENGINE / 'bin'
    bin_folder.mkdir(exist_ok=True)
    for name in TARGETS:
        file_name = name + EXE_SUFFIX
        folder = BUILD / 'EnvironmentSimulator/Applications' / name
        built = next((p for p in (folder / 'Release' / file_name, folder / file_name) if p.is_file()), None)
        if built is None:
            raise RuntimeError(f'Compiled {file_name} not found under {folder}.')
        shutil.copy2(built, bin_folder / file_name)


def build(*, with_controllers=True, cmake=None, jobs=None):
    selected = list(controller_registry.all_controllers().values()) if with_controllers else []
    cmake = find_cmake(cmake)
    jobs = jobs or max(2, (os.cpu_count() or 4) - 1)
    # Without controllers no controller source is involved, so that log is safe to share with anyone.
    log = RUNTIME / ('build.log' if with_controllers else 'build-public.log')
    RUNTIME.mkdir(parents=True, exist_ok=True)
    log.write_text('', encoding='utf-8')
    download_archive()
    extract_engine()
    patch_engine(selected)
    installed = install_controllers(selected)
    write_build_info(selected, installed, complete=False)
    print('Configuring esmini (the first run downloads about 1 GB of dependencies and models)...', flush=True)
    if run_logged(configure_command(cmake), log):
        raise RuntimeError(f'CMake configuration failed. Read the log: {log}')
    print(f'Compiling esmini with {len(selected)} controller(s), {jobs} jobs (the first run takes a few minutes)...', flush=True)
    if run_logged(compile_command(cmake, jobs), log):
        raise RuntimeError(f'Compilation failed. Read the log: {log}')
    install_binaries()
    write_build_info(selected, installed, complete=True)
    print(f'Ready: {ESMINI}', flush=True)


def write_build_info(selected, installed, *, complete):
    write_json(BUILD_INFO, {'engine_tag': TAG, 'engine_archive_sha256': ARCHIVE_SHA256, 'complete': complete,
                            'platform': platform.platform(), 'installed_sources': installed,
                            'controllers': {c.key: c.source_metadata() for c in selected},
                            'perception_noise_sha256': sha256(NOISE_HEADER)})


def check_build(controller):
    """Refuse to run a stale build. Compares file size and time for controllers, never their contents."""
    if not ESMINI.is_file() or not BUILD_INFO.is_file():
        raise RuntimeError('esmini is not built. Run: python -m pipeline setup')
    info = json.loads(BUILD_INFO.read_text(encoding='utf-8'))
    if not info.get('complete'):
        raise RuntimeError('The last esmini build did not finish. Run: python -m pipeline setup')
    if info['perception_noise_sha256'] != sha256(NOISE_HEADER):
        raise RuntimeError('PerceptionNoise.hpp changed. Run: python -m pipeline setup')
    if info['controllers'].get(controller.key) != controller.source_metadata():
        raise RuntimeError(f'Controller {controller.key!r} is new or changed. Run: python -m pipeline setup')
    return info
