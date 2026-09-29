"""Install official complete esmini 3.8.1 alongside the original demo package."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
VERSION = 'v3.8.1'
DEST = ROOT / 'runtime' / f'esmini-full-{VERSION}'
DOWNLOADS = ROOT / 'downloads'
EVIDENCE = ROOT / 'evidence' / 'full-install'
MODELS_URL = ('https://www.dropbox.com/scl/fi/3jwz3ie6xkgpga33cbyju/models_with_lights.7z'
              '?rlkey=9obptnofidldoagn4154iyntl&st=zxoupatb&dl=1')
HEADERS = {'User-Agent': 'TME180-esmini-setup', 'Accept': 'application/vnd.github+json'}


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def save_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding='utf-8')


def download(item):
    path = DOWNLOADS / item['name']
    if not path.exists():
        temporary = path.with_suffix(path.suffix + '.part')
        print('Downloading ' + item['name'], flush=True)
        request = urllib.request.Request(item['url'], headers={'User-Agent': HEADERS['User-Agent']})
        with urllib.request.urlopen(request, timeout=60) as response, temporary.open('wb') as stream:
            shutil.copyfileobj(response, stream, length=1024 * 1024)
        temporary.replace(path)
    actual = digest(path)
    if item.get('sha256') and actual != item['sha256']:
        raise ValueError('Checksum mismatch: ' + str(path))
    if item['name'].endswith('.7z'):
        with path.open('rb') as stream:
            if stream.read(6) != b'7z\xbc\xaf\x27\x1c':
                raise ValueError('The model download is not a 7z archive')
    receipt = item | {'sha256': actual, 'bytes': path.stat().st_size,
                      'checksum_source': 'GitHub release asset digest' if item.get('sha256') else 'locally recorded'}
    save_json(EVIDENCE / (item['name'] + '.json'), receipt)
    print(json.dumps(receipt), flush=True)
    return path, receipt


def safe_relative(name):
    parts = PurePosixPath(name.replace('\\', '/'))
    if parts.is_absolute() or '..' in parts.parts or any(':' in piece for piece in parts.parts):
        raise ValueError('Unsafe archive path: ' + name)
    return parts


def unpack_zip(archive):
    with zipfile.ZipFile(archive) as bundle:
        entries = bundle.infolist()
        roots = {safe_relative(e.filename).parts[0] for e in entries if e.filename}
        if len(roots) != 1:
            raise ValueError('Expected a single archive root: ' + str(roots))
        for entry in entries:
            relative = safe_relative(entry.filename)
            if len(relative.parts) < 2:
                continue
            path = DEST.joinpath(*relative.parts[1:])
            if entry.is_dir():
                path.mkdir(parents=True, exist_ok=True)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                with bundle.open(entry) as source, path.open('wb') as output:
                    shutil.copyfileobj(source, output)
    print('Extracted ' + archive.name, flush=True)


def main():
    DOWNLOADS.mkdir(exist_ok=True)
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(
        f'https://api.github.com/repos/esmini/esmini/releases/tags/{VERSION}', headers=HEADERS)
    with urllib.request.urlopen(request, timeout=30) as response:
        release = json.load(response)
    save_json(EVIDENCE / 'release.json', release)
    assets = {asset['name']: asset for asset in release['assets']}
    items = []
    for name in ('esmini-fullsrc.zip', 'esmini-bin_Windows.zip'):
        asset = assets[name]
        algorithm, expected = asset['digest'].split(':', 1)
        if algorithm != 'sha256':
            raise ValueError('Expected a publisher SHA256 digest')
        items.append({'name': name.replace('.zip', f'-{VERSION}.zip'),
                      'url': asset['browser_download_url'], 'sha256': expected})
    items.append({'name': 'esmini-models_with_lights.7z', 'url': MODELS_URL})
    with ThreadPoolExecutor(max_workers=3) as pool:
        downloaded = list(pool.map(download, items))
    if (DEST / 'installation.json').exists():
        print('Complete installation already exists: ' + str(DEST), flush=True)
        return
    DEST.mkdir(parents=True, exist_ok=True)
    for archive, receipt in downloaded[:2]:
        unpack_zip(archive)
    models = downloaded[2][0]
    flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
    listing = subprocess.run(['tar', '-tf', str(models)], capture_output=True, text=True,
                             creationflags=flags, check=True, timeout=120)
    entries = listing.stdout.splitlines()
    for name in entries:
        relative = safe_relative(name)
        if relative.parts and relative.parts[0] != 'models':
            raise ValueError('Unexpected model archive root: ' + name)
    with (EVIDENCE / 'models-extraction.log').open('w', encoding='utf-8') as log:
        subprocess.run(['tar', '-xf', str(models), '-C', str(DEST / 'resources')],
                       stdout=log, stderr=subprocess.STDOUT, creationflags=flags, check=True, timeout=300)
    for relative in ['bin/esmini.exe', 'bin/esminiLib.dll', 'CMakeLists.txt',
                     'EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp', 'resources/xosc/cut-in.xosc']:
        if not (DEST / relative).is_file():
            raise ValueError('Incomplete installation, missing: ' + relative)
    if 'v3.8.1' not in (DEST / 'version.txt').read_text():
        raise ValueError('Version does not match the pinned release')
    inventory = {}
    for name in ('bin', 'resources/xosc', 'resources/xodr', 'resources/models'):
        inventory[name] = len([p for p in (DEST / name).rglob('*') if p.is_file()])
    old = ROOT / 'runtime/esmini-demo'
    shared = {}
    for path in (old / 'bin').iterdir():
        if path.is_file() and (DEST / 'bin' / path.name).is_file():
            shared[path.name] = digest(path) == digest(DEST / 'bin' / path.name)
    receipt = {'installed_utc': datetime.now(timezone.utc).isoformat(), 'directory': str(DEST),
               'version': VERSION, 'method': 'official full source + Windows binaries + complete models',
               'official_guide': 'https://esmini.github.io/getting-started.html#_get_complete_esmini',
               'archives': [item[1] for item in downloaded], 'inventory': inventory,
               'shared_binaries_identical_to_demo': shared,
               'esmini_dll_sha256': digest(DEST / 'bin/esminiLib.dll'),
               'tools': sorted(p.name for p in (DEST / 'bin').iterdir() if p.is_file())}
    save_json(DEST / 'installation.json', receipt)
    save_json(EVIDENCE / 'installation.json', receipt)
    print(json.dumps(receipt, indent=2), flush=True)


if __name__ == '__main__':
    main()
