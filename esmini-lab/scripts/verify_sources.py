"""Check every extracted file against its downloaded archive, and inspect PE machine."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import zipfile

root = Path(__file__).resolve().parents[1]
result = {"archives": []}
for archive_name, destination in [("esmini-demo_Windows-v3.8.1.zip", "runtime"),
                                  ("OSC-NCAP-scenarios-15365d18.zip", "sources")]:
    archive = root / "downloads" / archive_name
    missing, changed, count = [], [], 0
    with zipfile.ZipFile(archive) as z:
        for entry in z.infolist():
            if entry.is_dir():
                continue
            count += 1
            path = root / destination / entry.filename
            if not path.is_file():
                missing.append(entry.filename)
            elif hashlib.sha256(path.read_bytes()).digest() != hashlib.sha256(z.read(entry)).digest():
                changed.append(entry.filename)
    result["archives"].append({"archive": archive_name, "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                                "files_checked": count, "missing": missing, "changed": changed})
exe = root / "runtime/esmini-demo/bin/esmini.exe"
data = exe.read_bytes()
pe_offset = struct.unpack_from("<I", data, 0x3C)[0]
assert data[pe_offset:pe_offset + 4] == b"PE\0\0"
machine = struct.unpack_from("<H", data, pe_offset + 4)[0]
result["executable"] = {"path": str(exe), "machine_hex": hex(machine),
                         "architecture": "AMD64/x64" if machine == 0x8664 else "other",
                         "sha256": hashlib.sha256(data).hexdigest()}
result["passed"] = machine == 0x8664 and all(not a["missing"] and not a["changed"] for a in result["archives"])
(root / "evidence/source-integrity.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
print(json.dumps(result, indent=2))
sys.exit(0 if result["passed"] else 1)
