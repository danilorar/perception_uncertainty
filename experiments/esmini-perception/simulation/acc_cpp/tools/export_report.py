#!/usr/bin/env python3
"""导出精选汇总，不改写原运行 / Export selected summaries without modifying runs.

Author: Zhuo Ma
Only summary CSVs and provenance JSON are exported, with portable path labels.
Frame-level logs, binaries, recordings and generated scenarios stay local.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT.parents[1]
CSV_FILES = ("comparison.csv", "sensitivity_summary.csv", "sensitivity_runs.csv")
JSON_FILES = ("batch_manifest.json", "experiment_manifest.json")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def portable(value, batch):
    """Normalize only absolute path values/keys; retain numbers and other text."""
    if isinstance(value, dict):
        return {portable(k, batch): portable(v, batch) for k, v in value.items()}
    if isinstance(value, list):
        return [portable(v, batch) for v in value]
    if not isinstance(value, str) or not Path(value).is_absolute():
        return value
    path = Path(value)
    for root, label in ((batch, "source_batch"), (PROJECT, "esmini-perception")):
        try:
            return label + "/" + path.relative_to(root).as_posix()
        except ValueError:
            pass
    # Older records may refer to a checkout outside the current project.
    parts = path.parts
    if "esmini-perception" in parts:
        return Path(*parts[parts.index("esmini-perception"):]).as_posix()
    return "external/" + path.name


def export_batch(batch, destination):
    batch = batch.resolve()
    files = [batch / name for name in (*CSV_FILES, *JSON_FILES) if (batch / name).is_file()]
    if not any(f.suffix == ".csv" for f in files) or not any(f.suffix == ".json" for f in files):
        raise ValueError("Need a completed comparison/sensitivity batch with CSV and manifest")
    prepared, row_counts, config_hashes = {}, {}, {}
    for source in files:
        if source.suffix == ".csv":
            with source.open(newline="", encoding="utf-8") as file:
                reader = csv.DictReader(file)
                fields, rows = reader.fieldnames, list(reader)
            if not fields or not rows or any(r.get("complete", "true").lower() != "true" for r in rows):
                raise ValueError("Empty or incomplete summary: " + source.name)
            prepared[source.name] = (fields, [{k: portable(v, batch) for k, v in row.items()} for row in rows])
            row_counts[source.name] = len(rows)
            if source.name == "comparison.csv":
                configs = {}
                for row in rows:
                    if row["model"] not in ("ideal", "dropout"):
                        raise ValueError("Unexpected comparison model: " + row["model"])
                    label = f"{row['model']}_seed{int(row['seed'])}"
                    config = batch / label / "run_config.json"
                    configs[label] = portable(json.loads(config.read_text(encoding="utf-8")), batch)
                    config_hashes[label + "/run_config.json"] = sha(config)
                prepared["run_configs.json"] = configs
        else:
            value = json.loads(source.read_text(encoding="utf-8"))
            if value.get("complete") is False:
                raise ValueError("Incomplete manifest: " + source.name)
            prepared[source.name] = portable(value, batch)
    # Refuse overwrites; all source data was checked before creating the export.
    destination.mkdir(parents=True, exist_ok=False)
    for name, value in prepared.items():
        target = destination / name
        if name.endswith(".csv"):
            fields, rows = value
            with target.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=fields, lineterminator="\n")
                writer.writeheader()
                writer.writerows(rows)
        else:
            target.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    manifest = dict(author="Zhuo Ma", source_batch_name=batch.name,
                    source_sha256={p.name: sha(p) for p in files}, row_counts=row_counts,
                    run_config_source_sha256=config_hashes,
                    exported_sha256={name: sha(destination / name) for name in prepared},
                    transformations=["absolute path values/keys replaced by portable labels; numeric cells unchanged",
                                     "exported CSV line endings normalized to LF"],
                    raw_runs_included=False)
    (destination / "export_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    links = "\n".join(f"- [{name}]({name})" for name in (*prepared, "export_manifest.json"))
    (destination / "README.md").write_text(
        f"# 精选汇总 / Selected summaries: {destination.name}\n\n"
        "作者 / Author: Zhuo Ma\n\n"
        f"原批次 / Original batch: `{batch.name}`\n\n"
        "保留原数值、参数与指纹，未包含逐帧日志/视频。`source_batch/` 是原批次内的相对引用，"
        "`esmini-perception/` 是实验根目录引用；它们不表示原运行已经随报告发布。"
        "`export_manifest.json` 记录路径转换、原文件与导出文件 SHA256。\n\n"
        "Original numbers, settings and fingerprints are retained, without frame logs/videos. "
        "`source_batch/` labels paths within the original batch; `esmini-perception/` labels the experiment root. "
        "These references do not mean raw runs are included. `export_manifest.json` records path normalization "
        "and source/export SHA256 values.\n\n"
        "## 文件 / Files\n\n" + links + "\n", encoding="utf-8")
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-dir", type=Path, required=True)
    parser.add_argument("--name", required=True, help="New report directory name, letters/numbers/_/-")
    parser.add_argument("--reports-dir", type=Path, default=ROOT / "reports")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", args.name):
        parser.error("name must contain only letters, numbers, underscores or hyphens")
    print("Exported:", export_batch(args.batch_dir, args.reports_dir.resolve() / args.name))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, csv.Error) as error:
        print("Export failed:", error, file=sys.stderr)
        sys.exit(1)
