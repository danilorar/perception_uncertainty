#!/usr/bin/env python3
"""Generate external-AEB scenarios without editing the original data.

Author: Zhuo Ma
Input: Data/C_original_<id>.xosc and its .xodr road.
Output: simulation/generated/C_aeb_<id>.xosc and generation_manifest.json.
Only the generated copy replaces HiDriveController with ExternalController.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from paths import DATA, GENERATED, SCENARIO_IDS


def sha256(path):
    """Fingerprint a file so experiments can identify their exact input data."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def generate(identifier, output_dir=GENERATED, overwrite=False):
    """Return a generated XOSC path and its provenance; preserve source bytes.

    The target still follows its original timed trajectory. Python owns the ego
    motion, so its original ManeuverGroup must be removed to avoid two competing
    position writers. Geometry, initial poses and the overall stop trigger stay.
    """
    source = DATA / f"C_original_{identifier}.xosc"
    road = source.with_suffix(".xodr")
    if not source.is_file() or not road.is_file():
        raise FileNotFoundError(f"Need both original scenario and road: {source}")
    output_dir = Path(output_dir).resolve()
    destination = output_dir / f"C_aeb_{identifier}.xosc"
    if destination.exists() and not overwrite:
        raise FileExistsError(f"Already exists: {destination}. Use --overwrite to regenerate.")

    tree = ET.parse(source)
    root = tree.getroot()
    ego = root.find('./Entities/ScenarioObject[@name="object_1"]')
    private = root.find('./Storyboard/Init/Actions/Private[@entityRef="object_1"]')
    road_ref = root.find("./RoadNetwork/LogicFile")
    if ego is None or private is None or road_ref is None:
        raise ValueError(f"Unsupported scenario structure: {source}")

    # A repository-relative road reference remains usable after cloning elsewhere.
    road_ref.set("filepath", Path(os.path.relpath(road, output_dir)).as_posix())
    for controller in list(ego.findall("ObjectController")):
        ego.remove(controller)
    controller = ET.SubElement(ET.SubElement(ego, "ObjectController"), "Controller", name="PythonAEB")
    properties = ET.SubElement(controller, "Properties")
    for name, value in (("esminiController", "ExternalController"),
                        ("mode", "override"), ("useGhost", "false")):
        ET.SubElement(properties, "Property", name=name, value=value)

    # OpenSCENARIO expects one action choice per PrivateAction. Move activation
    # out of the original speed action and into its own PrivateAction element.
    for action in list(private.findall("PrivateAction")):
        for child in list(action.findall("ControllerAction")):
            action.remove(child)
        if len(action) == 0:
            private.remove(action)
    activation = ET.SubElement(ET.SubElement(private, "PrivateAction"), "ControllerAction")
    ET.SubElement(activation, "ActivateControllerAction", longitudinal="true", lateral="true")

    removed = 0
    for act in root.findall("./Storyboard/Story/Act"):
        for group in list(act.findall("ManeuverGroup")):
            actors = [item.get("entityRef") for item in group.findall("./Actors/EntityRef")]
            if "object_1" in actors:
                if actors != ["object_1"]:
                    raise ValueError("Cannot remove a group shared by ego and other actors")
                act.remove(group)
                removed += 1
    if removed != 1:
        raise ValueError("Expected exactly one original ego ManeuverGroup")

    # Keep the original FileHeader author: the dataset is not authored by this
    # script. Record the transformation's author separately in the manifest.
    output_dir.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return destination, dict(
        scenario=identifier, generated_file=destination.name,
        source_file=source.name, road_file=road.name,
        source_sha256=sha256(source), road_sha256=sha256(road),
        generated_sha256=sha256(destination), generated_by="Zhuo Ma",
        changes=["relative road path", "external ego controller", "separate activation action",
                 "ego trajectory delegated to Python"],
    )


def main():
    """Generate one scenario or all three, then update their provenance manifest."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=(*SCENARIO_IDS, "all"), default="all")
    parser.add_argument("--output-dir", type=Path, default=GENERATED)
    parser.add_argument("--overwrite", action="store_true", help="Replace existing generated copies only")
    args = parser.parse_args()
    identifiers = SCENARIO_IDS if args.scenario == "all" else (args.scenario,)
    manifest_path = args.output_dir / "generation_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    # Check all destinations before writing anything, avoiding a partial batch.
    if not args.overwrite:
        existing = [args.output_dir / f"C_aeb_{i}.xosc" for i in identifiers
                    if (args.output_dir / f"C_aeb_{i}.xosc").exists()]
        if existing:
            parser.error(f"Already exists: {existing[0]}; use --overwrite")
    for identifier in identifiers:
        path, manifest[identifier] = generate(identifier, args.output_dir, args.overwrite)
        print(f"Generated: {path}")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
