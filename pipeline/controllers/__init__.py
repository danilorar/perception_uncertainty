"""Controllers compiled into esmini. Each one is a folder here with a controller.json."""
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from ..common import read_json

FOLDER = Path(__file__).resolve().parent


@dataclass(frozen=True)
class Controller:
    key: str
    folder: Path
    esmini_name: str   # registered name, used as <Controller name="..."> in the scene
    sources: tuple     # files copied into esmini; never opened by this package
    header: str
    factory: str
    type_enum: str     # added to esmini's Controller::Type; empty if the controller needs none
    properties: dict   # scene <Properties>; None keeps the ones already in the scenario file
    signals: dict      # LOG_VALUE names: 'active' (1 while intervening) and 'acceleration'

    def source_metadata(self):
        """Size and modification time only."""
        return {name: {'size': (self.folder / name).stat().st_size,
                       'mtime_ns': (self.folder / name).stat().st_mtime_ns}
                for name in self.sources}


@lru_cache(maxsize=None)
def all_controllers():
    found = {}
    for path in sorted(FOLDER.glob('*/controller.json')):
        data = read_json(path)
        found[path.parent.name] = Controller(
            key=path.parent.name, folder=path.parent, esmini_name=data['esmini_name'],
            sources=tuple(data['sources']), header=data['header'], factory=data['factory'],
            type_enum=data.get('type_enum', ''), properties=data.get('properties'),
            signals=data.get('signals', {}))
    return found


def get(key):
    controllers = all_controllers()
    if key not in controllers:
        raise ValueError(f'Unknown controller {key!r}. Available: {", ".join(controllers)}')
    return controllers[key]
