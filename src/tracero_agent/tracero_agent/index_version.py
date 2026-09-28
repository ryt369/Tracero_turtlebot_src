import hashlib
import json
from pathlib import Path
from typing import Dict


INDEX_VERSION_FILENAME = 'static_index_version.txt'


def generate_index_version(payload: Dict, prefix: str = 'tc01') -> str:
    versionless_payload = dict(payload)
    versionless_payload.pop('version', None)
    canonical = json.dumps(
        versionless_payload,
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')
    fingerprint = hashlib.sha256(canonical).hexdigest()[:12]
    return f'{prefix}-{fingerprint}'


def version_file(events_dir: str) -> Path:
    return Path(events_dir) / INDEX_VERSION_FILENAME


def read_index_version(events_dir: str, requested: str = '') -> str:
    requested = requested.strip()
    if requested:
        return requested
    path = version_file(events_dir)
    try:
        version = path.read_text(encoding='utf-8').strip()
    except OSError as error:
        raise RuntimeError(
            f'static index version file is unavailable: {path}'
        ) from error
    if not version:
        raise RuntimeError(f'static index version file is empty: {path}')
    return version


def write_index_version(events_dir: str, version: str):
    path = version_file(events_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f'{version}\n', encoding='utf-8')
