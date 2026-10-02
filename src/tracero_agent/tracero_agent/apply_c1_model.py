#!/usr/bin/env python3
import argparse
import hashlib
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

DEFAULT_TARGET = (
    '/root/turtlebot3_ws/src/turtlebot3_simulations/'
    'turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'
)


def parse_args():
    parser = argparse.ArgumentParser(description='Apply the C1 Gazebo model variant')
    parser.add_argument('--variant', choices=('normal', 'fault'), required=True)
    parser.add_argument('--target', default=DEFAULT_TARGET)
    parser.add_argument('--variant-dir', default=None)
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    options = parse_args()
    target = Path(options.target)
    if options.variant_dir:
        variant_dir = Path(options.variant_dir)
    else:
        variant_dir = Path(get_package_share_directory('tracero_agent')) / 'config' / 'c1'
    source = variant_dir / ('model_C1.sdf' if options.variant == 'fault' else 'model.sdf')
    if not source.is_file():
        raise SystemExit(f'model variant not found: {source}')
    if not target.is_file():
        raise SystemExit(f'active Gazebo model not found: {target}')
    shutil.copyfile(source, target)
    print(f'Applied C1 {options.variant} model to {target}')
    print(f'wheel_separation SHA256: {sha256(target)}')


if __name__ == '__main__':
    raise SystemExit(main())
