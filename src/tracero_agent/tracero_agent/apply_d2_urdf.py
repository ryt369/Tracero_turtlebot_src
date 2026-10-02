#!/usr/bin/env python3
import argparse
import hashlib
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


DEFAULT_TARGET = (
    '/root/turtlebot3_ws/src/turtlebot3_simulations/'
    'turtlebot3_gazebo/urdf/turtlebot3_burger.urdf'
)


def parse_args():
    parser = argparse.ArgumentParser(description='Apply the D2 lidar TF URDF variant')
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
        variant_dir = Path(get_package_share_directory('tracero_agent')) / 'config' / 'd2'
    source = variant_dir / (
        'turtlebot3_burger_D2.urdf'
        if options.variant == 'fault'
        else 'turtlebot3_burger.urdf'
    )
    if not source.is_file():
        raise SystemExit(f'URDF variant not found: {source}')
    if not target.is_file():
        raise SystemExit(f'active TurtleBot3 URDF not found: {target}')
    shutil.copyfile(source, target)
    print(f'Applied D2 {options.variant} URDF to {target}')
    print(f'URDF SHA256: {sha256(target)}')


if __name__ == '__main__':
    raise SystemExit(main())
