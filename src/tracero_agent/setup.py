from setuptools import find_packages, setup

package_name = 'tracero_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'requirements-static-index.txt',
            'STATIC_INDEX.md',
        ]),
    ],
    install_requires=[
        'setuptools',
        'tree-sitter==0.23.2',
        'tree-sitter-cpp==0.23.4',
        'tree-sitter-python==0.23.6',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'agent = tracero_agent.agent:main',
            'spawn_tc01 = tracero_agent.spawn_tc01:main',
            'benchmark_tc01 = tracero_agent.run_tc01_benchmark:main',
            'build_static_index = tracero_agent.static_indexer:main',
        ],
    },
    
)
