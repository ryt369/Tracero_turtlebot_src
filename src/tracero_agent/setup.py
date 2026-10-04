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
        ('share/' + package_name + '/config/common', [
            'config/common/burger.yaml',
        ]),
        ('share/' + package_name + '/config/a1', [
            'config/a1/burger_A1.yaml',
        ]),
        ('share/' + package_name + '/config/a2', [
            'config/a2/burger_A2.yaml',
        ]),
        ('share/' + package_name + '/config/c1', [
            'config/c1/model.sdf',
            'config/c1/model_C1.sdf',
        ]),
        ('share/' + package_name + '/config/d2', [
            'config/d2/turtlebot3_burger.urdf',
            'config/d2/turtlebot3_burger_D2.urdf',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/tc01_brake_nav2.launch.py',
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
            'benchmark_a1 = tracero_agent.benchmark_a1:main',
            'benchmark_a2 = tracero_agent.benchmark_a2:main',
            'benchmark_c1 = tracero_agent.benchmark_c1:main',
            'apply_c1_model = tracero_agent.apply_c1_model:main',
            'benchmark_d2 = tracero_agent.benchmark_d2:main',
            'apply_d2_urdf = tracero_agent.apply_d2_urdf:main',
            'build_static_index = tracero_agent.static_indexer:main',
            'safety_controller = tracero_agent.safety_controller:main',
        ],
    },
    
)
