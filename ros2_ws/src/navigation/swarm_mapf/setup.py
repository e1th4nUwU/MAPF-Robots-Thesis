from glob import glob

from setuptools import find_packages, setup

package_name = 'swarm_mapf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nahtie',
    maintainer_email='eiteenene18@gmail.com',
    description='Asignacion de metas y planificacion MAPF para colmenas de robots terrestres',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_allocator = swarm_mapf.goal_allocator:main',
            'allocation_client = swarm_mapf.allocation_client:main',
        ],
    },
)
