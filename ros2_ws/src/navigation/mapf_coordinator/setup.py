from setuptools import find_packages, setup

package_name = 'mapf_coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/goal_zone.launch.py',
            'launch/gnfc_allocator.launch.py',
            'launch/goal_zone_visualizer.launch.py',
            'launch/visualizer_only.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eithan',
    maintainer_email='eithantrevino@gmail.com',
    description='MAPF coordinator nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gnfc_goal_allocator   = mapf_coordinator.gnfc_goal_allocator:main',
            'goal_zone_visualizer  = mapf_coordinator.goal_zone_visualizer:main',
            'priority_manager      = mapf_coordinator.priority_manager:main',
            'whca_star             = mapf_coordinator.whca_star:main',
            'whca_coordinator      = mapf_coordinator.whca_coordinator:main',
            'cbs_coordinator       = mapf_coordinator.cbs_coordinator:main',
            'd_star_executor       = mapf_coordinator.d_star_executor:main',
        ],
    },
)
