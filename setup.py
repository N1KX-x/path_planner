from setuptools import find_packages, setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ykgroup',
    maintainer_email='ykgroup@todo.todo',
    description='Dijkstra path planner with LiDAR obstacle avoidance and emergency stop',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_planner = path_planner.main:main',
            'emergency_stop_keyboard = path_planner.emergency_stop_keyboard:main',
            'navigation_gui = path_planner.navigation_gui:main',
        ],
    },
)
