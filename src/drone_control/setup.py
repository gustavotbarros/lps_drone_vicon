from setuptools import setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustavobarros',
    maintainer_email='gustavo.tenoriobritodebarros@rub.de',
    description='Offboard mode package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_mode = drone_control.offboard_mode:main',
            'vehicle_visual_pub = drone_control.vehicle_visual_pub:main',
            'setpoint_publisher = drone_control.setpoint_publisher:main',
            'square_planner = drone_control.square_planner:main',
            'circle_planner = drone_control.circle_planner:main'
        ],
    },
)
