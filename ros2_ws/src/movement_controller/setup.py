from setuptools import setup

package_name = 'movement_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'movement_controller = movement_controller.movement_controller:main',
        ],
    },
)
