from glob import glob

from setuptools import setup

package_name = 'scara_sort'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Afham Ahmed',
    maintainer_email='afham12ahmed@gmail.com',
    description='Kinematics, vision, tracking and the nodes that sort items off the conveyor belt.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to = scara_sort.move_to:main',
            'item_spawner = scara_sort.item_spawner:main',
        ],
    },
)
