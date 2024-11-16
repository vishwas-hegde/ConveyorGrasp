from setuptools import find_packages, setup

package_name = 'first_cam_feed_move'

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
    maintainer='vishwas',
    maintainer_email='vishwas@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'moverobot = first_cam_feed_move.moverobot:main',
                             'getObjectWorldPos = first_cam_feed_move.getObjectWorldPos:main',
        ],
    },
)
