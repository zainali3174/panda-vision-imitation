from setuptools import find_packages, setup

package_name = 'panda_cartesian_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/panda_arm.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='1000',
    maintainer_email='zainalizahid471@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'cartesian_moveit_server = panda_cartesian_control.cartesian_moveit_server:main',
                'pick_place_server = panda_cartesian_control.pick_place_server:main',
                'camera_to_robot_node = panda_cartesian_control.camera_to_robot_node:main',
                'pick_from_camera_node = panda_cartesian_control.pick_from_camera_node:main',
                'cartesian_pick_place_server = panda_cartesian_control.cartesian_pick_place_server:main',

        ],
    },
)
