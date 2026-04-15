from setuptools import setup

package_name = 'cstu_group_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/diffbot.urdf.xacro']),
        ('share/' + package_name + '/worlds', ['worlds/city.world']),
        ('share/' + package_name + '/config', ['config/ros2_controllers.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='user@example.com',
    description='City drive sim integrated into cstu-group-project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = cstu_group_project.image_processor:main',
            'image_viewer = cstu_group_project.image_viewer:main',
            'motion_controller = cstu_group_project.motion_controller:main',
        ],
    },
)
