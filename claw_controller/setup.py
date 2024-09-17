from setuptools import find_packages, setup

package_name = 'claw_controller'

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
    maintainer='jaxw501',
    maintainer_email='jaxw501@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remote_control = claw_controller.remote_control:main',
            'leg_state_publisher = claw_controller.leg_state_publisher:main',
            'legIK_state_publisher = claw_controller.legIK_state_publisher:main',
            'leg_walk_state_publisher = claw_controller.leg_walk_state_publisher:main',
            'quadruped_walk_state_publisher = claw_controller.quadruped_walk_state_publisher:main'
        ],
    },
)
