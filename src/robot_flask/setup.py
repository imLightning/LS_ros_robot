from setuptools import find_packages, setup

package_name = 'robot_flask'

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
    maintainer='ros_system',
    maintainer_email='2744002576@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveControl = robot_flask.move_control:main',
            'app = robot_flask.app:main',
            'camera = robot_flask.camera:main',
        ],
    },
)
