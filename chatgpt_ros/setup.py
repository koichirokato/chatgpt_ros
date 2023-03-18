from setuptools import setup

package_name = 'chatgpt_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koichi',
    maintainer_email='k.koichiro0222@gmail.com',
    description='ChattGPT API ROS2 wrapper',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chatgpt_ros = ' + package_name + '.chatgpt_ros:main',
            'chatgpt_ros_service = ' + package_name + '.chatgpt_ros_service_server:main',
            'client_sample = ' + package_name + '.chatgpt_ros_service_client:main',
        ],
    },
)
