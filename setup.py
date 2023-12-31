from setuptools import find_packages, setup

package_name = 'ros2_course'

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
    maintainer='ros_user',
    maintainer_email='ros_user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'turtle = ros2_course.turtle:main',
        'listener = ros2_course.turtle_listener:main',
        #'turtlesim_controller = ros2_course.turtlesim_controller:main',
        #'talker = ros2_course.talker:main',
        #'listener = ros2_course.listener:main',

            'hello = ros2_course.hello:main'
        ],
    },
)
