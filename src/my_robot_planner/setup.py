from setuptools import find_packages, setup

package_name = 'my_robot_planner'

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
    maintainer='varun',
    maintainer_email='varun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "a_star.py = my_robot_planner.a_star:main",
            "global_planner_node.py = my_robot_planner.global_planner_node:main",
            "dwa.py = my_robot_planner.dwa:main",
            "local_planner_node.py = my_robot_planner.local_planner_node:main"
        ],
    },
)
