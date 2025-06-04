from setuptools import find_packages, setup

package_name = 'proto3_goal_manager_py'

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
    maintainer='prabhakar',
    maintainer_email='pmn2119@columbia.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          "goal_executor = proto3_goal_manager_py.goal_executor:main"
        ],
    },
)
