from setuptools import setup

package_name = 'amp_local_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adhomne',
    maintainer_email='adhomne@purdue.edu',
    description='Calculates local waypoint for the kart and sends it to goal_pose topic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['listener = amp_local_goal.follow_waypoint:main'],
    },
)
