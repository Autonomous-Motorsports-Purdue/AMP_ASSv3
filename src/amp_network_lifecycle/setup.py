from setuptools import setup

package_name = 'amp_network_lifecycle'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berliernj',
    maintainer_email='berlier3@gmail.com',
    description='Checks connection to the network. E-stops if disconnected for 5 seconds',
    license='GNU General Public License 3.0',
    entry_points={
        'console_scripts': [
            "lifecycle_client = amp_network_lifecycle.network_lifecycle_client:main",
            "lifecycle_server = amp_network_lifecycle.network_lifecycle_server:main"
        ],
    },
)
