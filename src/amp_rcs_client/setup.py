from setuptools import setup

package_name = 'amp_rcs'

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
    description='Service client for sent track state change requests to the service server',
    license='GNU General Public License 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        [ "amp_rcs_client = amp_rcs_client.service_client:main",
          "amp_rcs_server = amp_rcs_server.dummy_server:main"
        ],
    },
)
