from setuptools import setup

package_name = 'amp_network_lifecycle'

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
    maintainer='berliernj',
    maintainer_email='berlier3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lifecycle_client = amp_network_lifecycle.network_lifecycle_client:main",
            "lifecycle_server = amp_network_lifecycle.network_lifecycle_server:main"
        ],
    },
)
