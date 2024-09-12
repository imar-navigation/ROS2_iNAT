from setuptools import setup
from setuptools import find_packages

package_name = 'ixcom_driver'

setup(
    name=package_name,
    version='0.1.0',
    #packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zp',
    maintainer_email='p.ziegler@imar-navigation.de',
    description='ixcom_driver is a iNAT data publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = ixcom_driver.ixcom_driver_publisher:main',
            'sub = ixcom_driver.ixcom_driver_subscriber:main',
            'cli = ixcom_driver.ixcom_driver_client:main',
        ],
    },
)
