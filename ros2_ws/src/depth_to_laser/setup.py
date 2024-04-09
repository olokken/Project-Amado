from setuptools import find_packages, setup

package_name = 'depth_to_laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', "transformers", "torch"],
    zip_safe=True,
    maintainer='ole',
    maintainer_email='olelokken1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_laser_node = depth_to_laser.depth_to_laser_node:main'
        ],
    },
)
