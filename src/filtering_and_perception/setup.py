from setuptools import find_packages, setup

package_name = 'filtering_and_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='user@example.com',
    description='Filtering and perception package for the RC car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'filtering_and_perception_node = '
            'filtering_and_perception.filtering_and_perception_node:main',
        ],
    },
)
