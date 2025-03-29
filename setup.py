# Copyright 2025 BzY*FuZy <bzy.fuzy@gmail.com>
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

from setuptools import setup

package_name = 'generic_subscriber_example'

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
    maintainer='BzY*FuZy',
    maintainer_email='bzy.fuzy@gmail.com',
    description='ROS 2 example demonstrating a generic subscriber with multiple publishers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generic_subscriber = generic_subscriber_example.generic_subscriber:main',
            'string_publisher = generic_subscriber_example.string_publisher:main',
            'int_publisher = generic_subscriber_example.int_publisher:main',
        ],
    },
)
