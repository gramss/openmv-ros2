from setuptools import setup

package_name = 'openmv_driver_action_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, "openmv_driver_res", "openmv_driver_res.rpc"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'pillow'],
    zip_safe=True,
    author='Florian Gramß',
    author_email='flo.gramss@gmail.com',
    maintainer='Florian Gramß',
    maintainer_email='flo.gramss@gmail.com',
    keywords=['ROS', 'OpenMV', 'Open Hardware', 'Image Processing', 'ML'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 action server driver for the awesome OpenMV cams.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = ' + package_name + '.server:main',
            'test_client = ' + package_name + '.test_client:main'
        ],
    },
)
