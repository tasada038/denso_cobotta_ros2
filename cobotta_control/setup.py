from setuptools import setup

package_name = 'cobotta_control'
submodules = 'cobotta_control/pybcapclient'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fish',
    maintainer_email='fish@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'to_rviz_node = cobotta_control.to_rviz_node:main',
            'to_cobotta_node = cobotta_control.to_cobotta_node:main',
        ],
    },
)
