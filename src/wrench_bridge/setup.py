from setuptools import find_packages, setup

# CHANGE THIS LINE
package_name = 'wrench_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A package to demonstrate Python node setup.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # CHANGE THIS ENTRY POINT to match your package and file name
    entry_points={
        'console_scripts': [
            'wrench_bridge_node = wrench_bridge.wrench_bridge_node:main',
        ],
    },
)