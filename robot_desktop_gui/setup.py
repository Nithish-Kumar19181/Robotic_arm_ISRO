from setuptools import find_packages, setup

package_name = 'robot_desktop_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['icons/*.svg'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=[
        'setuptools',
        'PyQt5',
        'pyqtgraph',
    ],

    zip_safe=True,
    maintainer='nithish',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # THIS is what makes ros2 run work
            'gui_node = robot_desktop_gui.gui_node:main',
            'gui_driver_node = robot_desktop_gui.gui_driver_node:main',
        ],
    },
)
