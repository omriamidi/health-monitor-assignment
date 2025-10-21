from setuptools import find_packages, setup

package_name = 'health_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/health_monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omri Amidi',
    maintainer_email='omriamidi5@gmail.com',
    description='Health monitor nodes for robot diagnostics',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'health_analyzer_node = health_monitor.health_analyzer_node:main',
            'alert_manager = health_monitor.alert_manager_node:main',
            'health_history_service = health_monitor.health_history_service:main',
        ],
    },
)
