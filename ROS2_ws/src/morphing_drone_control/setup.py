from setuptools import setup

package_name = 'morphing_drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={package_name: package_name},
    data_files=[
        ('share/' + package_name + '/launch', ['launch/control_system.launch.py']),
        ('share/' + package_name + '/config', ['config/motor_params.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'geographiclib'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Morphing drone control node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'main_controller = morphing_drone_control.main_controller_node:main',
        ],
    },
)


