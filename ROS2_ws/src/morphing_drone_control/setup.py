from setuptools import setup

package_name = 'morphing_drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'scripts.main_controller_node',
        'scripts.guidance_manager',
        'scripts.mode_controller',
        'scripts.motor_controller'
        'scripts.kalman_filter',
        'scripts.fault_detection',
    ],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/control_system.launch.py']),
        ('share/' + package_name + '/config', ['config/motor_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Morphing drone control node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'morphing_controller = scripts.main_controller_node:main',
        ],
    },
)

