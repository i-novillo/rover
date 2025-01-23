from setuptools import find_packages, setup

package_name = 'kb_input_manager'

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
    maintainer='ignacio',
    maintainer_email='ignovillo1@gmail.com',
    description='Test package to register keyboard inputs to move the rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kb_input_manager = kb_input_manager.kb_input_manager:main',
        ],
    },
)
