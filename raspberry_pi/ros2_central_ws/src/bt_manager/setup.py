from setuptools import find_packages, setup

package_name = 'bt_manager'

setup(
    name=package_name,
    version='0.1.1',
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
    description='Package to handle bluetooth inputs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_manager = bt_manager.bt_manager:main',
        ],
    },
)
