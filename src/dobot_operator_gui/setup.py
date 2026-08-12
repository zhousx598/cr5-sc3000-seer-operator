from setuptools import find_packages
from setuptools import setup


package_name = 'dobot_operator_gui'


setup(
    name=package_name,
    version='0.10.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zsx',
    maintainer_email='zsx@example.com',
    description='Unified CR5, SC3000, gripper, and SEER AGV operator panel.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dobot_operator_gui = dobot_operator_gui.main:main',
        ],
    },
)
