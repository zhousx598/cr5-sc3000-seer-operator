from setuptools import find_packages, setup


package_name = 'dobot_point_manager'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='zsx',
    maintainer_email='zsx@example.com',
    description=(
        'Move between named Dobot joint points with mode, start-position, '
        'speed, completion, and final-position checks.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'move_between_points = '
            'dobot_point_manager.move_between_points:main',
        ],
    },
)
