from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vris_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # [추가] launch 폴더 안의 모든 .launch.py 파일을 share 영역으로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wosasa',
    maintainer_email='swpants05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gamepad_teleop = vris_teleop.gamepad_teleop_node:main',
            'gamepad_joy = vris_teleop.gamepad_joy_node:main',
        ],
    },
)
