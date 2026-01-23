import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'patrol_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    # launch파일 설치
    # setup.py의 data_files에서 말하는 share/... 는 install 하위 경로임 
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),

            ('share/' + package_name, ['package.xml']),

            # Launch 폴더 설치
            (os.path.join('share', package_name, 'launch'),
                glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

            # Maps 폴더 설치 (yaml, pgm 등)
            ('share/' + package_name + '/maps', glob('maps/*')),

            # params 폴더 설치 (yaml, urdf 등)
            ('share/' + package_name + '/params', glob('params/*')),


        ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='kkkgim@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [

        ],
    },
)


