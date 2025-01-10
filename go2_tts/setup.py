from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'go2_tts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tamas Foldi',
    maintainer_email='tfoldi@xsi.hu',
    description='Text to speech node using elevenlabs',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_node = go2_tts.tts_node:main',
        ],
    },
)
