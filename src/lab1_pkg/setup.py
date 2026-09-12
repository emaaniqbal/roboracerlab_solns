from setuptools import find_packages, setup

# for launch files to run....
import os
from glob import glob

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #config to copy
        #https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html

        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
         #^only for .py files
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emaan',
    maintainer_email='emaan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker=lab1_pkg.talker:main',
            'relay=lab1_pkg.relay:main'
        ],
    },
)
