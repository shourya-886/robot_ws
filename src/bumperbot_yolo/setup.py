import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bumperbot_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Static files
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/firebase', glob('firebase/*')),
        ('share/' + package_name + '/sample_images', glob('sample_images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shourya',
    maintainer_email='pihushourya100@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'log_test = bumperbot_yolo.logging_test:main', #testing done
            'serial_test = bumperbot_yolo.serial_test:main', #testing done
            'image_publisher = bumperbot_yolo.image_publisher:main', #testing done
            'cloudinary_firebase_test = bumperbot_yolo.cloudinary_firebase_test:main', #testing done
            'cloudinary_test = bumperbot_yolo.cloudinary_test:main', #testing done
            'main_with_ros = bumperbot_yolo.main_with_ros:main',
            'testing = bumperbot_yolo.testing:main',
            'testing2 = bumperbot_yolo.testing2:main',
        ],
    },
)
