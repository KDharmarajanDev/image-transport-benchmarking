from setuptools import setup
from glob import glob
import os

package_name = 'image_transport_benchmarker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karthikdharmarajan',
    maintainer_email='kdharmarajan@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'png_test = image_transport_benchmarker.png_test:main',
            'theora_test = image_transport_benchmarker.theora_test:main',
            'h264_test = image_transport_benchmarker.h264_test:main',
            'raw_test = image_transport_benchmarker.raw_test:main',

            'png_test_cloud = image_transport_benchmarker.png_test_cloud:main',
            'theora_test_cloud = image_transport_benchmarker.theora_test_cloud:main',
            'h264_test_cloud = image_transport_benchmarker.h264_test_cloud:main',
            'raw_test_cloud = image_transport_benchmarker.raw_test_cloud:main',

            'image_pub = image_transport_benchmarker.image_folder_publisher:main',
            'video_pub = image_transport_benchmarker.video_publisher:main'
        ],
    },
)
