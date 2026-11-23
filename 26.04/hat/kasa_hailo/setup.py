from setuptools import setup
from glob import glob
pkg = 'kasa_hailo'
setup(
    name=pkg, version='0.1.0', packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/launch', glob('launch/*.launch.py')),
        ('share/' + pkg + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'hailo_yolo_inference_node = kasa_hailo.hailo_yolo_inference_node:main',
    ]},
)
