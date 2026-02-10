from setuptools import find_packages, setup

package_name = 'webcam_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='Raphaell',
    maintainer_email='raphaell@todo.todo',
    description='Webcam and OpenCV Grayscale Demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = webcam_demo.camera_node:main',
            'processor = webcam_demo.processor_node:main',
        ],
    },
)

