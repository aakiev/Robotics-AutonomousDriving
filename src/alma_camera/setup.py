from setuptools import find_packages, setup

package_name = 'alma_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'camera_pub = alma_camera.camera_publisher:main',
           'camera_image_sub = alma_camera.image_subscriber:main', 
           'camera_colorHSV_sub = alma_camera.colorHSV_subscriber:main',
           'camera_line_detector = alma_camera.camera_line_detector:main',
        ],
    },
)
