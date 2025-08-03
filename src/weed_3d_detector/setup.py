from setuptools import setup

package_name = 'weed_3d_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tamir',
    maintainer_email='tamir@example.com',
    description='Weed 3D Detector Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weed_3d_detector_node = weed_3d_detector.weed_3d_detector_node:main'
        ],
    },
)
