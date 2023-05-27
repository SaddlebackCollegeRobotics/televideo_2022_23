from setuptools import setup

package_name = 'telecom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/base_telecom.launch.py']),
        ('share/' + package_name, ['launch/rover_telecom.launch.py']),
        ('share/' + package_name, ['config/frame_data.json']),
        ('share/' + package_name, ['scripts/find_devpath.bash']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='camwolff',
    maintainer_email='36940948+camwolff02@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_stitcher = telecom.stitch_img:main',
            'zed_pub = telecom.zed_pub:main',
            'fpv_pub = telecom.fpv_pub:main',
            'cam_sub = telecom.cam_sub:main',
        ],
    },
)
