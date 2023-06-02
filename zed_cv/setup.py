from setuptools import setup

package_name = 'zed_cv'

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
    maintainer='jasper',
    maintainer_email='49565505+wluxie@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'zed_pub = {package_name}.zed_pub:main',
            f'cam_sub = {package_name}.cam_sub:main',
            f'compress_img = {package_name}.compress_img:main',
            f'decompress_img = {package_name}.decompress_img:main',
        ],
    },
)
