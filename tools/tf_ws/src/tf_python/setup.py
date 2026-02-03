from setuptools import find_packages, setup

package_name = 'tf_python'

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
    maintainer='zzw',
    maintainer_email='3092875415@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster = tf_python.static_tf_broadcaster:main',
            'dynamic_tf_broadcaster = tf_python.dynamic_tf_broadcaster:main',
            'tf_listener = tf_python.tf_listener:main',
        ],
    },
)
