from setuptools import find_packages, setup

package_name = 'demo_python_pkg'

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
    maintainer_email='zzw@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'python_node = demo_python_pkg.demo_python_node:main',
            'person_node = demo_python_pkg.demo_person_node:main',
            'son_node = demo_python_pkg.demo_person_son_node:main',
            'node_son = demo_python_pkg.demo_node_son:main',
            'thread_node = demo_python_pkg.learn_thread:main',
        ],
    },
)
