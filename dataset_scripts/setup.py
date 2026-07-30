from setuptools import find_packages, setup

package_name = 'dataset_scripts'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'spawn_objects_node = dataset_scripts.spawn_objects:main',
            'delete_objects_node = dataset_scripts.delete_objects:main',
            'save_image_node = dataset_scripts.save_image:main',
        ],
    },
)
