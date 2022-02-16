from setuptools import setup

package_name = 'py_pubsub'

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
    maintainer='shkim',
    maintainer_email='shkim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'media_pub = py_pubsub.media_pub:main',
                'media_listen = py_pubsub.media_listen:main',
                'camera_sub1 = py_pubsub.camera_listen:main',
                'temp_pub = py_pubsub.temp_pub:main',
                'temp_sub = py_pubsub.temp_sub:main',
                'camera_pub = py_pubsub.camera_pub:main',
                'camera_sub = py_pubsub.camera_sub:main',
                'arduino = py_pubsub.listen1:main',
        ],
    },
)
