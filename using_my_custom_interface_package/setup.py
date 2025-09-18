from setuptools import find_packages, setup

package_name = 'using_my_custom_interface_package'

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
    maintainer='jinhan',
    maintainer_email='jinhan3579@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'custom_msg_publisher = using_my_custom_interface_package.my_msg_publisher:main',
        'custom_msg_subscriber = using_my_custom_interface_package.my_msg_subscriber:main',
        'jinhan_publisher = using_my_custom_interface_package.sum_publisher:main',
        'jinhan_subscriber = using_my_custom_interface_package.sum_subscriber:main',
        
        ],
    },
)
