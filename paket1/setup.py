from setuptools import find_packages, setup

package_name = 'paket1'

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
    maintainer='arash',
    maintainer_email='arash@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'denger = paket1.denger:main',
            'ngoce = paket1.ngoce:main',
            'mecanum = paket1.mecanum:main',
        ],
    },
)
