from setuptools import find_packages, setup
from glob import glob

package_name = 'franka_lock_unlock'

setup(
    name=package_name,
    version='4.2.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jk-ethz',
    maintainer_email='ethz@juliankeller.net',
    description='Lock or unlock the Franka Emika Panda joint brakes programmatically.',
    license='AGPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'franka_lock_unlock = franka_lock_unlock.franka_lock_unlock:main',
            'franka_shutdown = franka_lock_unlock.franka_shutdown:main',
        ],
    },
)
