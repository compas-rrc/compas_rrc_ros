import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'compas_rrc_driver'

setup(
    name=package_name,
    version='1.1.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gonzalo Casas, Philippe Fleischmann, Michael Lyrenmann, Anton Tetov',
    maintainer_email='casas@arch.ethz.ch,fleischmann@arch.ethz.ch,lyrenmann@arch.ethz.ch,tetov@abm.lth.se',
    description='COMPAS RRC: ROS Driver',
    license='MIT',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'driver = compas_rrc_driver.driver:main',
        ],
    },
)
