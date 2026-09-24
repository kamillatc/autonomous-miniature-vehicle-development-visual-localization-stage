from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carrinho_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),   # ADICIONADO diz: "pegue todos os .yaml da pasta config/ e instale em share/carrinho_localization/config/ quando o pacote for buildado".
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),   # ADICIONADO
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='epiibots',
    maintainer_email='kamilla.txcostaif@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_odom_relay = carrinho_localization.wheel_odom_relay:main',
        ],
    },
)
