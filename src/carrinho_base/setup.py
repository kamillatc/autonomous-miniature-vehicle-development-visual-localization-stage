from setuptools import find_packages, setup

package_name = 'carrinho_base'

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
    maintainer='epiibots',
    maintainer_email='epiibots@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = carrinho_base.motor_control:main',
            'checkpoint_recorder = carrinho_base.checkpoint_recorder:main',
            'checkpoint_player   = carrinho_base.checkpoint_player:main',
        ],
    },
)
