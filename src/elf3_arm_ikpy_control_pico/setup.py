from setuptools import find_packages, setup


package_name = 'elf3_arm_ikpy_control_pico'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkkk',
    maintainer_email='kkkk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'elf3_arm_ikpy_control_pico = '
                'elf3_arm_ikpy_control_pico.elf3_arm_ikpy_control_pico:main'
            ),
        ],
    },
)
