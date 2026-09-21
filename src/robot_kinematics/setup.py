from setuptools import find_packages, setup

setup(
    name='robot_kinematics',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_kinematics']),
        ('share/robot_kinematics', ['package.xml', 'README.md']),
        ('share/robot_kinematics/config', ['config/joint_conventions.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    zip_safe=True,
    maintainer='nilesh',
    maintainer_email='nileshamarathunge@gmail.com',
    description='Shared GP7 kinematics, trajectories and joint conventions',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
)
