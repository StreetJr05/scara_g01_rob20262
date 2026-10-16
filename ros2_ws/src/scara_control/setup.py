from glob import glob

from setuptools import setup

package_name = 'scara_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel Calle',
    maintainer_email='miguelcalleromero05@gmail.com',
    description='High-level software of the RRP SCARA',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge = scara_control.bridge:main',
            'teach_pendant = scara_control.teach_pendant:main',
            'goto = scara_control.goto:main',
            'characterize = scara_control.characterize:main',
            'analyze = scara_control.analyze:main',
        ],
    },
)
