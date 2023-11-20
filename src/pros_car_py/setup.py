from setuptools import find_packages, setup
from glob import glob

package_name = 'pros_car_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kylin',
    maintainer_email='kylingithubdev@gmail.com',
    description='This is a pkg to control PCar.',
    license='Commercial License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carA_random = pros_car_py.carA_random_AI:main',
            'carA_keyboard = pros_car_py.carA_keyboard:main',
            'carA_reader = pros_car_py.carA_serial_reader:main',
            'carA_writer = pros_car_py.carA_serial_writer:main',
            'carB_random = pros_car_py.carB_random_AI:main',
            'carB_keyboard = pros_car_py.carB_keyboard:main',
            'carB_reader = pros_car_py.carB_serial_reader:main',
            'carB_writer = pros_car_py.carB_serial_writer:main',
            'carD_random = pros_car_py.carD_random_AI:main',
            'carD_keyboard = pros_car_py.carD_keyboard:main',
            'carD_reader = pros_car_py.carD_serial_reader:main',
            'carD_writer = pros_car_py.carD_serial_writer:main',
        ],
    },
)
