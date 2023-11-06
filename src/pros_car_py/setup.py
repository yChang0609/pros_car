from setuptools import find_packages, setup

package_name = 'pros_car_py'

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
    maintainer='Kylin',
    maintainer_email='kylingithubdev@gmail.com',
    description='This is a pkg to control PCar.',
    license='Commercial License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carA_random = pros_car_py.carA_random_controller:main'
            'carB_random = pros_car_py.carB_random_controller:main'
            'carC_random = pros_car_py.carC_random_controller:main'
            'carD_random = pros_car_py.carD_random_controller:main'
        ],
    },
)
