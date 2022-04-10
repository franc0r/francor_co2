from setuptools import setup

package_name = 'francor_co2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Bauernschmitt',
    maintainer_email='martin.bauernschmitt@francor.de',
    description='CO2 sensor interface',
    license='BSD 3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'co2_sensor_interface = francor_co2.co2_sensor_interface:main'
        ],
    },
)
