from setuptools import setup

package_name = 'computer_vision'

setup(
    name='rs2',  # must match your package.xml
    version='0.0.1',
    packages=[package_name],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    description='ROS2 hybrid package with C++ and Python',
    license='MIT',
    entry_points={
        'console_scripts': [
            'chess_core = computer_vision.chess_core:main',
        ],
    },
)

