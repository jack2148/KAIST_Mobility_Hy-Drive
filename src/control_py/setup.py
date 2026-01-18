from setuptools import setup

package_name = 'control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ym',
    maintainer_email='ym@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_cav01 = control_py.control_cav1:main',
            'control_1_1 = control_py.control_1_1:main',
            'control_1_12 = control_py.control_1_12:main',
        ],
    },
)
