from setuptools import setup

PKG_NAME='fgs_opt_data_storage'

setup(
    name=PKG_NAME,
    version='2.0.0',
    packages=[PKG_NAME],
    install_requires=['setuptools'],
    zip_safe=True,
    author='fugashy',
    author_email='fugashy@icloud.com',
    maintainer='fugashy',
    maintainer_email='fugashy@icloud.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: MIT :: MIT license',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=('A data generator'),
    license='MIT',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'generate_data_2d = fgs_opt_data_storage.generator:ros2_entry_point',
            'convert_ba_in_large_to_cv = fgs_opt_data_storage.convert_ba_in_large_to_cv:main',
        ],
    },
)
