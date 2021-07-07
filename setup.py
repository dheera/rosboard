from setuptools import setup, find_packages

package_name = 'rosboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(), #[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'tornado>=4.2.1',
    ],
    #zip_safe=True,
    maintainer='dheera',
    maintainer_email='dheera.r.e.m.o.v.e.t.h.i.s@dheera.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rosboard_node = rosboard.rosboard:main",
        ],
    },
)
