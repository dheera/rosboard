from setuptools import setup, find_packages

package_name = 'rosboard'

setup(
    name=package_name,
    version='1.2.1',
    packages=find_packages(), #[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'tornado>=4.2.1',
    ],
    extras_require = {
        'system_stats': ['psutil'],
    },
    package_data={
        'rosboard': [
            'html/*',
            'html/fonts/*', 
            'html/css/*',
            'html/css/images/*',
            'html/js/*',
            'html/js/viewers/*',
            'html/js/viewers/meta/*',
            'html/js/transports/*'
        ]
    },
    #zip_safe=True,
    maintainer='dheera',
    maintainer_email='dheera.r.e.m.o.v.e.t.h.i.s@dheera.net',
    description='ROS node that turns your robot into a web server to visualize ROS topics',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rosboard_node = rosboard.rosboard:main",
        ],
    },
)
