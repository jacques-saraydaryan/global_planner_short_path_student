from setuptools import find_packages, setup

package_name = 'global_planner_short_path_student'

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
    maintainer='jsaraydaryan',
    maintainer_email='j@j.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ShortPathMng = global_planner_short_path_student.ShortPathMng:main'
        ],
    },
)
