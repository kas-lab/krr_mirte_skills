from setuptools import find_packages, setup

package_name = 'krr_mirte_skills'

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
    maintainer='ega',
    maintainer_email='45923444+EGAlberts@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_objects = krr_mirte_skills.get_objects_node:main'
            'get_object_info = krr_mirte_skills.get_object_info:main'
        ],
    },
)
