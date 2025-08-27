from setuptools import find_packages, setup

package_name = 'human_pose_detector'

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
    maintainer='pedroalg',
    maintainer_email='pedroalg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = human_pose_detector.pose_publisher:main',
            'pose_subscriber = human_pose_detector.pose_subscriber:main',
            'pose_tf_publisher = human_pose_detector.pose_tf_publisher:main',
        ],
    },
)
