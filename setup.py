from setuptools import find_packages, setup
import os
import glob

package_name = 'group03'

# ✅ Build the full data_files list including launch, params, worlds, sounds
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ('share/' + package_name + '/params', glob.glob('params/*.yaml')),
    ('share/' + package_name + '/sounds', glob.glob('sounds/*.wav')),
]

# ✅ Recursively include everything under `worlds/`
for dirpath, _, files in os.walk('worlds'):
    if not files:
        continue
    dest = os.path.join('share', package_name, dirpath)
    srcs = [os.path.join(dirpath, f) for f in files]
    data_files.append((dest, srcs))

# ✅ Use the data_files variable you defined above
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,  # <-- use this
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jverhoog',
    maintainer_email='j.t.verhoog@tudelft.nl',
    description='The group03 package contains ROS 2 nodes, drivers and launch files to autonomously detect, plan and pick apples in an orchard environment.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = group03.FiniteStateMachine:main',
            'farmer_command_node = group03.FarmerInterface:main',
            'sonar_obstacles = group03.EmergencyBrakeCheck:main',
            'navigation_node = group03.NavPlanner:main'
            'gripper_manager_node = group03.GripperManager:main',
            'apple_detection_node = group03.AppleDetection.apple_detection_node:main',
            'talker = group03.AppleDetection.apple_detection_publisher:main',
            'base_cam = group03.AppleDetection.base_cam_subscriber:main',
            'gripper_cam = group03.AppleDetection.gripper_cam_subscriber:main',
        ],
    },
)
