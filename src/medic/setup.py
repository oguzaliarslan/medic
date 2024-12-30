from setuptools import find_packages, setup
import os
from glob import glob
from distutils.dir_util import copy_tree

package_name = 'medic'

data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:
        
        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                    
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models'),
         glob(os.path.join('models', '*'))),
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = medic.robot_controller:main',
            'perception_node = medic.perception:main', 
            'obstacle_detection_node = medic.obstacle_detection:main',
            'navigation_node = medic.navigation:main',            
        ],
    },
)
