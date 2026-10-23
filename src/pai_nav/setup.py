from setuptools import setup
from glob import glob
from os.path import join
package_name = 'pai_nav'
setup(name=package_name,version='0.1.0',packages=[package_name],
 data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
             ('share/'+package_name,['package.xml']),
             (join('share',package_name,'launch'),glob('launch/*.py')),
             (join('share',package_name,'worlds'),glob('worlds/*.sdf')),
             (join('share',package_name,'models'),glob('models/*.json'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='Course participant',maintainer_email='maintainer@example.invalid',
 description='Learning-based obstacle avoidance',license='MIT',
 entry_points={'console_scripts':['nav_node = pai_nav.nav_node:main']} )
