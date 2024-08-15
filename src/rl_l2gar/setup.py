# from setuptools import setup
from distutils.core import setup

setup(
  name='rl_l2gar',
  version='0.0.0',  # 这个版本号必须要和package.xml中的版本号一致
  packages=['rl_gazebo_class','utils','mdp'],  # 这里的包名必须要和src文件夹下的包名一致
  package_dir={'': 'scripts'},  
)