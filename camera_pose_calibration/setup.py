from distutils.core import setup
import glob
import os

scripts = glob.glob(os.path.join('scripts', '*.py'))

setup(
    version='0.5.0',
    scripts=scripts,
    packages=['camera_pose_calibration'],
    package_dir={'': 'src'}
)

