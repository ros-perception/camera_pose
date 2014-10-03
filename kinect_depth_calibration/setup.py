from distutils.core import setup
import glob

scripts = glob.glob(os.path.join('scripts', '*.py'))

setup(
    version='0.5.0',
    scripts=scripts,
    packages=['kinect_depth_calibration'],
    package_dir={}
)

