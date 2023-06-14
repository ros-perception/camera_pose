from distutils.core import setup
import glob

scripts = glob.glob(os.path.join('scripts', '*.py'))

setup(
    version='0.5.0',
    scripts=scripts,
    packages=['camera_pose_toolkits'],
    package_dir={}
)

