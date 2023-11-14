import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion
from pathlib import Path


class CMakeExtension(Extension):
  def __init__(self, name, sourcedir='', target=None):
    Extension.__init__(self, name, sources=[])
    self.sourcedir = os.path.abspath(sourcedir)
    self.target = target


class CMakeBuild(build_ext):
  def run(self):
    try:
      out = subprocess.check_output(['cmake', '--version'])
    except OSError:
      raise RuntimeError("CMake must be installed to build the following extensions: " +
                         ", ".join(e.name for e in self.extensions))

    if platform.system() == "Windows":
      cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
      if cmake_version < '3.1.0':
        raise RuntimeError("CMake >= 3.1.0 is required on Windows")

    for ext in self.extensions:
      self.build_extension(ext)

  def build_extension(self, ext):
    extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
    # required for auto-detection of auxiliary "native" libs
    if not extdir.endswith(os.path.sep):
      extdir += os.path.sep

    add_python_path_args = os.getenv('NO_PYTHON_ARGS', 'NO') == 'NO'
    print('Add python path args: '+str(add_python_path_args))

    cmake_args = []
    # Set our Python version, default to 3.6
    cmake_args += ['-DDARTPY_PYTHON_VERSION:STRING=' +
                   os.getenv('PYTHON_VERSION_NUMBER', '3.6')]
    cmake_args += ['-DPYBIND11_PYTHON_VERSION:STRING=' +
                   os.getenv('PYTHON_VERSION_NUMBER', '3.6')]
    cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                   '-DPYTHON_EXECUTABLE:FILEPATH=' + sys.executable]

    # TODO: We include debug info in our released binaries, because it makes
    # it easier to profile and debug in the wild, which is more valuable than
    # the small performance gain from stripping debug symbols.
    #
    # So this should be 'RelWithDebInfo', not 'Release'. But that seems
    # to causes Azure Pipelines to hang while building our binaries, so this
    # needs investigation.
    cfg = 'Debug' if self.debug else 'Release'
    build_args = ['--config', cfg]
    if ext.target is not None:
      build_args += ['--target', ext.target]

    if platform.system() == "Windows":
      cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
      if sys.maxsize > 2**32:
        cmake_args += ['-A', 'x64']
      build_args += ['--', '/m']
    else:
      cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
      # We need this on the manylinux2010 Docker images to find the correct Python
      if platform.system() == 'Linux' and add_python_path_args:
        # Use ENV vars, and default to 3.8 if we don't specify
        PYTHON_INCLUDE_DIR = os.getenv(
            'PYTHON_INCLUDE', '/opt/python/cp38-cp38/include/python3.8/')
        PYTHON_LIBRARY = os.getenv('PYTHON_LIB', '/opt/python/cp38-cp38/lib/python3.8/')
        print('Using PYTHON_INCLUDE_DIR='+PYTHON_INCLUDE_DIR)
        print('Using PYTHON_LIBRARY='+PYTHON_LIBRARY)
        cmake_args += ['-DPYTHON_INCLUDE_DIR:PATH='+PYTHON_INCLUDE_DIR]
        cmake_args += ['-DPYTHON_LIBRARY:FILEPATH='+PYTHON_LIBRARY]
      build_args += ['--', '-j1']

    env = os.environ.copy()
    env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                          self.distribution.get_version())
    if not os.path.exists(self.build_temp):
      os.makedirs(self.build_temp)
    print('Using CMake Args: '+str(cmake_args))
    subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
    subprocess.check_call(['cmake', '--build', '.'] + build_args,
                          cwd=self.build_temp, env=env)
    # Create the __init__.py in the library folder, so that delocate-wheel works properly
    Path(extdir+"/__init__.py").touch()


with open('VERSION.txt', 'r') as file:
  VERSION = file.read().replace('\n', '')
print("VERSION: "+VERSION)

setup(
    name='nimblephysics',
    version=VERSION,
    author='Keenon Werling',
    author_email='keenonwerling@gmail.com',
    description='A differentiable fully featured physics engine',
    long_description='',
    license='MIT',
    package_dir={'': 'python'},
    packages=['nimblephysics'],
    package_data={'nimblephysics': ['web_gui/*', 'web_gui/*/*', 'web_gui/*/*/*', 'models/*', 'models/*/*', 'models/*/*/*']},
    ext_package='nimblephysics_libs',
    ext_modules=[CMakeExtension('cmake_example', target='_nimblephysics')],
    install_requires=[
        'torch',
        'numpy'
    ],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)
