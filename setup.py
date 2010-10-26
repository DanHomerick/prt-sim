# Gets setuptools if it's not already installed.
# See: http://peak.telecommunity.com/DevCenter/setuptools#using-setuptools-without-bundling-it
import ez_setup
ez_setup.use_setuptools()

from setuptools import setup

setup(name='pyprt',
      version='0.1',
      description='A Personal Rapid Transit (PRT) simulator.',
##      long_description='TODO',
      author='Dan Homerick',
      author_email='danhomerick@gmail.com',
      url='http://code.google.com/p/prt-sim/',
      keywords="PRT Personal Rapid Transit Simulator",
      packages=['pyprt'],
      scripts=['pyprt/scripts/spline_explorer.py',
               'pyprt/scripts/spline_explorer.bat'],
      requires=['Python (>=2.6)',
                'SimPy (>=2.0.1)',
                'protobuf (>=2.3.0)',
                'networkx (>=1.3)',
                'numpy (>=1.3.0)',
                'scipy (>=0.7.1)',
                'enthought.traits (>=3.2.0)',
                'enthought.chaco (>=3.2.0)',
                'wx (>=2.8)'],
      install_requires=['SimPy >= 2.0.1',
                        'protobuf >= 2.3.0',
                        'networkx >= 1.3',
                        'numpy >= 1.3.0',
                        'scipy >= 0.7.1',
                        'traits >= 3.2.0',
                        'chaco >= 3.2.0',
##                        'wxPython >= 2.8' # Not suitable for easy_install installation
                        ],
      include_package_data=True, # includes files under SVN control when True
##      test_suite="pyprt.tests", # TODO (Include package data, if needed)
      classifiers=['Development Status :: 3 - Alpha',
                   'Intended Audience :: Science/Research',
                   'Intended Audience :: End Users/Desktop',
                   'License :: OSI Approved :: GNU General Public License (GPL)',
                   'Natural Language :: English',
                   'Operating System :: MacOS :: MacOS X',
                   'Operating System :: Microsoft :: Windows',
                   'Operating System :: POSIX',
                   'Programming Language :: Python :: 2.6',
                   'Topic :: Scientific/Engineering'],
      entry_points = {
          'gui_scripts' : ['prt_simulator = pyprt.sim.main:main'],
          'console_scripts' : []
      }

    )