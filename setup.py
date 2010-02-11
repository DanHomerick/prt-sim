# It's not viable to use setuptools's dependency resolution to auto-install
# many of the C/C++ wrapper projects. The situation is especially bad under
# Windows, but it dosen't work well even when compilers are already installed.
### Bootstraps setuptools, for the case where it's not already installed.
##import ez_setup
##ez_setup.use_setuptools()
##
##from setuptools import setup

from distutils.core import setup

# setuptools format:
##install_requires = [
##          'SimPy >= 2.0.1',
###          'protobuf >= 2.3.0',
##          'networkx >= 1.0.1',
###          'numpy >= 1.4.0',  # requires c and fortran compilers. Used binary installer on windows.
###          'chaco >= 3.2.0',  # requires numpy as an unlisted requirement
##          'traits >= 3.2.0', # easy_install worked
###          'scipy >= 0.7.1',  # installed from binary installer on windows
##          'Sphinx'
###          'wxPython >= 2.8.10.1'     # not compatible with easy_install. Used binary installer on windows.
##
##          ],

# distutils format:
requires = [
    'SimPy (>=2.0.1)',
    'protobuf (>=2.3.0)',
    'networkx (>=1.0.1)',
    'numpy (>=1.4.0)',
    'scipy (>=0.7.1)',
    'traits (>=3.2.0)',
    'chaco (>=3.2.0)',
    'wx (>=2.8.10.1)'
    ]

setup(name="pyprt",
      version="0.1.0",
      description="A Personal Rapid Transit simulator",
      author="Dan Homerick",
      author_email="danhomerick@gmail.com",
      url="http://code.google.com/p/prt-sim/",
      packages=['pyprt', 'pyprt.sim', 'pyprt.ctrl', 'pyprt.tests'],
      requires=requires, # distutils style
##      install_requires = install_requires, # setuptools style

##      scripts=['./scripts/pyprt.py'],
      data_files=[('data', ['pyprt/data/default.cfg'])]
      )


##setup(
##    name = "HelloWorld",
##    version = "0.1",
##    packages = find_packages(),
##    scripts = ['say_hello.py'],
##
##    # Project uses reStructuredText, so ensure that the docutils get
##    # installed or upgraded on the target machine
##    install_requires = ['docutils>=0.3'],
##
##    package_data = {
##        # If any package contains *.txt or *.rst files, include them:
##        '': ['*.txt', '*.rst'],
##        # And include any *.msg files found in the 'hello' package, too:
##        'hello': ['*.msg'],
##    }
##
##    # metadata for upload to PyPI
##    author = "Me",
##    author_email = "me@example.com",
##    description = "This is an Example Package",
##    license = "PSF",
##    keywords = "hello world example examples",
##    url = "http://example.com/HelloWorld/",   # project home page, if any
##
##    # could also include long_description, download_url, classifiers, etc.
