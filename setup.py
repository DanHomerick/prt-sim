from distutils.core import setup

setup(name="pyPRT",
      version="0.1.0",
      description="A Personal Rapid Transit simulator",
      author="Dan Homerick",
      author_email="danhomerick@gmail.com",
      url="http://code.google.com/p/prt-sim/",
      packages=['pyprt', 'pyprt.sim', 'pyprt.ctrl', 'pyprt.tests'],
      requires=['protobuf (>=2.3.0)', 'networkx (>=5.0)']
      )