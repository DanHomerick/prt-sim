## Installing Dependencies ##

The sim and vehicle controllers are written in Python. This guide assumes that you have some familiarity with installing Python packages. The following are required (package name, version num):

  * Python (>= 2.6)
  * SimPy (>=2.0.1)
  * protobuf (>=2.3.0)
  * networkx (>=1.0.1)
  * numpy (>=1.4.0)
  * scipy (>=0.7.1)
  * traits (>=3.2.0)
  * chaco (>=3.2.0)
  * wx (>=2.8.10)  _also known as wxPython_

Most of these, at least under Windows, can **not** be installed via '[easy\_install](http://pypi.python.org/pypi/setuptools)' or '[pip](http://pypi.python.org/pypi/pip)', if you're familiar with those Python package management tools.

If you're running Windows, I recommend downloading [PythonXY](http://www.pythonxy.com/). During installation, you may configure which packages to include. You may deselect all packages except for:
  * wxPython
  * Numpy
  * SciPy
  * SetupTools
  * Networkx
  * ETS  (Provides Chaco and Traits)
  * VTK
  * nose

You can get the two remaining dependencies, SimPy and protobuf from [here](http://simpy.sourceforge.net/archive.htm) and [here](http://code.google.com/p/protobuf/downloads/list), respectively.

If you're running Mac OSX or Linux, then you may have better luck using 'easy\_install' or 'pip' since those platforms generally have C++ and Fortan compilers installed, which some of the dependencies require. Even here, though, I've had trouble getting [Chaco](http://code.enthought.com/projects/chaco/) and [Traits](http://code.enthought.com/projects/traits/) to compile from source, so it may be worth looking at the [Enthought Python Distribution](http://www.enthought.com/products/epd.php) (EPD), which is similar to PythonXY in that supplies a large number of Python packages... but the EPD is not free for non-academic use, which is part of why I steer Windows users towards PythonXY instead.

Once the dependencies are installed, you may checkout the project with:
`svn checkout http://prt-sim.googlecode.com/svn/trunk/ prt-sim`

In a terminal or console window, navigate to the checkout directory and run:
`python setup.py develop`

## A First Simulation ##

Once installed, a sample simulation can be started by navigating to directory:

`prt-sim/pyprt/sim/`

and entering

`python main.py  ../scenarios/FiveStation_Merge/config.ini`

Once the simulator is done launching, you should see a map with a simple track layout. To begin the sim, using the menu:

`Simulation -> Connect External Controller ...`

`Simulation -> Start Sim`