 -- Preparing a Release --
For Windows, prepare a frozen executable, then create an InnoSetup installer.
Requires:
 - SVN checkout: svn checkout http://prt-sim.googlecode.com/svn/trunk/ prt-sim
 - bbfreeze: http://pypi.python.org/pypi/bbfreeze/
 - InnoSetup: http://www.jrsoftware.org/isinfo.php

To create the frozen executable(s), from the prt-sim checkout directory run:

python bb_freeze_setup.py

When completed, a 'dist' directory should have been created.

To create the program installer, launch InnoIDE and open:
prt-sim/inno_setup_script.iss

Click the compile button create the installer. The resulting setup.exe file
will be found in:
prt-sim/Output
