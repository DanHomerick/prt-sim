from bbfreeze import Freezer

includes = ['wx.lib.gridmovers',     # Required for TabularEditor to work
            'wx.lib.mixins.listctrl' # Required for TabularEditor to work
            ]
excludes = ['nose',
            'networkx.tests',
            'networkx.drawing',
            'enthought.traits.ui.qt4',
            'enthought.chaco2',
            'enthought.enable2',
            'PyQt4',
            'pyprt.scenarios'] # included separately by installer

freezer = Freezer('dist', includes=includes, excludes=excludes)

# Switch gui_only to True for a release (Broken currently. Unknown why.)
freezer.addScript('pyprt/sim/main.py', gui_only=True)
freezer.addScript('pyprt/ctrl/prt_controller.py', gui_only=True)
freezer.addScript('pyprt/ctrl/gtf_controller.py', gui_only=True)
freezer.include_py = False

freezer()