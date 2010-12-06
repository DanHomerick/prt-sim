#!/usr/bin/env python

import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.traits.ui.menu as menu
import enthought.chaco.api as chaco
from enthought.enable.component_editor import ComponentEditor
from enthought.enable.container import Container
from enthought.pyface.api import FileDialog, OK, confirm, YES

from pyprt.ctrl.trajectory_solver import TrajectorySolver
from pyprt.shared.cspline_plotter import CSplinePlotter
from pyprt.shared.cubic_spline import Knot, CubicSpline

class SEButtonHandler(ui.Handler):

    def recalc(self, info):
        info.object.calc()

    def dump(self, info):
        print "Initial:", info.object.initial
        print "Final:", info.object.final
        print info.object.spline

    def save(self, info):
        path = info.object.get_save_filename()
        if path:
            info.object.save(path)

class SplineExplorer(traits.HasTraits):
    """A simple UI to adjust the parameters and view the resulting splines."""

    v_min = traits.Float(0)
    v_max = traits.Float(15)
    a_min = traits.Float(-5)
    a_max = traits.Float(5)
    j_min = traits.Float(-2.5)
    j_max = traits.Float(2.5)
    mass = traits.Float(400)

    q_i = traits.Float
    v_i = traits.Float
    a_i = traits.Float
    t_i = traits.Float

    q_f = traits.Float(100)
    v_f = traits.Float(0)
    a_f = traits.Float(0)
    t_f = traits.Float(18)

    plot_names = traits.List( ["Position", "Jerk", "Velocity", "Power", "Acceleration"] )
    active_plots = traits.List

    target_type = traits.Enum( ('Position', 'Velocity', 'Acceleration', 'Time') )

    plot_container = traits.Instance(Container)
    recalculate = menu.Action(name="Recalculate", action="recalc")
    dump = menu.Action(name="Print", action="dump")
    save = menu.Action(name="Save", action="save")
    trait_view = ui.View(
                     ui.HGroup(
                         ui.VGroup(
                             ui.Item(name='target_type', label='Target'),

                             ui.VGroup(
                                 ui.Item(name='active_plots', show_label=False,
                                         editor=ui.CheckListEditor(
                                             cols=3,
                                             name='plot_names'),
                                         style='custom'),
                                 label='Show Plots',
                                 show_border=True
                             ),

                             ui.VGroup(
                                 ui.Item(name='q_i', label='Position'),
                                 ui.Item(name='v_i', label='Velocity'),
                                 ui.Item(name='a_i', label='Acceleration'),
                                 ui.Item(name='t_i', label='Time'),
                                 label='Initial Conditions',
                                 show_border=True
                             ),

                             ui.VGroup(
                                 ui.Item(name='q_f', label='Position', enabled_when="target_type not in ('Velocity', 'Acceleration')"),
                                 ui.Item(name='v_f', label='Velocity', enabled_when="target_type != 'Acceleration'"),
                                 ui.Item(name='a_f', label='Acceleration'),
                                 ui.Item(name='t_f', label='Time', enabled_when="target_type == 'Time'"),
                                 label='Final Conditions:',
                                 show_border=True
                             ),

                             ui.VGroup(
                                 ui.Item(name='v_min', label='Min Velocity'),
                                 ui.Item(name='v_max', label='Max Velocity'),
                                 ui.Item(name='a_min', label='Min Acceleration'),
                                 ui.Item(name='a_max', label='Max Acceleration'),
                                 ui.Item(name='j_min', label='Min Jerk'),
                                 ui.Item(name='j_max', label='Max Jerk'),
                                 ui.Item(name='mass', label='Vehicle Mass'),
                                 label='Constraints',
                                 show_border=True
                             )
                         ),

                         ui.Item('plot_container', editor=ComponentEditor(), show_label=False)
                    ),

                    title='Cubic Spline Explorer',
                    handler=SEButtonHandler(),
                    buttons=[recalculate, dump, save],
                    resizable=True,
                    width=1000
                )


    def __init__(self):
        super(SplineExplorer, self).__init__()
        self.active_plots = self.plot_names[:]
        self.active_plots.remove("Power")
        self.calc()

    def calc(self):
        try:
            self.solver = TrajectorySolver(self.v_max, self.a_max, self.j_max,
                                           self.v_min, self.a_min, self.j_min)
            self.initial = Knot(self.q_i, self.v_i, self.a_i, self.t_i)
            self.final = Knot(self.q_f, self.v_f, self.a_f, self.t_f)

            if self.target_type == 'Position':
                self.spline = self.solver.target_position(self.initial, self.final)
            elif self.target_type == 'Velocity':
                self.spline = self.solver.target_velocity(self.initial, self.final)
            elif self.target_type == 'Acceleration':
                self.spline = self.solver.target_acceleration(self.initial, self.final)
            elif self.target_type == 'Time':
                self.spline = self.solver.target_time(self.initial, self.final)

            pos = vel = accel = jerk = power = False
            if "Position" in self.active_plots: pos=True
            if "Velocity" in self.active_plots: vel=True
            if "Acceleration" in self.active_plots: accel=True
            if "Jerk" in self.active_plots: jerk=True
            if "Power" in self.active_plots: power=True

            self.plotter = CSplinePlotter(self.spline,
                                          self.v_max, self.a_max, self.j_max,
                                          self.v_min, self.a_min, self.j_min,
                                          mass=self.mass,
                                          plot_pos=pos,
                                          plot_vel=vel,
                                          plot_accel=accel,
                                          plot_jerk=jerk,
                                          plot_power=power)
            self.plot_container = self.plotter.container
        except:
            self.initial = None
            self.final = None
            self.spline = None
            self.plot_container = Container()

    def display(self):
        self.configure_traits()

    def get_save_filename(self):
        """Get a filename from the user via a FileDialog. Returns the filename."""
        dialog = FileDialog(action="save as", default_filename="spline_00", wildcard="*.png")
        dialog.open()
        if dialog.return_code == OK:
            return dialog.path

    def save(self, path):
        """Save an image of the plot. Does not catch any exceptions."""
        # Create a graphics context of the right size
        win_size = self.plot_container.outer_bounds
        plot_gc = chaco.PlotGraphicsContext(win_size)
        #plot_gc.set_fill_color("transparent")
        # Place the plot component into it
        plot_gc.render_component(self.plot_container)

        # Save out to the user supplied filename
        plot_gc.save(path)

    def _active_plots_changed(self):
        self.calc()

    def _target_type_changed(self):
        self.calc()

if __name__ == '__main__':
    explorer = SplineExplorer()
    explorer.display()
