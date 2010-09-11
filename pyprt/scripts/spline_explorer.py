import enthought.traits.api as traits
import enthought.traits.ui.api as ui
import enthought.traits.ui.menu as menu
import enthought.chaco.api as chaco
from enthought.enable.component_editor import ComponentEditor
from enthought.enable.container import Container

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

    target_type = traits.Enum( ('Position', 'Velocity', 'Acceleration', 'Time') )
    plot_container = traits.Instance(Container)
    recalculate = menu.Action(name="Recalculate", action="recalc")
    dump = menu.Action(name="Print", action="dump")

    trait_view = ui.View(
                     ui.HGroup(
                         ui.VGroup(
                             ui.Item(name='target_type', label='Target'),

                             ui.VGroup(
                                 ui.Label('Initial Conditions:'),
                                 ui.Item(name='q_i', label='Position'),
                                 ui.Item(name='v_i', label='Velocity'),
                                 ui.Item(name='a_i', label='Acceleration'),
                                 ui.Item(name='t_i', label='Time'),
                                 ui.Item(label=' 	'),
                                 ui.Label('Final Conditions:'),
                                 ui.Item(name='q_f', label='Position', enabled_when="target_type not in ('Velocity', 'Acceleration')"),
                                 ui.Item(name='v_f', label='Velocity', enabled_when="target_type != 'Acceleration'"),
                                 ui.Item(name='a_f', label='Acceleration'),
                                 ui.Item(name='t_f', label='Time', enabled_when="target_type == 'Time'"),
                                 show_border = True
                             ),

                             ui.VGroup(
                                 ui.Label('Limits:'),
                                 ui.Item(name='v_min', label='Min Velocity'),
                                 ui.Item(name='v_max', label='Max Velocity'),
                                 ui.Item(name='a_min', label='Min Acceleration'),
                                 ui.Item(name='a_max', label='Max Acceleration'),
                                 ui.Item(name='j_min', label='Min Jerk'),
                                 ui.Item(name='j_max', label='Max Jerk'),
                                 ui.Item(name='mass', label='Vehicle Mass'),
                                 show_border=True
                             )
                         ),

                         ui.Item('plot_container', editor=ComponentEditor(), show_label=False)
                    ),

                    title='Cubic Spline Explorer',
                    handler=SEButtonHandler(),
                    buttons=[recalculate, dump],
                    resizable=True,
                    width=1000
                )


    def __init__(self):
        super(SplineExplorer, self).__init__()
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

            self.plotter = CSplinePlotter(self.spline,
                                          self.v_max, self.a_max, self.j_max,
                                          self.v_min, self.a_min, self.j_min,
                                          mass=self.mass)
            self.plot_container = self.plotter.container
        except:
            self.initial = None
            self.final = None
            self.spline = None
            self.plot_container = Container()

    def display(self):
        self.configure_traits()

if __name__ == '__main__':
    explorer = SplineExplorer()
    explorer.display()
