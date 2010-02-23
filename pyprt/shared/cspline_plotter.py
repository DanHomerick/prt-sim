import enthought.chaco.api as chaco
import enthought.chaco.tools.api as tools
import enthought.traits.api as traits
import enthought.traits.ui.api as ui
from enthought.enable.component_editor import ComponentEditor
import numpy

from cubic_spline import CubicSpline

class CSplinePlotter(traits.HasTraits):
    """Generates and displays a plot for a cubic spline."""
    container = traits.Instance(chaco.OverlayPlotContainer)
    plotdata = traits.Instance(chaco.ArrayPlotData)
    traits_view = ui.View(
        ui.Item('container',editor=ComponentEditor(), show_label=False),
        width=500, height=500, resizable=True, title='CubicSpline Plot')

    def __init__(self, cubic_spline, velocity_max=0, acceleration_max=0, jerk_max=0, velocity_min=None, acceleration_min=None, jerk_min=None, title=""):

        super(CSplinePlotter, self).__init__()
        self.cspline = cubic_spline

        self.v_max = velocity_max
        self.a_max = acceleration_max
        self.j_max = jerk_max

        self.v_min = 0 if velocity_min is None else velocity_min
        self.a_min = -self.a_max if acceleration_min is None else acceleration_min
        self.j_min = -self.j_max if jerk_min is None else jerk_min
        self.title = title

        self.container = chaco.OverlayPlotContainer(padding=50, fill_padding=True, bgcolor="lightgray")
        self.make_plotdata()
        self.make_plots()

    def make_plotdata(self):
        knot_times = self.cspline.t
        sample_times = numpy.linspace(self.cspline.t[0], self.cspline.t[-1], 50)
        endpoint_times = numpy.array([self.cspline.t[0], self.cspline.t[-1]])

        positions = []
        velocities = []
        for t in sample_times:
            sample = self.cspline.evaluate(t)
            positions.append(sample.pos)
            velocities.append(sample.vel)
        accelerations = self.cspline.a
        jerks = numpy.array([((af-ai)/hi if hi else 0) for ai, af, hi in zip(self.cspline.a[:-1],
                                                              self.cspline.a[1:],
                                                              self.cspline.h)] + [0])

        max_vel = numpy.array([self.v_max for t in endpoint_times])
        min_vel = numpy.array([self.v_min for t in endpoint_times])

        max_accel = numpy.array([self.a_max for t in endpoint_times])
        min_accel = numpy.array([self.a_min for t in endpoint_times])

        max_jerk = numpy.array([self.j_max for t in endpoint_times])
        min_jerk = numpy.array([self.j_min for t in endpoint_times])

        self.plotdata = chaco.ArrayPlotData(positions=positions,
                                       endpoint_times=endpoint_times,
                                       knot_times=knot_times,
                                       sample_times=sample_times,
                                       velocities=velocities,
                                       accelerations=accelerations,
                                       jerks=jerks,
                                       max_vel=max_vel,
                                       min_vel=min_vel,
                                       max_accel=max_accel,
                                       min_accel=min_accel,
                                       max_jerk=max_jerk,
                                       min_jerk=min_jerk
                                       )

    def make_plots(self):
        colors = {'pos':'black', 'vel':'blue', 'accel':'red', 'jerk':'green'}

        main_plot = chaco.Plot(self.plotdata, padding=0)
        main_plot.y_axis.title="Velocity (m/s), Accel (m/s2), Jerk (m/s3)"
        vel_plot = main_plot.plot(("sample_times", "velocities"), type="line", color=colors['vel'], line_width=2)
        accel_plot = main_plot.plot(("knot_times", "accelerations"), type="line", color=colors['accel'], line_width=2)
        jerk_plot = main_plot.plot(("knot_times", "jerks"), type="line", color=colors['jerk'], line_width=2, render_style="connectedhold")

        # draw limits
        max_vel_plot = main_plot.plot(("endpoint_times", "max_vel"), color=colors['vel'], line_style='dash', line_width=0.60)
        min_vel_plot = main_plot.plot(("endpoint_times", "min_vel"), color=colors['vel'], line_style='dash', line_width=0.60)
        max_accel_plot = main_plot.plot(("endpoint_times", "max_accel"), color=colors['accel'], line_style='dash', line_width=0.55)
        min_accel_plot = main_plot.plot(("endpoint_times", "min_accel"), color=colors['accel'], line_style='dash', line_width=0.55)
        max_jerk_plot = main_plot.plot(("endpoint_times", "max_jerk"), color=colors['jerk'], line_style='dash', line_width=0.45)
        min_jerk_plot = main_plot.plot(("endpoint_times", "min_jerk"), color=colors['jerk'], line_style='dash', line_width=0.45)
        self.container.add(main_plot)

        # plot positions (on a separate scale from the others)
        pos_plot = chaco.create_line_plot([self.plotdata.arrays["sample_times"], self.plotdata.arrays["positions"]], color=colors['pos'], width=2)
        self.container.add(pos_plot)

        # add a second y-axis for the positions
        pos_y_axis = chaco.PlotAxis(pos_plot, orientation="right", title="Position (meters)")
        self.container.overlays.append(pos_y_axis)

        # make Legend
        legend = chaco.Legend(component=self.container, padding=20, align="ul")
        legend.tools.append(tools.LegendTool(legend, drag_button="right"))
        self.container.overlays.append(legend)

        legend.plots = {'pos':pos_plot, 'vel':vel_plot, 'accel':accel_plot, 'jerk':jerk_plot}

        # Add title, if any
        if self.title:
            main_plot.title = self.title

    def display_plot(self):
        self.configure_traits()


