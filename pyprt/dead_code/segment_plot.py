""" A plot made up of disconnected line segments.
By Robert Kern (provided on the Chaco discussion mailing list).
"""

import warnings

import numpy as np

from enthought.chaco.base_xy_plot import BaseXYPlot
from enthought.enable.api import black_color_trait, ColorTrait, LineStyle
from enthought.traits.api import Float, List, Str, on_trait_change
from enthought.traits.ui import api as tui


class SegmentPlot(BaseXYPlot):
    """ A plot consisting of disconnected line segments.
    """

    # The color of the line.
    color = black_color_trait

    # The color to use to highlight the line when selected.
    selected_color = ColorTrait("lightyellow")

    # The style of the selected line.
    selected_line_style = LineStyle("solid")

    # The name of the key in self.metadata that holds the selection mask
    metadata_name = Str("selections")

    # The thickness of the line.
    line_width = Float(1.0)

    # The line dash style.
    line_style = LineStyle

    # Traits UI View for customizing the plot.
    traits_view = tui.View(tui.Item("color", style="custom"), "line_width", "line_style",
                       buttons=["OK", "Cancel"])

    #------------------------------------------------------------------------
    # Private traits
    #------------------------------------------------------------------------

    # Cached list of non-NaN arrays of (x,y) data-space points; regardless of
    # self.orientation, this is always stored as (index_pt, value_pt).  This is
    # different from the default BaseXYPlot definition.
    _cached_data_pts = List

    # Cached list of non-NaN arrays of (x,y) screen-space points.
    _cached_screen_pts = List

    def hittest(self, screen_pt, threshold=7.0):
        # NotImplemented
        return None

    def get_screen_points(self):
        self._gather_points()
        return [self.map_screen(ary) for ary in self._cached_data_pts]

    #------------------------------------------------------------------------
    # Private methods; implements the BaseXYPlot stub methods
    #------------------------------------------------------------------------

    def _gather_points(self):
        """
        Collects the data points that are within the bounds of the plot and 
        caches them.
        """
        if self._cache_valid or not self.index or not self.value:
            return

        index = self.index.get_data()
        value = self.value.get_data()

        # Check to see if the data is completely outside the view region
        for ds, rng in ((self.index, self.index_range), (self.value, self.value_range)):
            low, high = ds.get_bounds()
            if low > rng.high or high < rng.low:
                return

        if len(index) == 0 or len(value) == 0 or len(index) != len(value):
            self._cached_data_pts = []
            self._cache_valid = True

        size_diff = len(value) - len(index)
        if size_diff > 0:
            warnings.warn('len(value) %d - len(index) %d = %d' \
                          % (len(value), len(index), size_diff))
            index_max = len(index)
            value = value[:index_max]
        else:
            index_max = len(value)
            index = index[:index_max]
        if index_max % 2:
            # We need an even number of points. Exclude the final one and
            # continue.
            warnings.warn('need an even number of points; got %d' % index_max)
            index = index[:index_max-1]
            value = value[:index_max-1]

        # TODO: restore the functionality of rendering highlighted portions 
        # of the line
        #selection = self.index.metadata.get(self.metadata_name, None) 
        #if selection is not None and type(selection) in (ndarray, list) and \
        #        len(selection) > 0:

        # Exclude NaNs and Infs.
        finite_mask = np.isfinite(value) & np.isfinite(index)
        # Since the line segment ends are paired, we need to exclude the whole pair if
        # one is not finite.
        finite_mask[::2] &= finite_mask[1::2]
        finite_mask[1::2] &= finite_mask[::2]
        self._cached_data_pts = [np.column_stack([index[finite_mask],
                                                  value[finite_mask]])]
        self._cache_valid = True

    def _render(self, gc, points, selected_points=None):
        if len(points) == 0:
            return

        gc.save_state()
        try:
            gc.set_antialias(True)
            gc.clip_to_rect(self.x, self.y, self.width, self.height)

            if selected_points is not None:
                self._render_segments(gc, selected_points, self.selected_color_,
                    self.line_width+10.0, self.selected_line_style_)

            # Render using the normal style
            self._render_segments(gc, points, self.color_,
                self.line_width, self.line_style_)
        finally:
            gc.restore_state()

    def _render_segments(self, gc, points, color, line_width, line_style):
        gc.set_stroke_color(color)
        gc.set_line_width(line_width)
        gc.set_line_dash(line_style)
        gc.begin_path()
        for ary in points:
            if len(ary) > 0:
                gc.line_set(ary[::2], ary[1::2])
        gc.stroke_path()

    @on_trait_change('color,line_style,line_width')
    def _redraw(self):
        self.invalidate_draw()
        self.request_redraw()
