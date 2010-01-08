import sys
import gtk, gobject
import pylab as p
import matplotlib.numerix as nx
import time
from matplotlib.collections import LineCollection

fig = p.figure(1)
ax = p.subplot(111)
canvas = ax.figure.canvas

# for profiling
tstart = time.time()

# create the initial line
x = nx.arange(0,2*nx.pi,0.01)
l_pos_a = [((0,0),(1,1)), ((0,1),(1,0))]
l_pos_b = [((0,0),(0,1)), ((1,1),(1,0))]
line_c = LineCollection(l_pos_a, animated=True)
line, = p.plot(x, nx.sin(x), animated=False)
ax.add_collection(line_c)

def update_line(*args):
    # save the clean slate background -- everything but the animated line
    # is drawn and saved in the pixel buffer background
    if update_line.background is None:
        update_line.background = canvas.copy_from_bbox(ax.bbox)

    # restore the clean slate background
    canvas.restore_region(update_line.background)
    # update the data
#    line.set_ydata(nx.sin(x+update_line.cnt/10.0))
    if (update_line.cnt/10) % 2 == 0:
        line_c.set_verts(l_pos_b)
    else:
        line_c.set_verts(l_pos_a)
    # just draw the animated artist
    ax.draw_artist(line_c)
    # just redraw the axes rectangle
    canvas.blit(ax.bbox)

    if update_line.cnt==50:
        # print the timing info and quit
        print 'FPS:' , update_line.cnt/(time.time()-tstart)
        sys.exit()

    update_line.cnt += 1
    return True
update_line.background = None
update_line.cnt = 0

gobject.idle_add(update_line)

p.show()
