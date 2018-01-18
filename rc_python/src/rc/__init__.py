import atexit
from mecanum import Mecanum
from omni import Omni
from merge import Merge
from dash import Dash
from feed import Feed
from flow import Flow
from remote import Remote
from spy import Spy

__isLaunched = False

base = None
merge = None
dash = None
feed = None
flow = None
remote = None
spy = None
viewer = None
ros = None

def launch(baseType='mecanum', debug=True, record=True, openViewer=False, openROS=False):
    global __isLaunched
    if not __isLaunched:
        global base, merge, dash, feed, flow, remote, spy
        base = Mecanum() if baseType == 'mecanum' else Omni()
        merge = Merge()
        dash = Dash(base, merge)
        feed = Feed(dash)
        flow = Flow(dash, debug=debug)
        remote = Remote(dash, flow)
        spy = Spy(dash)
        __isLaunched = True
    base.enable()
    if record:
        spy.begin()
    if openViewer:
        from viewer import Viewer
        global viewer
        viewer = Viewer(dash)
    if openROS:
        from ros import Ros
        global ros
        ros = Ros(dash)

@atexit.register
def destroy():
    try:
        base.disable()
        spy.stop()
        remote.destroy()
    except:
        pass
