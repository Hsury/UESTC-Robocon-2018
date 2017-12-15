from mecanum import Mecanum
from omni import Omni
from merge import Merge
from dash import Dash
from feed import Feed
from flow import Flow
from remote import Remote
from spy import Spy

class Launch():
    '''UESTC 2018 Robocon Team
    Launch Package
    '''
    isLaunched = False

    def __init__(self, baseType='mecanum', debug=True, record=True, viewer=False, ros=False):
        if not Launch.isLaunched:
            Launch.base = Mecanum() if baseType == 'mecanum' else Omni()
            Launch.merge = Merge()
            Launch.dash = Dash(Launch.base, Launch.merge)
            Launch.feed = Feed(Launch.dash)
            Launch.flow = Flow(Launch.dash, debug=debug)
            Launch.remote = Remote(Launch.dash, Launch.flow)
            Launch.spy = Spy(Launch.dash)
            Launch.isLaunched = True
        Launch.base.enable()
        if record:
            Launch.spy.begin()
        if viewer:
            from viewer import Viewer
            Launch.viewer = Viewer(Launch.dash)
        if ros:
            from ros import Ros
            Launch.ros = Ros(Launch.dash)
    
    def exit(self):
        Launch.base.disable()
        Launch.spy.stop()

if __name__=='__main__':
    launch = Launch(baseType='mecanum', debug=True, record=True, viewer=False, ros=False)
    input('Press [Enter] to exit')
    launch.exit()
