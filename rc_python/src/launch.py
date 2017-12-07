from mecanum import Mecanum
from omni import Omni
from merge import Merge
from dash import Dash
from feed import Feed
from remote import Remote

class Launch():
    '''UESTC 2018 Robocon Team
    Launch Package
    '''
    isLaunched = False

    def __init__(self, baseType='mecanum', showEditor=False, showViewer=False, ros=False):
        if not Launch.isLaunched:
            Launch.base = Mecanum() if baseType == 'mecanum' else Omni()
            Launch.merge = Merge()
            Launch.dash = Dash(Launch.base, Launch.merge)
            Launch.feed = Feed(Launch.dash)
            Launch.remote = Remote(Launch.dash, Launch.feed)
            Launch.isLaunched = True
        if showEditor:
            from editor import Editor
            Launch.editor = Editor()
        if showViewer:
            from viewer import Viewer
            Launch.viewer = Viewer(Launch.dash)
        if ros:
            from ros import Ros
            Launch.ros = Ros(Launch.dash)
    
    def exit(self):
        Launch.base.stop()
        Launch.base.disable()

if __name__=='__main__':
    launch = Launch(baseType='mecanum', showEditor=False, showViewer=False, ros=False)
    input('Press [Enter] to exit')
    launch.exit()

