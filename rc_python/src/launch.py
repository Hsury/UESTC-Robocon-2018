from omni import Omni
from merge import Merge
from dash import Dash
from feed import Feed
from editor import Editor
from viewer import Viewer

class Launch():
    '''UESTC 2018 Robocon Team
    Launch Package
    '''
    isLaunched = False

    def __init__(self, showEditor=False, showViewer=True):
        if not Launch.isLaunched:
            Launch.omni = Omni()
            Launch.merge = Merge()
            Launch.dash = Dash(Launch.omni, Launch.merge)
            Launch.feed = Feed(Launch.dash)
            Launch.isLaunched = True
        if showEditor:
            Launch.editor = Editor()
        if showViewer:
            Launch.viewer = Viewer(Launch.dash, Launch.merge)
