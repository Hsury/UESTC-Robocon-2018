"""
UESTC 2018 Robocon Team
Variable Manager
"""

from enum import Enum

class Elmo:
    class Config:
        """
        SAMPLE
        """
        def __init__(self):
            self.a=1
    
    class Mode(Enum):
        SPEED = 1
        POSITION = 2

class VElmo:
    """
    Elmo Variable
    """
    def __init__(self):
        pass
