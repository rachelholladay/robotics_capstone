'''
Localization subsystem
'''
from geometry import DirectedPoint # TODO fix imports

class LocalizationSystem(object):
    '''
    Contains localization subsystem. Most localization code is handled via the
    AprilTags C++ library
    '''
    def __init__(self):
        self.localizationData = LocalizationData()

    def setup(self):
        """
        Calls C++ functions to instantiate AprilTags libary, activate camera,
        and run diagnostics
        """
        pass

    def localize(self):
        """
        Call C++ AprilTags function to get updated localization information
        and update the localization struct
        """
        # TODO update LocalizationData struct
        return self.localization


class LocalizationData:
    """
    Struct containing localization data for each of the 4 corners and 2 robots
    """
    def __init__(self):
        corners = [DirectedPoint(), DirectedPoint(), DirectedPoint(), \
            DirectedPoint()]
        robots = [DirectedPoint(), DirectedPoint()]

