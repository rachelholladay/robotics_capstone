'''
Localization subsystem
'''
from __future__ import print_function

from utils.dataStorage import LocalizationData
from utils.geometry import DirectedPoint # TODO fix imports
import apriltags.src.boost_apriltags as apriltags

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

    def test(self):
        """
        Test function for apriltags library
        """
        detector = apriltags.TagDetector()
        detector.test()
