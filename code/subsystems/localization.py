'''
Localization subsystem
'''
from __future__ import print_function
import math
import time

import numpy as np
from IPython import embed

from utils.dataStorage import LocalizationData
from utils.geometry import DirectedPoint
from utils import constants as cst

import apriltags.src.boost_apriltags as apriltags

class LocalizationSystem(object):
    '''
    Contains localization subsystem. Most localization code is handled via the
    AprilTags C++ library
    '''
    def __init__(self):

        # Calibrated LocalizationData values. Corner and robot
        # locations are normalized on a (0,0) to (1,1) scale.
        self.localizationData = LocalizationData()

        # LocalizationData struct with raw pixel location values
        self._raw_loc = LocalizationData()

        # AprilTag TagDetector object from C++ bindings
        self._detector = None

    def setup(self):
        """
        Calls C++ functions to instantiate AprilTags libary, activate camera,
        and run diagnostics
        """
        self._detector = apriltags.TagDetector()
        self._detector.setup()

    def localize(self, verbose=0):
        """
        Call C++ AprilTags function to get updated localization information
        and update the localization struct
        """
        # while True:
        #     self.detector.detect_apriltags()
        #     if self.detector.num_detected() > 0:
        #         break
        self._detector.detect_apriltags()
        if self._detector.num_detected() is 0:
            return

        corners_detected = []
        robots_detected = []
        for x in range(self._detector.num_detected()):

            tag = self._detector.getTag(x)
        
            # Normalize cos(theta) and sin(theta) elements of H
            # Normalization by H(i,j) / H(2,2)
            # Avoids normalization bugs by only dividing if 
            #   abs(H(2,2)-H(i,j)) > sigma
            # Similar to http://stackoverflow.com/questions/10935047/avoiding-weird-homography-values-when-normalizing
            h00 = tag.h00
            h01 = tag.h01
            if abs(tag.h22 - tag.h00) > cst.SIGMA:
                h00 = h00 / tag.h22
            if abs(tag.h22 - tag.h01) > cst.SIGMA:
                h01 = h01 / tag.h22
            
            x_pixels = tag.cx
            y_pixels = tag.cy
            theta = math.degrees(math.atan2(h01,h00))

            # Fill in raw_loc struct
            if tag.id in cst.TAG_CORNERS:
                corners_detected.append(tag.id)
                self._raw_loc.corners[tag.id] = \
                    DirectedPoint(x_pixels, y_pixels, theta)
            elif tag.id in cst.TAG_ROBOTS:
                robots_detected.append(tag.id)
                self._raw_loc.robots[tag.id] = \
                    DirectedPoint(x_pixels, y_pixels, theta)


            if verbose >= 2:
                pt = 'x:{:0.2f} y:{:0.2f} theta:{:0.2f}'.format(
                        tag.id, x_pixels, y_pixels, theta)
                print("id:",tag.id," ",pt,sep='')
        
        if verbose >= 1:
            print("Corners found:", corners_detected, 
                  "Robots found:", robots_detected)
    def loop(self):
        while(True):
            self.localize(verbose=1)
