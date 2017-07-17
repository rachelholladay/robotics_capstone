#!/user/bin/python
"""
Localization subsystem. Calls C++ Apriltags library for raw localization data,
then processes into data usable by the offboard controller.
"""
from __future__ import print_function
import math
import threading

import numpy as np
import cv2

from utils.dataStorage import LocalizationData
from utils.geometry import DirectedPoint
from utils import constants as cst
import subsystems.apriltags.src.boost_apriltags as apriltags


class LocalizationSystem(object):
    """
    Contains localization subsystem. Most localization code is handled via the
    AprilTags C++ library

    Processed localization is pulled from the C++ library and processed into
    usable data for the offboard controller.
    """

    def __init__(self, scaled_dims=[1, 1]):

        # Fixed [width, height] dimensions to scale localization tags to.
        # Calibrated LocalizationData struct scales the corners between
        # [0,0] and scaled_dims
        self.scaled_dims = scaled_dims

        # Calibrated LocalizationData values. Corner and robot
        # locations are normalized on a (0,0) to (1,1) scale.
        # external_data is used outside of the class to pass information
        # between subsystems
        self.data = LocalizationData()
        self.external_data = LocalizationData()

        # LocalizationData struct with raw pixel location values
        self._raw_loc = LocalizationData()

        # AprilTag TagDetector object from C++ bindings
        self._detector = None

        # Thread running localization loop
        self._localization_thread = None
        # Stop flag for thread
        self._stop_flag = False

        self._global_localization_status = False

    def _localize(self, verbose=0):
        """
        Call C++ AprilTags function to get updated localization information
        and update the localization struct

        @param verbose Debug level to display for runtime logging
        """
        self._detector.detect_apriltags()

        corners_detected = []
        robots_detected = []
        for x in range(self._detector.num_detected()):

            tag = self._detector.getTag(x)

            # Normalize cos(theta) and sin(theta) elements of H
            # Normalization by H(i,j) / H(2,2)
            # Avoids normalization bugs by only dividing if
            #   abs(H(2,2)-H(i,j)) > sigma
            # Similar to
            # http://stackoverflow.com/questions/10935047/avoiding-weird-homography-values-when-normalizing
            h00 = tag.h00
            h01 = tag.h01
            if abs(tag.h22 - tag.h00) > cst.EPSILON:
                h00 = h00 / tag.h22
            if abs(tag.h22 - tag.h01) > cst.EPSILON:
                h01 = h01 / tag.h22

            x_pixels = tag.cx
            y_pixels = tag.cy
            theta = math.degrees(math.atan2(h01, h00))

            # Fill in raw_loc struct with found corners and robots
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
                     x_pixels, y_pixels, theta)
                print("id:", tag.id, " ", pt, sep='')

        if verbose >= 1:
            print("Corners found:", corners_detected,
                  "Robots found:", robots_detected)

    def _worker_localization(self, verbose=0):
        """
        Worker function to begin localization
        """
        while(True):
            if self._stop_flag:
                return
            self._localize(verbose=verbose)
            self._getLocalizationData()

    def setup(self, camera=0, filename=""):
        """
        Calls C++ functions to instantiate AprilTags libary, activate camera,
        and run diagnostics

        @param camera OpenCV Camera object to use during initialization in
            the C++ AprilTag library
        @param filename If empty, uses camera value. If filled, attempts
            to read the filename as input camera data. Filename is assumed
            to be a .mp4 video file
        """
        # Detector for hardcoded test file
        # self._detector =
        #   apriltags.TagDetector("/home/neil/data/apriltag_test.mp4")
        self._detector = apriltags.TagDetector(filename)
        self._detector.setup()

    def begin_loop(self, verbose=0):
        """
        Main localization loop, spawns thread

        @param verbose Debug level for runtime logging
        """
        t = threading.Thread(name='localization',
                             target=self._worker_localization,
                             args=(verbose,))
        self._localization_thread = t
        t.start()
        if verbose > 0:
            print("Localization thread started")

    def close(self):
        self._stop_flag = True
        if self._localization_thread is not None:
            self._localization_thread.join()
            pass

    def _getLocalizationData(self):
        """
        Returns LocalizationData struct with points scaled to specified
        rectangle size.
        Computes affine transform between corners of the raw localization
        and the specified scaled_dims, and applies to all corners.

        Only uses 3 fixed corners (all but 1,0) corner
        TODO: Use dynamic 3 corners (possible improvement for consistency)
        """
        found_tags = []

        # Uses all but top_right for now TODO update dynamically
        bottom_left = self._raw_loc.corners[cst.TAG_BOTTOM_LEFT]
        bottom_right = self._raw_loc.corners[cst.TAG_BOTTOM_RIGHT]
        top_left = self._raw_loc.corners[cst.TAG_TOP_LEFT]
        top_right = self._raw_loc.corners[cst.TAG_TOP_RIGHT]

        robot1 = self._raw_loc.robots[cst.TAG_ROBOT1]
        robot2 = self._raw_loc.robots[cst.TAG_ROBOT2]

        # Ensure points for affine tranform computation are valid
        if not bottom_left.valid or not bottom_right.valid or \
           not top_right.valid:
            return  # self.data

        found_tags.append(cst.TAG_BOTTOM_LEFT)
        found_tags.append(cst.TAG_BOTTOM_RIGHT)
        found_tags.append(cst.TAG_TOP_RIGHT)

        raw_pts = np.float32([[bottom_left.x, bottom_left.y],
                              [bottom_right.x, bottom_right.y],
                              # [top_left.x, top_left.y]])
                              [top_right.x, top_right.y]])
        target_pts = np.float32([[0, 0],
                                 [self.scaled_dims[0], 0],
                                 # [0, self.scaled_dims[1]]])
                                 [self.scaled_dims[0], self.scaled_dims[1]]])

        # Append other valid tags onto raw_pts
        if top_left.valid:
            raw_pts = np.vstack((raw_pts, [top_left.x, top_left.y]))
            found_tags.append(cst.TAG_TOP_LEFT)
        if robot1.valid:
            raw_pts = np.vstack((raw_pts, [robot1.x, robot1.y]))
            found_tags.append(cst.TAG_ROBOT1)
        if robot2.valid:
            raw_pts = np.vstack((raw_pts, [robot2.x, robot2.y]))
            found_tags.append(cst.TAG_ROBOT2)
        raw_pts = raw_pts.astype(np.float32)

        # Compute affine transform using only 3 corners
        transform = cv2.getAffineTransform(raw_pts[0:3, :], target_pts)

        # Transform raw tag points into scaled coordinates
        # Calibrated is of the form [x y], each row is a new coordinate
        new_row = np.ones((1, raw_pts.shape[0]))
        new_pts = np.vstack((np.transpose(raw_pts), new_row))
        calibrated = np.transpose(np.matmul(transform, new_pts))

        # Clamp calibrated values from 0 to the respective dimension
        calibrated[:, 0] = np.clip(calibrated[:, 0], 0, self.scaled_dims[0])
        calibrated[:, 1] = np.clip(calibrated[:, 1], 0, self.scaled_dims[1])

        # Push calibrated points into localization struct
        for row_idx, tag_key in enumerate(found_tags):
            if tag_key in cst.TAG_CORNERS:
                self.data.corners[tag_key] = \
                    DirectedPoint(
                        calibrated[row_idx][0],
                        calibrated[row_idx][1],
                        self._raw_loc.corners[tag_key].theta)
            elif tag_key in cst.TAG_ROBOTS:
                self.data.robots[tag_key] = \
                    DirectedPoint(
                        calibrated[row_idx][0],
                        calibrated[row_idx][1],
                        self._raw_loc.robots[tag_key].theta)

        self._global_localization_status = True
        self.external_data = self.data
        self._global_localization_status = False

        return  # self.data

    def getLocalizationData(self):
        """
        External function to gather self.data only when it is not being
        updated
        """
        while self._global_localization_status:
            continue

        return self.external_data
