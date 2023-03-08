import cv2
import numpy as np


def init():
    global font, criteria, desiredMarker, camMat, dist, markerAxes, detectAll

    # Set font for display
    font = cv2.FONT_HERSHEY_COMPLEX

    # Set criteria for cv2.cornerSubPix
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)

    # Set target marker
    desiredMarker = [0, 0, 0, 0, 0, 0, 0,
                     0, 1, 0, 1, 0, 0, 0,
                     0, 0, 1, 0, 1, 1, 0,
                     0, 0, 1, 1, 0, 0, 0,
                     0, 1, 0, 1, 0, 1, 0,
                     0, 1, 1, 1, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, ]

    camMat = [[871.44787261, 0., 650.25160681],
              [0., 871.52075908, 360.34973424],
              [0., 0., 1.]]
    camMat = np.array(camMat)

    dist = [[1.25687971e-01, -5.62764418e-01, 1.49339503e-03, 5.88682632e-04,
             6.86198927e-01]]
    dist = np.array(dist)

    # Define marker axes from corner
    # markerAxes = [[0., 0., 0.],
    #               [50.8, 0., 0.],
    #               [0., 50.8, 0.],
    #               [0., 0., -50.8]]

    # Define marker axes from center
    markerAxes = [[25.4, 25.4, 0.],
                  [50.8, 25.4, 0.],
                  [25.4, 50.8, 0.],
                  [25.4, 25.4, -25.4]]
    markerAxes = np.array(markerAxes)

    detectAll = True