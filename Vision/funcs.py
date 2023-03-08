import cv2
import staticVars as v
import numpy as np

v.init()


# Define world locations
def genWorldPoint():
    r0 = [[[0, 0, 0],
           [50.8, 0, 0],
           [50.8, 50.8, 0],
           [0, 50.8, 0]]]

    r1 = [[[0, 50.8, 0],
           [0, 0, 0],
           [50.8, 0, 0],
           [50.8, 50.8, 0]]]

    r2 = [[[50.8, 50.8, 0],
           [0, 50.8, 0],
           [0, 0, 0],
           [50.8, 0, 0]]]

    r3 = [[[50.8, 0, 0],
           [50.8, 50.8, 0],
           [0, 50.8, 0],
           [0, 0, 0]]]

    worldPointList = list([r0, r1, r2, r3])

    worldPointList[0] = np.array(r0)
    worldPointList[1] = np.array(r1)
    worldPointList[2] = np.array(r2)
    worldPointList[3] = np.array(r3)

    return worldPointList


def resizeWarp(img, square_size):
    # Get image dimensions
    height, width = img.shape[:2]

    # Save larger of the two variables as square size
    resolution = np.max([width, height])

    # Get dimensions for resizing
    nx, ny = int(resolution / square_size), int(resolution / square_size)

    # Resize image to new dimensions
    resized_image = cv2.resize(img, (nx, ny), interpolation=cv2.INTER_LINEAR)

    # Convert to grayscale
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

    # Otsu Threshold grayscale image
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    return binary


def matchBIT(contour):
    dim = 7  # Set dimension of marker
    square_size = 1  # in meters

    # Initialize array to be filled with binary data
    c1 = np.zeros((dim, dim))

    # Set dimension for resizing of contour to be compared
    width = int(7)
    height = int(7)
    dim1 = (width, height)

    # Resize contour to
    cnt1 = cv2.resize(contour, dim1, interpolation=cv2.INTER_AREA)

    # disp = cv2.resize(contour, (500,500), interpolation=cv2.INTER_AREA)
    # cv2.imshow('resized',disp)

    # Convert image data to binary data
    for i in range(dim):
        for j in range(dim):
            x1, y1 = j * square_size, i * square_size
            x2, y2 = (j + 1) * square_size, (i + 1) * square_size
            c1[i, j] = int(np.mean(cnt1[y1:y2, x1:x2]) > 128)

    # Create bins for rotated contour binary
    c2 = cv2.rotate(c1, cv2.ROTATE_90_CLOCKWISE)
    c3 = cv2.rotate(c2, cv2.ROTATE_90_CLOCKWISE)
    c4 = cv2.rotate(c3, cv2.ROTATE_90_CLOCKWISE)

    if v.fullFeedback:
        print(c1)

    # Convert from 2D to 1D array
    c1 = c1.reshape(-1)
    c2 = c2.reshape(-1)
    c3 = c3.reshape(-1)
    c4 = c4.reshape(-1)

    match1 = np.sum(c1 == v.desiredMarker) / len(v.desiredMarker)
    match2 = np.sum(c2 == v.desiredMarker) / len(v.desiredMarker)
    match3 = np.sum(c3 == v.desiredMarker) / len(v.desiredMarker)
    match4 = np.sum(c4 == v.desiredMarker) / len(v.desiredMarker)

    # Set threshold for match
    threshold = .96

    # Check for matches
    if (match1 > threshold) | (match2 > threshold) | (match3 > threshold) | (
            match4 > threshold):
        if v.fullFeedback:
            print(f"MARKER MATCH! Match:{max(match1, match2, match3, match4)}\n\n")

        if match1 > match2 and match1 > match3 and match1 > match4:
            return True, 0
        elif match2 > match1 and match2 > match3 and match2 > match4:
            return True, 1
        elif match3 > match1 and match3 > match2 and match3 > match4:
            return True, 2
        else:
            return True, 3
    else:
        return False, 999


# returns corners ordered in clockwise order
def orderCornersN(crnrs):
    # Create copy of corners
    crnrs = crnrs.reshape(4, 2)
    ordrd_crnrs = crnrs

    # Calculate center point
    cx = (crnrs[0][0] + crnrs[1][0] + crnrs[2][0] + crnrs[3][0]) / 4.0
    cy = (crnrs[0][1] + crnrs[1][1] + crnrs[2][1] + crnrs[3][1]) / 4.0

    if crnrs[0][0] <= cx and crnrs[0][1] <= cy:  # Top Left Corner
        # Swap Diagonals
        ordrd_crnrs[[1, 3]] = crnrs[[3, 1]]

    else:  # Top right Corner
        ordrd_crnrs[[0, 1]] = crnrs[[1, 0]]
        ordrd_crnrs[[2, 3]] = crnrs[[3, 2]]

    ordrd_crnrs = ordrd_crnrs.reshape(4, 1, 2)
    return ordrd_crnrs
