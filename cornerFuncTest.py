import cv2
import numpy as np

def orderCorners1(crnrs):
    ordrdCrnrs = crnrs

    # Calculate Centerpoint
    center_x = (crnrs[0][0] + crnrs[1][0] + crnrs[2][0] + crnrs[3][0]) / 4.0
    center_y = (crnrs[0][1] + crnrs[1][1] + crnrs[2][1] + crnrs[3][1]) / 4.0

    print(crnrs)

    if(crnrs[0][0] <= center_x and crnrs[0][1] <= center_y):
        ordrdCrnrs[[1, 3]] = crnrs[[3, 1]]

    else:
        ordrdCrnrs[[0, 1]] = crnrs[[1, 0]]
        ordrdCrnrs[[2, 3]] = crnrs[[3, 2]]


    print(f'ordrdCrnrs size{ordrdCrnrs.shape}')
    print(ordrdCrnrs)
    return ordrdCrnrs

def orderCorners2(crnrs):
    c = crnrs.tolist()
    print(c)

    if cv2.contourArea(crnrs) < 0:
        c = np.flip(c,0)

    ymin = [*map(min,zip(*c))][1]
    c.sort(key=lambda x: x[1])

    if c[0][1] == ymin:
        tl = c[0]
        tr = c[1]
        br = c[2]
        bl = c[3]
    elif corners[3][1] == y_min:
        bl = corners[0]
        tl = corners[1]
        tr = corners[2]
        br = corners[3]
    else:
        tr = c[0]
        br = c[1]
        bl = c[2]
        top_left = c[3]

    ordered_corners = np.array([tl, tr, br, bl])
    print(ordered_corners)

    return ordered_corners

def orderCorners3(corners):
    # Find the average x and y coordinates
    x_mean = np.mean(corners[:, 0])
    y_mean = np.mean(corners[:, 1])

    # Calculate the angle of each corner relative to the center
    angles = np.arctan2(corners[:, 1] - y_mean, corners[:, 0] - x_mean)

    # Normalize the angles to be between 0 and 2*pi
    angles = np.mod(angles, 2 * np.pi)

    # Find the top-most corner
    top_corner_index = np.argmin(corners[:, 1])

    # Sort the corners counter-clockwise starting from the top-most corner
    corners = corners[np.argsort(np.concatenate((angles[top_corner_index:], angles[:top_corner_index]), axis=None))]

    return corners

def orderCorners1(corners):
    # Find the average x and y coordinates
    x_mean = np.mean(corners[:, 0])
    y_mean = np.mean(corners[:, 1])

    # Calculate the angle of each corner relative to the center
    angles = np.arctan2(corners[:, 1] - y_mean, corners[:, 0] - x_mean)

    # Normalize the angles to be between 0 and 2*pi
    angles = np.mod(angles, 2 * np.pi)

    # Find the top-most corner
    top_corner_index = np.argmin(corners[:, 1])

    # Create a list of labels for each corner
    labels = ['red', 'green', 'blue', 'yellow']

    # Sort the labels and corners counter-clockwise starting from the top-most corner
    labels = [label for label, _ in sorted(zip(labels, corners),
                                           key=lambda x: x[1][0] * np.cos(angles[top_corner_index]) + x[1][1] * np.sin(
                                               angles[top_corner_index]))]
    corners = corners[np.argsort(np.concatenate((angles[top_corner_index:], angles[:top_corner_index]), axis=None))]

    # Create a dictionary to map the original corner index to its new label
    corner_index_to_label = {i: labels[i] for i in range(4)}

    # Use the dictionary to find the label of each corner after rotation
    def get_corner_label(rotated_corner, corner_index_to_label):
        for i, corner in enumerate(corners):
            if np.allclose(corner, rotated_corner):
                return corner_index_to_label[i]

    # Example of how to use the get_corner_label function with a rotated square
    rotated_corners = np.array([[100.0, 0.0], [100.0, 100.0], [0.0, 100.0], [0.0, 0.0]], dtype=np.float32)
    rotated_labels = [get_corner_label(corner, corner_index_to_label) for corner in rotated_corners]
    print(rotated_labels)  # Output: ['green', 'blue', 'yellow', 'red']

    return corners

def orderCorners2(corners):
    center = np.mean(corners, axis=0)
    angles = np.arctan2([corner[1] - center[1] for corner in corners], [corner[0] - center[0] for corner in corners])
    corners = np.array(corners)
    tl = corners[np.argmin(angles), :]
    rightMost = corners[np.argmax(corners[:, 0] - tl[0])]
    D = np.linalg.norm(rightMost - tl)
    sorted_corners = np.array(sorted(corners, key=lambda x: np.arctan2(x[1] - tl[1], x[0] - tl[0])))
    return sorted_corners

def orderCorners(crnrs, numRotations):
    ordrdCrnrs = crnrs

    # Calculate Centerpoint
    center_x = (crnrs[0][0] + crnrs[1][0] + crnrs[2][0] + crnrs[3][0]) / 4.0
    center_y = (crnrs[0][1] + crnrs[1][1] + crnrs[2][1] + crnrs[3][1]) / 4.0

    # print(crnrs)

    if (crnrs[0][0] <= center_x and crnrs[0][1] <= center_y):  # Top Left Corner
        # Swap Diagonals
        ordrdCrnrs[[1, 3]] = crnrs[[3, 1]]

    else:  # Top right Corner

        ordrdCrnrs[[0, 1]] = crnrs[[1, 0]]
        ordrdCrnrs[[2, 3]] = crnrs[[3, 2]]

    for i in range(numRotations):
        ordrdCrnrs = np.insert(ordrdCrnrs, [0], ordrdCrnrs[3])
        ordrdCrnrs = np.resize(ordrdCrnrs, (4, 2))
    #
    # print(f'ordrdCrnrs size{ordrdCrnrs.shape}')
    # print(ordrdCrnrs)

    return ordrdCrnrs

def orderCorners(crnrs, numRotations):
    ordrdCrnrs = crnrs

    # Calculate Centerpoint
    center_x = (crnrs[0][0] + crnrs[1][0] + crnrs[2][0] + crnrs[3][0]) / 4.0
    center_y = (crnrs[0][1] + crnrs[1][1] + crnrs[2][1] + crnrs[3][1]) / 4.0

    # print(crnrs)

    if (crnrs[0][0] <= center_x and crnrs[0][1] <= center_y):  # Top Left Corner
        # Swap Diagonals
        ordrdCrnrs[[1, 3]] = crnrs[[3, 1]]

    else:  # Top right Corner

        ordrdCrnrs[[0, 1]] = crnrs[[1, 0]]
        ordrdCrnrs[[2, 3]] = crnrs[[3, 2]]

    for i in range(numRotations):
        ordrdCrnrs = np.insert(ordrdCrnrs, [0], ordrdCrnrs[3])
        ordrdCrnrs = np.resize(ordrdCrnrs, (4, 2))
    #
    # print(f'ordrdCrnrs size{ordrdCrnrs.shape}')
    # print(ordrdCrnrs)

    return ordrdCrnrs

def orderCornersN(crnrs):
    ordrdCrnrs = crnrs

    # Calculate Centerpoint
    center_x = (crnrs[0][0] + crnrs[1][0] + crnrs[2][0] + crnrs[3][0]) / 4.0
    center_y = (crnrs[0][1] + crnrs[1][1] + crnrs[2][1] + crnrs[3][1]) / 4.0

    # print(crnrs)

    if (crnrs[0][0] <= center_x and crnrs[0][1] <= center_y):  # Top Left Corner
        # Swap Diagonals
        ordrdCrnrs[[1, 3]] = crnrs[[3, 1]]

    else:  # Top right Corner

        ordrdCrnrs[[0, 1]] = crnrs[[1, 0]]
        ordrdCrnrs[[2, 3]] = crnrs[[3, 2]]

    # print(f'ordrdCrnrs size{ordrdCrnrs.shape}')
    # print(ordrdCrnrs)

    return ordrdCrnrs

def orderCorners1(corners):
    # Find the average x and y coordinates
    x_mean = np.mean(corners[:, 0])
    y_mean = np.mean(corners[:, 1])

    # Calculate the angle of each corner relative to the center
    angles = np.arctan2(corners[:, 1] - y_mean, corners[:, 0] - x_mean)

    # Normalize the angles to be between 0 and 2*pi
    angles = np.mod(angles, 2 * np.pi)

    # Find the top-most corner
    top_corner_index = np.argmin(corners[:, 1])

    # Create a list of labels for each corner
    labels = ['red', 'green', 'blue', 'yellow']

    # Sort the labels and corners counter-clockwise starting from the top-most corner
    labels = [label for label, _ in sorted(zip(labels, corners),
                                           key=lambda x: x[1][0] * np.cos(angles[top_corner_index]) + x[1][1] * np.sin(
                                               angles[top_corner_index]))]
    corners = corners[np.argsort(np.concatenate((angles[top_corner_index:], angles[:top_corner_index]), axis=None))]

    # Create a dictionary to map the original corner index to its new label
    corner_index_to_label = {i: labels[i] for i in range(4)}

    # Use the dictionary to find the label of each corner after rotation
    def get_corner_label(rotated_corner, corner_index_to_label):
        for i, corner in enumerate(corners):
            if np.allclose(corner, rotated_corner):
                return corner_index_to_label[i]

    # Example of how to use the get_corner_label function with a rotated square
    rotated_corners = np.array([[100.0, 0.0], [100.0, 100.0], [0.0, 100.0], [0.0, 0.0]], dtype=np.float32)
    rotated_labels = [get_corner_label(corner, corner_index_to_label) for corner in rotated_corners]
    print(rotated_labels)  # Output: ['green', 'blue', 'yellow', 'red']

    return corners

def orderCorners2(corners):
    center = np.mean(corners, axis=0)
    angles = np.arctan2([corner[1] - center[1] for corner in corners], [corner[0] - center[0] for corner in corners])
    corners = np.array(corners)
    tl = corners[np.argmin(angles), :]
    rightMost = corners[np.argmax(corners[:, 0] - tl[0])]
    D = np.linalg.norm(rightMost - tl)
    sorted_corners = np.array(sorted(corners, key=lambda x: np.arctan2(x[1] - tl[1], x[0] - tl[0])))
    return sorted_corners

captureCam = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while captureCam.isOpened():
    ret, frame = captureCam.read()
    if ret:
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = np.float32(gray)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)

        dst = cv2.cornerHarris(gray, 2, 3, 0.04)

        #Corner Detection
        fast = cv2.FastFeatureDetector_create()
        corners = fast.detect(gray, None)
        mask = np.zeros_like(gray)
        for k in corners:
            x, y = k.pt
            cv2.circle(mask, (int(x), int(y)), 2, (255, 255, 255), -1)
        corners_pts = []
        for x in corners:
            corners_pts.append(x.pt)
        corners = np.int0(corners_pts)
        ret, thresh = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
