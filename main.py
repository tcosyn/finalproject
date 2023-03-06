import cv2
import numpy as np

# Define a video capture object
captureCam = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Set resolution to 720p
captureCam.set(3, 1280)
captureCam.set(4, 720)

# Set font for display
font = cv2.FONT_HERSHEY_COMPLEX

# Set criteria for cv2.cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)

# Get resolution & FPS
camW = int(captureCam.get(3))
camH = int(captureCam.get(4))
fps = captureCam.get(cv2.CAP_PROP_FPS)

# Calculate center point coordinates
centerW = int(camW / 2)
centerH = int(camH / 2)
centerFrame = (centerW, centerH)

# Print camera information
print(f'Resolution: {camW} x {camH}')
print(f'FPS: {fps}')

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
              [76.2, 25.4, 0.],
              [25.4, 76.2, 0.],
              [25.4, 25.4, -76.2]]
markerAxes = np.array(markerAxes)

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

    print(c1)
    # Convert from 2D to 1D array
    c1 = c1.reshape(-1)
    c2 = c2.reshape(-1)
    c3 = c3.reshape(-1)
    c4 = c4.reshape(-1)

    match1 = np.sum(c1 == desiredMarker) / len(desiredMarker)
    match2 = np.sum(c2 == desiredMarker) / len(desiredMarker)
    match3 = np.sum(c3 == desiredMarker) / len(desiredMarker)
    match4 = np.sum(c4 == desiredMarker) / len(desiredMarker)

    # Set threshold for match
    threshold = .96

    # Check for matches
    if (match1 > threshold) | (match2 > threshold) | (match3 > threshold) | (
            match4 > threshold):
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


worldPoints = genWorldPoint()

while captureCam.isOpened():
    # Read camera data
    ret, frame = captureCam.read()

    if ret:  # If frame is opened
        # Gaussian blur image with kernel size of 5 x 5 & sigmaY = 0
        gBlur = cv2.GaussianBlur(frame, (5, 5), 0)

        # Grayscale image
        gray = cv2.cvtColor(gBlur, cv2.COLOR_BGR2GRAY)

        # Adaptive threshold image using gaussian average w/ region size of 11 x 11 and 5 offset
        # thresh = cv2.adaptiveThreshold(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)

        thresh = ~thresh  # Invert thresholded image

        # Find contours in image using simple approximation
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area from least to greatest
        contours = sorted(contours, key=lambda x: cv2.contourArea(x),
                          reverse=True)  # REPLACE WITH CHILD/PARENT CONTOUR DELETION

        for cnt in contours:
            # Reduce complexity of contours to reduce amount of points
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

            if len(approx) == 4 and cv2.isContourConvex(approx):  # If number of sides equals four & is convex
                area = cv2.contourArea(approx)  # Get area of contour

                # If contour is within threshold and is convex
                if 700.0 < area < 600000:
                    # Draw rejected contours
                    approx = np.float32(approx)
                    approx = cv2.cornerSubPix(gray, approx, (5, 5), (-1, -1), criteria)

                    # CHECKME = approx
                    approx = orderCornersN(approx)

                    dst_pts = np.array([[0, 0], [48, 0], [48, 48], [0, 48]], dtype="float32")

                    # Get perspective transform
                    M = cv2.getPerspectiveTransform(approx, dst_pts)

                    # Warp image from perspective transform
                    warped = cv2.warpPerspective(frame, M, (49, 49))
                    warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

                    # warpbin = cv2.adaptiveThreshold(warped, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)
                    _, warpbin = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                    # Resize image and show perspective transformed image
                    resized_up = cv2.resize(warpbin, (490, 490), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow("warped", resized_up)

                    # Remove small artifacts from the image by morphologically opening the image w/ a 3 x 3 kernel
                    warpbin = cv2.erode(warpbin, np.ones((3, 3)))
                    warpbin = cv2.dilate(warpbin, np.ones((3, 3)))

                    # Resize image and show cleaned image
                    resized_up = cv2.resize(warpbin, (490, 490), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow("opened", resized_up)

                    # Check if square is marker
                    print("checking")
                    isMarker, rotVal = matchBIT(warpbin)

                    if isMarker:
                        approxINT = np.intp(approx)

                        # Outline contour as green in frame
                        # cv2.drawContours(frame, [approxINT], 0, (0, 255, 0), 1)

                        # Calculate center point
                        center_x = (approx[0][0][0] + approx[1][0][0] + approx[2][0][0] + approx[3][0][0]) / 4.0
                        center_y = (approx[0][0][1] + approx[1][0][1] + approx[2][0][1] + approx[3][0][1]) / 4.0

                        center = (center_x, center_y)  # Save coordinates
                        center = np.intp(center)

                        # Draw center point
                        cv2.circle(frame, center, 2, (125, 125, 125), 2)

                        # Draw line from center point to center of marker
                        cv2.line(frame, center, centerFrame, (191, 0, 255), 1)

                        # Calculate offset from the center
                        offsetx = center_x - centerW
                        offsety = centerH - center_y

                        # Create string to write on frame
                        STR = str(int(offsetx)) + ", " + str(int(offsety))

                        # Write offset from center at center of frame
                        cv2.putText(frame, STR, centerFrame, font, 0.5, (255, 0, 0))

                        # Draw Corners
                        cv2.circle(frame, approxINT[0].ravel(), 2, (255, 0, 0), 5)
                        cv2.circle(frame, approxINT[1].ravel(), 2, (0, 255, 0), 5)
                        cv2.circle(frame, approxINT[2].ravel(), 2, (0, 0, 255), 5)
                        cv2.circle(frame, approxINT[3].ravel(), 2, (0, 255, 255), 5)

                        # Write amount of rotations on top left corner
                        cv2.putText(frame, str(rotVal), approxINT[0].ravel(), font, 0.5, (255, 0, 0))

                        # Get pose of marker using solvePnP
                        _, rvec, tvec = cv2.solvePnP(worldPoints[rotVal], approx, camMat, dist)

                        # Translate 3D points into 2D for display on image
                        drawAxes = 0
                        drawAxes, _ = cv2.projectPoints(markerAxes, rvec, tvec, camMat, dist, drawAxes)
                        drawAxes = np.int0(drawAxes)

                        # Draw pose and axes
                        cv2.line(frame, drawAxes[0, 0, :], drawAxes[1, 0, :], (0, 0, 255), 2)
                        cv2.line(frame, drawAxes[0, 0, :], drawAxes[2, 0, :], (0, 255, 0), 2)
                        cv2.line(frame, drawAxes[0, 0, :], drawAxes[3, 0, :], (255, 0, 0), 2)

                        # convert rvec from radians to degrees
                        rvecDeg = np.rad2deg(rvec)

                        # Write degree of rotation with respect to axis
                        cv2.putText(frame, str(int(rvecDeg[0])), drawAxes[1, 0, :], font, 0.8, (0, 0, 255))
                        cv2.putText(frame, str(int(rvecDeg[1])), drawAxes[2, 0, :], font, 0.8, (0, 255, 0))
                        cv2.putText(frame, str(int(rvecDeg[2])), drawAxes[3, 0, :], font, 0.8, (255, 0, 0))

                        print(approx)
                        break
                    else:
                        cv2.drawContours(frame, [np.intp(approx)], 0, (0, 0, 255), 2)

        cv2.imshow("Webcam", frame)  # stream with corner detected points
        key = cv2.waitKey(1)
        if (key == ord('q')) or (key == ord('Q')):
            break
    else:
        break

captureCam.release()
cv2.destroyAllWindows()
