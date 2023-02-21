import cv2
import numpy as np
from scipy.spatial import distance as dist

# Define a video capture object
captureCam = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Set resolution to 720p
captureCam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
captureCam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
captureCam.set(cv2.CAP_PROP_FPS, 60)

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

    # Set target marker
    desiredMarker = [0, 0, 0, 0, 0, 0, 0,
                     0, 1, 0, 1, 0, 0, 0,
                     0, 0, 1, 0, 1, 1, 0,
                     0, 0, 1, 1, 0, 0, 0,
                     0, 1, 0, 1, 0, 1, 0,
                     0, 1, 1, 1, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, ]

    # Initialize array to be filled with binary data
    cropped_bin1 = np.zeros((dim, dim))

    # Set dimension for resizing of contour to be compared
    width = int(7)
    height = int(7)
    dim1 = (width, height)

    # Resize contour to
    cnt1 = cv2.resize(contour, dim1, interpolation=cv2.INTER_AREA)

    # Convert image data to binary data
    for i in range(dim):
        for j in range(dim):
            x1, y1 = j * square_size, i * square_size
            x2, y2 = (j + 1) * square_size, (i + 1) * square_size
            cropped_bin1[i, j] = int(np.mean(cnt1[y1:y2, x1:x2]) > 128)

    # Create bins for rotated contour binary
    cropped_bin2 = cv2.rotate(cropped_bin1, cv2.ROTATE_90_CLOCKWISE)
    cropped_bin3 = cv2.rotate(cropped_bin2, cv2.ROTATE_90_CLOCKWISE)
    cropped_bin4 = cv2.rotate(cropped_bin3, cv2.ROTATE_90_CLOCKWISE)

    # Convert from 2D to 1D array
    cropped_bin1 = cropped_bin1.reshape(-1)
    cropped_bin2 = cropped_bin2.reshape(-1)
    cropped_bin3 = cropped_bin3.reshape(-1)
    cropped_bin4 = cropped_bin4.reshape(-1)

    matchRating1 = np.sum(cropped_bin1 == desiredMarker) / len(desiredMarker)
    matchRating2 = np.sum(cropped_bin2 == desiredMarker) / len(desiredMarker)
    matchRating3 = np.sum(cropped_bin3 == desiredMarker) / len(desiredMarker)
    matchRating4 = np.sum(cropped_bin4 == desiredMarker) / len(desiredMarker)

    # Set threshold for match
    threshold = .96

    # Check for matches
    if (matchRating1 > threshold) | (matchRating2 > threshold) | (matchRating3 > threshold) | (
            matchRating4 > threshold):
        print(f"MARKER MATCH! Match:{max(matchRating1, matchRating2, matchRating3, matchRating4)}\n\n")
        if (matchRating1 > matchRating2 and matchRating1 > matchRating3 and matchRating1 > matchRating4):
            return True, 0
        elif (matchRating2 > matchRating1 and matchRating2 > matchRating3 and matchRating2 > matchRating4):
            return True, 1
        elif (matchRating3 > matchRating1 and matchRating3 > matchRating2 and matchRating3 > matchRating4):
            return True, 2
        else:
            return True, 3
    else:
        return False, 999


def orderCornersN(crnrs):
    # Create copy of corners
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

    return ordrd_crnrs


while captureCam.isOpened():
    # Read camera data
    ret, frame = captureCam.read()

    if ret:  # If frame is opened
        # Gaussian blur image with kernel size of 5 x 5 & sigmaY = 0
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # Grayscale image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Adaptive threshold image using gaussian average w/ region size of 11 x 11 and 5 offset
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)

        thresh = ~thresh  # Invert thresholded image

        # Find contours in image using simple approximation
        contours, _ = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area from least to greatest
        contours = sorted(contours, key=lambda x: cv2.contourArea(x))  # REPLACE WITH CHILD/PARENT CONTOUR DELETION

        for cnt in contours:
            # Reduce complexity of contours to reduce amount of points
            approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)

            if len(approx) == 4:  # If number of sides equals four
                area = cv2.contourArea(approx)  # Get area of contour

                # If contour is within threshold and is convex
                if 10000.0 < area < 600000 and cv2.isContourConvex(approx):

                    # cv2.drawContours(frame, [approx], 0, (0, 0, 255), 2)

                    rect = cv2.minAreaRect(approx)  # Create bounding rectangle with minimum bounding area
                    box = cv2.boxPoints(rect)  # Convert Box2D into set of corners for drawing
                    box = np.intp(box)  # Cast as int

                    (x, y, w, h) = cv2.boundingRect(cnt)  # Get bounding rectangle for contour extraction
                    crop_img = frame[y:y + h, x:x + w]  # Extract frame with rectangle
                    src_pts = box.astype("float32")  # Cast as float32 & set source points

                    # Set destination points
                    dst_pts = np.array([[0, h - 1], [0, 0], [w - 1, 0], [w - 1, h - 1]], dtype="float32")

                    # Get perspective transform
                    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

                    # Warp image from perspective transform
                    warped = cv2.warpPerspective(frame, M, (w, h))

                    # Convert warped image to binary
                    _, warped_bin = cv2.threshold(warped, 128, 255, cv2.THRESH_BINARY)

                    # Resize warped image
                    rWarp = resizeWarp(warped, 5.0)

                    # Resize warped image with square size 5
                    rWarp = resizeWarp(warped, 5.0)

                    # Check if square is marker
                    isMarker, rotVal = matchBIT(rWarp)

                    if isMarker:
                        # Outline contour as green in frame
                        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)

                        # Convert Box2D into set of corners (float32) for drawing
                        corner = cv2.boxPoints(rect)

                        # Refine corner positions to subpixel level
                        corners = cv2.cornerSubPix(gray, corner, (5, 5), (-1, -1), criteria)

                        # Calculate center point
                        center_x = (corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4.0
                        center_y = (corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4.0

                        center_x = np.int0(center_x)  # Cast as int
                        center_y = np.int0(center_y)  # Cast as int
                        center = (center_x, center_y)  # Save coordinates

                        # Draw center point
                        cv2.circle(frame, center, 2, (125, 125, 125), 2)

                        # Draw line from center point to center of marker
                        cv2.line(frame, center, centerFrame, (191, 0, 255), 2)

                        # Calculate offset from the center
                        offsetx = center_x - centerW
                        offsety = centerH - center_y

                        # Create string to write on frame
                        STR = str(offsetx) + ", " + str(offsety)

                        # Write offset from center at center of frame
                        cv2.putText(frame, STR, centerFrame, font, 0.5, (255, 0, 0))

                        newCrnrs = orderCornersN(corners)  # Order corners
                        cornersINT = np.int0(newCrnrs)  # Cast as int

                        # Draw Corners
                        cv2.circle(frame, cornersINT[0].ravel(), 2, (255, 0, 0), 5)
                        cv2.circle(frame, cornersINT[1].ravel(), 2, (0, 255, 0), 5)
                        cv2.circle(frame, cornersINT[2].ravel(), 2, (0, 0, 255), 5)
                        cv2.circle(frame, cornersINT[3].ravel(), 2, (0, 255, 255), 5)

                        # Write amount of rotations on top left corner
                        cv2.putText(frame, str(rotVal), cornersINT[0].ravel(), font, 0.5, (255, 0, 0))

                        break

        cv2.imshow("Webcam", frame)  # stream with corner detected points
        key = cv2.waitKey(1)
        if (key == ord('q')) or (key == ord('Q')):
            break
    else:
        break

cv2.destroyAllWindows()
