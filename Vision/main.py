import cv2
import numpy as np
import funcs as f
import staticVars as v

# Define a video capture object
captureCam = cv2.VideoCapture(0)

# Set resolution to 720p
captureCam.set(3, 1280)
captureCam.set(4, 720)

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

worldPoints = f.genWorldPoint()
i = 0
tot = 0
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
                    approx = cv2.cornerSubPix(gray, approx, (5, 5), (-1, -1), v.criteria)

                    # CHECKME = approx
                    approx = f.orderCornersN(approx)

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
                    warpbin = cv2.morphologyEx(warpbin, cv2.MORPH_OPEN, np.ones((3, 3)))

                    # Resize image and show cleaned image
                    resized_up = cv2.resize(warpbin, (490, 490), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow("opened", resized_up)

                    # Check if square is marker
                    print("checking")
                    isMarker, rotVal = f.matchBIT(warpbin)

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
                        cv2.putText(frame, STR, centerFrame, v.font, 0.5, (255, 0, 0))

                        # Draw Corners
                        cv2.circle(frame, approxINT[0].ravel(), 2, (255, 0, 0), 5)
                        cv2.circle(frame, approxINT[1].ravel(), 2, (0, 255, 0), 5)
                        cv2.circle(frame, approxINT[2].ravel(), 2, (0, 0, 255), 5)
                        cv2.circle(frame, approxINT[3].ravel(), 2, (0, 255, 255), 5)

                        # Write amount of rotations on top left corner
                        cv2.putText(frame, str(rotVal), approxINT[0].ravel(), v.font, 0.5, (255, 0, 0))

                        # Get pose of marker using solvePnP
                        _, rvec, tvec = cv2.solvePnP(worldPoints[rotVal], approx, v.camMat, v.dist)

                        # Translate 3D points into 2D for display on image
                        drawAxes = 0
                        drawAxes, _ = cv2.projectPoints(v.markerAxes, rvec, tvec, v.camMat, v.dist, drawAxes)
                        drawAxes = np.int0(drawAxes)

                        # Draw pose and axes
                        cv2.line(frame, drawAxes[0, 0, :], drawAxes[1, 0, :], (0, 0, 255), 2)
                        cv2.line(frame, drawAxes[0, 0, :], drawAxes[2, 0, :], (0, 255, 0), 2)
                        cv2.line(frame, drawAxes[0, 0, :], drawAxes[3, 0, :], (255, 0, 0), 2)

                        # convert rvec from radians to degrees
                        rvecDeg = np.rad2deg(rvec)

                        # Write degree of rotation with respect to axis
                        cv2.putText(frame, str(int(rvecDeg[0])), drawAxes[1, 0, :], v.font, 0.8, (0, 0, 255))
                        cv2.putText(frame, str(int(rvecDeg[1])), drawAxes[2, 0, :], v.font, 0.8, (0, 255, 0))
                        cv2.putText(frame, str(int(rvecDeg[2])), drawAxes[3, 0, :], v.font, 0.8, (255, 0, 0))

                        print(approx)
                        if not v.detectAll:
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
