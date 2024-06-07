import cv2
import numpy as np
from controller import Robot, Camera

# Initialize the robot and camera
robot = Robot()
timestep = int(robot.getBasicTimeStep())
print(robot.getName() )

# Get the camera device and enable it
camera = robot.getDevice('camera')
camera.enable(timestep)

# Parameters for the chessboard pattern
chessboard_size = (10, 10)
w = 640
h = 480
square_size = 0.03  # Size of a square in your defined unit (meters, etc.)

# Prepare object points
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Main loop
image_count = 0
while robot.step(timestep) != -1 and image_count < 100:  # Capture 20 images
    # Get the camera image
    # image = camera.getImageArray()
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((480,640, 4))
    image = image[:,:,:3]
    
    # Convert the image to a numpy array and flip it
    frame = np.array(image, dtype=np.uint8)
    frame = np.flip(frame, axis=0)  # Flip the image vertically

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
    
    # If found, add object points and image points
    if ret:

        objpoints.append(objp)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(gray, chessboard_size, corners, ret)
        image_count += 1
        print(image_count)

    # Display the resulting frame
    cv2.imshow('Frame', gray)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When enough images are captured, calibrate the camera
if len(objpoints) > 0:
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Print the calibration results
    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coeffs)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 1, (w,h))

    dst = cv2.undistort(gray, camera_matrix, dist_coeffs, None, newcameramtx)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    
    print( "total error: {}".format(mean_error/len(objpoints)) )

else:
    print("Not enough images for calibration")


# Cleanup
camera.disable()
cv2.destroyAllWindows()
