# C4D-Camera-Calibration

Python script based on OpenCV that calibrates a camera
+
Plugin for C4D that uses calibration and pose estimation. 

calibrate.py is based on the camera calibration example provided with OpenCV. It computes camera calibration params based on a sequence of images of a calibration grid.
measure.py computes the pose of a camera in a series of images using a calibration previously computed with calibrate.py
CameraPose is a C4D plugin that lets users choose a series of images on disk, a calibration computed by calibrate.py and a camera in the C4D scene and compute the pose of the camera for each image and places a plane in the scene in this same pose. 

NOTE: last tested with OpenCV 3.0 on OSX 10.11 and C4D R 17.
