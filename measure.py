#!/usr/bin/env python

import numpy as np
import cv2

# local modules
from common import splitfn

# built-in modules
import os

USAGE = '''
USAGE: measure.py [--square_size] [<calibration dir>] [<image dir>]
'''

if __name__ == '__main__':
    import sys
    import getopt
    from glob import glob

	# Get Command line arguments
    args, dirs = getopt.getopt(sys.argv[1:], '', ['square_size='])
    args = dict(args)

	# Determine the path of this script
    script = os.path.realpath(sys.argv[0])
    scriptPath = os.path.split( script )[0]

	# Stop if no input image dir specified
    if not dirs:
        sys.exit("No arguments provided. Need: <calibration dir> <image dir>")

	# Determine complete path of images and make sure it exists
    imageDir = dirs[1]
    imagesCompletePath = os.path.join( scriptPath, imageDir )
    if not os.path.isdir(imagesCompletePath):
        sys.exit("Dir " + imagesCompletePath + " does not exist.")

	# Get the list of image files
    extensions = ("*.pgn","*.jpg","*.jpeg","*.JPG",)
    img_names = []
    for ext in extensions:
        img_names.extend( glob( os.path.join( imagesCompletePath, ext ) ) )

    # Get size of squares on the calibration grid
    square_size = float(args.get('--square_size', 1.0))

    # Load camera matrix and distortion coefs
    calibrationDir = dirs[0]
    calibCompletePath = os.path.join( scriptPath, calibrationDir )
    if not os.path.isdir(calibCompletePath):
        sys.exit("Dir " + calibCompletePath + " does not exist.")

    cameraMatrixFilename = os.path.join( calibCompletePath, "camera-matrix.npy")
    try:
        cameraMatrix = np.load( cameraMatrixFilename )
    except:
        sys.exit("Cannot read camera matrix.")

    distortionFilename = os.path.join( calibCompletePath, "dist-coefs.npy")
    try:
        distCoefs = np.load( distortionFilename )
    except:
        sys.exit("Cannot read camera matrix.")    

    pattern_size = (8, 6)
    pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
    pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    h, w = 0, 0
    for fn in img_names:
        print 'processing %s...' % fn,
        img = cv2.imread(fn, 0)
        if img is None:
          print "Failed to load", fn
          continue

        h, w = img.shape[:2]
        found, corners = cv2.findChessboardCorners(img, pattern_size)
        if found:
            term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
        if not found:
            print 'chessboard not found'
            continue

        imgPoints = corners.reshape(-1,2)
        retVal, rvec, tvec = cv2.solvePnP( pattern_points, imgPoints, cameraMatrix, distCoefs )

        print 'ok' 
        print 'translation: ( {0}, {1}, {2} )'.format( tvec[0], tvec[1], tvec[2] )
        print 'rotation: ( {0}, {1}, {2} )\n'.format( rvec[0], rvec[1], rvec[2] )

    cv2.destroyAllWindows()
