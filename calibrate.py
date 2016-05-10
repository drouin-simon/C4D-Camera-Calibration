#!/usr/bin/env python

import numpy as np
import cv2

# local modules
from common import splitfn

# built-in modules
import os

USAGE = '''
USAGE: calibration.py [--save <filename>] [--debug <output path>] [--square_size] [<image dir>]
'''

if __name__ == '__main__':
    import sys
    import getopt
    from glob import glob

	# Get Command line arguments
    args, imageDir = getopt.getopt(sys.argv[1:], '', ['save=', 'debug=', 'square_size='])
    args = dict(args)

	# Determine the path of this script
    script = os.path.realpath(sys.argv[0])
    scriptPath = os.path.split( script )[0]

	# Stop if no input image dir specified
    if not imageDir:
        sys.exit("No image dir provided.")

	# Determine complete path of images and make sure it exists
    imagesCompletePath = os.path.join( scriptPath, imageDir[0] )
    if not os.path.isdir(imagesCompletePath):
        sys.exit("Dir " + imagesCompletePath + " does not exist.")

	# Get the list of image files
    extensions = ("*.pgn","*.jpg","*.jpeg","*.JPG",)
    img_names = []
    for ext in extensions:
        img_names.extend( glob( os.path.join( imagesCompletePath, ext ) ) )

    debug_dir = args.get('--debug')
    square_size = float(args.get('--square_size', 1.0))

    # Nombre de carres sur la grille ici
    pattern_size = (8, 6)
    pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
    pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
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
        if debug_dir:
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            path, name, ext = splitfn(fn)
            cv2.imwrite('%s/%s_chess.bmp' % (debug_dir, name), vis)
        if not found:
            print 'chessboard not found'
            continue
        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)

        print 'ok'

    # do calibration
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)

    # Write result to a file 
    cameraMatFileName = os.path.join( imagesCompletePath, 'camera-matrix' )
    np.save( cameraMatFileName, camera_matrix )
    distCoefsFileName = os.path.join( imagesCompletePath, 'dist-coefs' )
    np.save( distCoefsFileName, dist_coefs )

    # Compute focal distance in mm
    ccdWidth = 22.2
    xFocalPix = camera_matrix[0,0]
    xFocalmm = ccdWidth * xFocalPix / w
    ccdHeight = 14.8
    yFocalPix = camera_matrix[1,1]
    yFocalmm = ccdHeight * yFocalPix / h
    print( "Focal (mm): ( {0}, {1} )".format(xFocalmm, yFocalmm) )
    print "Reprojection error:", rms, " pixels"

    resultsFileName = os.path.join( imagesCompletePath, 'results.txt' )
    with open( resultsFileName, 'w' ) as f:
        f.write( 'Reprojection error: {}\n'.format( rms ) )
        f.write( 'Focal dist(mm): ( {0}, {1} )\n'.format(xFocalmm, yFocalmm ) )

    cv2.destroyAllWindows()
