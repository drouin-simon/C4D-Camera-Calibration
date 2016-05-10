"""
Camera Pose
Copyright: Simon Drouin
Written for Cinema 4D R17.048

Modified Date: 28/02/2016
"""

import sys

# paths for opencv and numpy
sys.path.append('/System/Library/Frameworks/Python.framework/Versions/2.7/Extras/lib/python')
sys.path.append('/Users/simon/Ibis/superbuild-release/ibisExternalDependencies/opencv-3.0.0/install/lib/python2.7/site-packages')

import c4d
import os
import numpy as np
import cv2
import math

from c4d import gui, plugins, bitmaps, documents
import glob

#be sure to use a unique ID obtained from www.plugincafe.com
PLUGIN_ID = 1036914

#for GeLoadString
#values must match with the header file
IDS_PRIMITIVETOOL = 50000

# Variables a configurer
BaseImageDirectory = '/Users/simon/Dropbox/calibration/'
ccdWidth = 22.2

def NumpyMat2C4dMat( npMat ):
    t = c4d.Vector( npMat[0,3], npMat[1,3], npMat[2,3] )
    vx = c4d.Vector( npMat[0,0], npMat[1,0], npMat[2,0] )
    vy = c4d.Vector( npMat[0,1], npMat[1,1], npMat[2,1] )
    vz = c4d.Vector( npMat[0,2], npMat[1,2], npMat[2,2] )
    outMat = c4d.Matrix( t, vx, vy, vz )
    return outMat

class SettingsDialog(gui.SubDialog):
    widthSpin = 0
    heightSpin = 0
    squareSizeSpin = 0
    calibDirCombo = 0
    calibDirs = []
    imageDirCombo = 0
    imagesDirs = []

    def __init__(self, arg):
        self.data = arg

    def GatherCalibDirs(self):
        dirs = []
        calibDir = os.path.join( BaseImageDirectory, "calibration", '*' )
        for f in glob.glob( calibDir ):
            if os.path.isdir( f ):
                camFile = os.path.join( f, 'camera-matrix.npy' )
                if os.path.exists( camFile ):
                    dirs.append( os.path.basename( f ) )
        return dirs

    def GatherImagesDirs(self):
        dirs = []
        calibDir = os.path.join( BaseImageDirectory, "images", '*' )
        for f in glob.glob( calibDir ):
            if os.path.isdir( f ):
                dirs.append( os.path.basename(f) )
        return dirs

    def FillCombo( self, combo, names ):
        self.FreeChildren( combo )
        for i, n in enumerate(names):
            self.AddChild(id=combo, subid=i, child=n)

    def UpdateCombos(self):
        self.calibDirs = self.GatherCalibDirs()
        self.FillCombo( self.calibDirCombo, self.calibDirs )
        self.imagesDirs = self.GatherImagesDirs()
        self.FillCombo( self.imageDirCombo, self.imagesDirs )

    def CreateLayout(self):
        self.GroupBegin(id=1000, flags=c4d.BFH_SCALEFIT, cols=2, rows=6)
        self.GroupBorderSpace(10, 10, 10, 10)
        
        # Grid Width
        self.AddStaticText(id=1001, flags=c4d.BFH_MASK, initw=140, name="Grid Width", borderstyle=c4d.BORDER_NONE)
        self.widthSpin = self.AddEditNumberArrows(id=1002, flags=c4d.BFH_MASK)
        self.SetInt32(id=self.widthSpin, value=self.data['gridWidth'], min=1, max=20 )
        
        # Grid Height
        self.AddStaticText(id=1004, flags=c4d.BFH_MASK, initw=140, name="Grid Height", borderstyle=c4d.BORDER_NONE)
        self.heightSpin = self.AddEditNumberArrows(id=1005, flags=c4d.BFH_MASK)
        self.SetInt32(id=self.heightSpin, value=self.data['gridHeight'], min=1, max=20)

        # Square size
        self.AddStaticText(id=1006, flags=c4d.BFH_MASK, initw=140, name="Square Size (mm)", borderstyle=c4d.BORDER_NONE)
        self.squareSizeSpin = self.AddEditNumberArrows(id=1007, flags=c4d.BFH_MASK)
        self.SetInt32(id=self.squareSizeSpin, value=self.data['squareSize'], min=1, max=100)

        # Calibration Combo
        self.AddStaticText(id=1008, flags=c4d.BFH_MASK, initw=140, name="Calibration", borderstyle=c4d.BORDER_NONE)
        self.calibDirCombo = self.AddComboBox(id=1009, flags=c4d.BFH_MASK, initw=200 )

        # Image Dir Combo
        self.AddStaticText(id=1010, flags=c4d.BFH_MASK, initw=140, name="Images Dir", borderstyle=c4d.BORDER_NONE)
        self.imageDirCombo = self.AddComboBox(id=1011, flags=c4d.BFH_MASK, initw=200)

        self.UpdateCombos()

        # Generate planes button
        self.AddButton(id=1012, flags=c4d.BFH_MASK, name="Generate Planes")
        self.AddButton(id=1013, flags=c4d.BFH_MASK, name="Refresh")
        
        self.GroupEnd()

        return True

    def ComputePlanes( self, calDir, imDir ):
        # compute pattern points
        pattern_size = (self.data['gridWidth'],self.data['gridHeight'])
        square_size = self.data['squareSize']
        pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
        pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
        pattern_points *= square_size
        
        # Load calibration data
        calibCompletePath = os.path.join( BaseImageDirectory, 'calibration', calDir )
        if not os.path.isdir(calibCompletePath):
            gui.MessageDialog("Calibration directory doesn't exist")
            return
        cameraMatrixFilename = os.path.join( calibCompletePath, "camera-matrix.npy")
        try:
            cameraMatrix = np.load( cameraMatrixFilename )
        except:
            gui.MessageDialog("Cannot read camera matrix.")
            return
        distortionFilename = os.path.join( calibCompletePath, "dist-coefs.npy")
        try:
            distCoefs = np.load( distortionFilename )
        except:
            gui.MessageDialog("Cannot read camera matrix.")
            return

        # Get the list of image files
        imagesCompletePath = os.path.join( BaseImageDirectory, 'images', imDir )
        if not os.path.isdir(imagesCompletePath):
            gui.MessageDialog("Image directory " + imagesCompletePath + " does not exist.")
            return
        extensions = ("*.pgn","*.jpg","*.jpeg","*.JPG",)
        img_names = []
        for ext in extensions:
            img_names.extend( glob.glob( os.path.join( imagesCompletePath, ext ) ) )

        # Get selected object and make sure it is a camera
        doc = c4d.documents.GetActiveDocument()
        select = doc.GetActiveObjects(0)
        if len(select) <= 0:
            gui.MessageDialog('No object selected! Must select a camera')
            return
        cam = select[0]
        if cam.GetTypeName() != 'Camera':
            gui.MessageDialog('Selected object is not a camera')
            return

        # Process all images
        h, w = 0, 0
        for fn in img_names:
            img = cv2.imread(fn, 0)
            if img is None:
                gui.MessageDialog( "Couldn't load " + fn )
                continue

            h, w = img.shape[:2]
            found, corners = cv2.findChessboardCorners(img, pattern_size)
            if found:
                term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
                cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
            if not found:
                gui.MessageDialog("chessboard not found on " + fn )
                continue

            imgPoints = corners.reshape(-1,2)
            retVal, rvec, tvec = cv2.solvePnP( pattern_points, imgPoints, cameraMatrix, distCoefs )

            #ts = 'translation: ( {0}, {1}, {2} )'.format( tvec[0,0], tvec[1,0], tvec[2,0] )
            #rs = 'rotation: ( {0}, {1}, {2} )\n'.format( rvec[0,0], rvec[1,0], rvec[2,0] )
            #gui.MessageDialog( ts + "\n" + rs )
            #gui.MessageDialog( "rvec: " + str(type(rvec[0,0])) + "tvec: " + str(type(tvec[0,0])) )

            # Generate the plane
            plane = c4d.BaseObject(c4d.Opolygon) #Create an empty polygon object
            plane.SetName( os.path.basename( fn ) )
            plane.ResizeObject(4,1) #New number of points, New number of polygons
            squareSizeCm = square_size * 0.1
            plane.SetPoint( 0, c4d.Vector( -squareSizeCm, 0, pattern_size[1] * squareSizeCm ) )
            plane.SetPoint( 1, c4d.Vector( squareSizeCm * pattern_size[0], 0, squareSizeCm * pattern_size[1] ) )
            plane.SetPoint( 2, c4d.Vector( squareSizeCm * pattern_size[0], 0, -squareSizeCm ) )
            plane.SetPoint( 3, c4d.Vector( -squareSizeCm, 0, -squareSizeCm ) )
            plane.SetPolygon( 0, c4d.CPolygon( 0, 1, 2, 3 ) ) #The Polygon's index, Polygon's points
            doc.InsertObject( plane, None, None )

            # Compute Plane matrix
            rotMat = cv2.Rodrigues( rvec )[0]  # transform to rot matrix
            rotMat4 = np.eye(4,4)
            rotMat4[:3,:3] = rotMat

            transVec = np.array([2.0,3.0,4.0])
            transMat = np.eye(4,4)
            transMat[:3,3:4] = 0.1 * tvec

            # transform with opencv convention
            tCv = np.dot( transMat, rotMat4 )

            # transform c4d space -> cv space 
            c4d2cv = np.eye(4,4)
            c4d2cv[1,1] = 0.0
            c4d2cv[1,2] = 1.0
            c4d2cv[2,1] = 1.0
            c4d2cv[2,2] = 0.0

            # transform cv cam -> c4d cam (flip y)
            cvCam2c4dCam = np.eye(4,4)
            cvCam2c4dCam[1,1] = -1.0

            # combine matrices
            mat = np.dot( cvCam2c4dCam, tCv )
            mat = np.dot( mat, c4d2cv )

            # Set matrix
            c4dMat = NumpyMat2C4dMat( mat )
            plane.SetMl( c4dMat )

            #plane.SetRelPos( c4d.Vector( tvec[0,0] * 0.1, tvec[1,0] * 0.1, tvec[2,0] * 0.1 ) )   # Set position of cube
            #plane.SetRelRot( c4d.Vector( rvec[0,0], rvec[1,0] , rvec[2,0] ) )  # rotation en radians
            plane.InsertUnderLast(cam)

            plane.Message( c4d.MSG_UPDATE )
            c4d.EventAdd()

        # Set cam params
        xCenter = cameraMatrix[0,2]
        yCenter = cameraMatrix[1,2]
        xOffset = ( 0.5 * w - xCenter ) / w
        yOffset = ( 0.5 * h - yCenter ) / h
        xFocalPix = cameraMatrix[0,0]
        yFocalPix = cameraMatrix[1,1]
        fov = 2 * math.atan( w * 0.5 / xFocalPix )
        cam.SetAperture( ccdWidth )
        cam[c4d.CAMERAOBJECT_FOV] = fov
        cam[c4d.CAMERAOBJECT_FILM_OFFSET_X] = xOffset
        cam[c4d.CAMERAOBJECT_FILM_OFFSET_Y] = yOffset
        message = "Focal =  ( " + str(xFocalPix) + ", " + str(yFocalPix) + " ) "
        message += "- center = ( " + str(xCenter) + ", " + str(yCenter) + " ) "
        message += "- fov = " + str(fov)
        #gui.MessageDialog( message )

    def Command(self, id, msg):
        if id==1002:
            self.data['gridWidth'] = self.GetInt32(self.widthSpin)
        if id==1005:
            self.data['gridHeight'] = self.GetInt32(self.heightSpin)
        if id==1007:
            self.data['squareSize'] = self.GetInt32(self.squareSizeSpin)
        if id==1012:
            i = self.GetInt32( self.calibDirCombo )
            calDir = self.calibDirs[i]
            i = self.GetInt32( self.imageDirCombo )
            imDir = self.imagesDirs[i]
            self.ComputePlanes(calDir,imDir)
        if id==1013:
            self.UpdateCombos()
        return True


class CameraPoseTool(plugins.ToolData):
    """Inherit from ToolData to create your own tool"""

    def __init__(self):
        self.data = dict(gridWidth=6,gridHeight=8,squareSize=30,inputDir="None")

    def AllocSubDialog(self, bc):
        return SettingsDialog(self.data) #always return new instance


if __name__ == "__main__":
    bmp = bitmaps.BaseBitmap()
    dir, file = os.path.split(__file__)
    fn = os.path.join(dir, "res", "liquid.tif")
    bmp.InitWith(fn)
    plugins.RegisterToolPlugin(id=PLUGIN_ID, str="Camera Pose",
                                info=0, icon=bmp, 
                                help="Compute camera pose from grid image",
                                dat=CameraPoseTool())
