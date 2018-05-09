/**
 * stereoCalibration.hpp
 * This file is created as part of the Stereoendoscopy project
 *
 *
 * The stereoCalibration module takes as input a set of left and right chessboard images
 * and uses OpenCV routines to compute the following parameters:
 *  > Camera matrix, Distortion Coefficients, Rotation & Translation vector -- of left and right camera
 *  > Rotation and Translation matrix between left & right cameras
 *  > Essential and Fundamental matrix of stereo-camera
 *
 *
 * This is a code snippet for representative purposes.
 * @author Lalith Nag
 * @version 1.1 09/04/2018
 *
 */

#ifndef stereoCalibration_hpp
#define stereoCalibration_hpp

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/ccalib.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;


void stereoCalibration(); /// Interacts with the user and calls all appropriate functions in this module


/**
 * The 'Chessboard' class contains the image and related parameters like height, width, board size.
 */

class Chessboard {

protected:
    int height, width;
    Size board_size;

public:
    vector<Point3f> obj; // Contains all chessboard coordinates
    string path;
    inline Size boardSize() { Size size=Size(width, height); return size; }

    void initializeChessboard(); /// Instantiates Chessboard params
    void pushBackObjectPoints(); /// Store known chessboard corners if corners are found


} static chessboard;



/// Forward declaration so that they can be declared as friend functions to class 'Camera'
class StereoCamera;
class Rectify;



/**
 * The 'Camera' class contains the intrinsic and extrinsic camera params
 * and also the error associated with calibrating the camera
 * Classes 'StereoCamera' and 'Rectify' are declared as friends, so they can
 * access L & R camera params for stereo-calibration and stereo-rectification
 */

class Camera {

protected:

///Image params
    Mat image;
    Size image_size;
    int number_frames;
    string image_path, video_path;

    string getVideoPath();
    void getImage() {image = imread (image_path);}

/// Camera params
    vector<vector<Point2f>> imagepoints; // Image coordinates of chessboard corners
    vector<Point2f> corners;
    Mat camera_matrix, distortion_coeffs;
    vector<Mat> rotation_vec,translation_vec;
    double average_error;

public:
    friend class StereoCamera;

    bool found_corners;
    void extractCorners(const int&, const string&, const int&); /// Computes corner locations and stores them if found
    void pushBackImagePoints();
    void calibrate(); /// Computes intrinsic and extrinsic params of a camera
    double computeReprojectionError(); /// Computes total average error by reprojection image points
} ;

extern Camera left_camera, right_camera;

void extractValidCorners(); /// Interacts with user and calls extractCorners based on user-defined params



/**
 * The 'StereoCamera' class contains the stereo-camera params
 * such as R & T matrix between cameras, and Essential & Fundamental matrix
 * Class 'Rectify' and rectifyStereoImage(0 are declared as friends so they can
 * access stereo params for rectification
 */

class StereoCamera {

protected:
    Mat rotation_matrix, translation_matrix;
    Mat essential_matrix;
    Mat fundamental_matrix;
    double rms_error;

public:
    friend class Rectify;
    friend void rectifyStereoImage();

    void calibrateStereoCamera(); /// Uses L & R camera params to compute stereo-camera params

} ;

extern StereoCamera stereo_camera;


#endif
