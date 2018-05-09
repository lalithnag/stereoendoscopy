/**
 * stereoCalibration.cpp
 * This file is created as part of the Stereoendoscopy project
 *
 *
 * This is a code snippet for representative purposes.
 * @author Lalith Nag
 * @version 1.1 09/04/2018
 *
 */

#include "stereoCalibration.hpp"

vector<vector<Point3f>> objectpoints; // Known coordinates of all valid chessboards


Camera left_camera, right_camera;
StereoCamera stereo_camera;


void stereoCalibration() {

    cout<<"-------------------- Sub-module: Stereo-calibration ------------------------- \n";
    cout<<"\nStereo-calibration takes in a stereo-image set of chessboards in different orientations, \n";
    cout<<"and computes the intrinsic & extrinsic parameters of the stereo-camera \n";
    cout<<"These parameters are then used to rectify the images. \n";

    chessboard.initializeChessboard();

    extractValidCorners();

    left_camera.calibrate();
    right_camera.calibrate();

    double lefterror, righterror;
    lefterror = left_camera.computeReprojectionError();
    righterror = right_camera.computeReprojectionError();

    cout<<"\nTotal average error in Left Camera is: "<<lefterror<<endl;
    cout<<"Total average error in Right Camera is: "<<righterror<<endl;

    stereo_camera.calibrateStereoCamera();

    cout<<"\nExiting sub-module..."<<endl;

}



void Chessboard::initializeChessboard() {

    cout<<"\nEnter path to directory where Chessboard images are located. \n";
    cout<<"\nNote! : The images inside the directory must be named as follows: \n";
    cout<<" '.../leftframe1, .../rightframe1, .../leftframe2, .../rightframe2 and so on. \n";
    cout<<"\nEnter path: ";
    cin.ignore(); getline(cin, path);
    cout<<endl;

    cout<<"Enter Board Height (No. of squares): ";
    cin>>height;
    cout<<endl;
    height--;

    cout<<"Enter Board Width (No. of squares): ";
    cin>>width;
    cout<<endl;
    width--;

    board_size = Size(width, height);

    for (int i=0; i<height; i++) {
        for(int j=0; j<width; j++) {
            obj.push_back(Point3f( j, i , 0));
        }
    }

}



void extractValidCorners() {

    int number_of_boards;
    cout<<"Enter number of boards to be examined: ";
    cin>>number_of_boards;
    cout<<endl;

    int sub_pixel_window;
    cout<<"Enter size of window for Sub-Pixel Optimizer: ";
    cin>>sub_pixel_window;
    cout<<endl;

    cout<<"Examining valid chessboards and extracting their corners..\n"<<endl;
    int valid_boards =0;

    for (int board_number=1; board_number<=number_of_boards; board_number++) {

        cout<<"Checking Chessboard "<<board_number;

        left_camera.extractCorners(board_number, "/left", sub_pixel_window);
        right_camera.extractCorners(board_number, "/right", sub_pixel_window);

        if (left_camera.found_corners && right_camera.found_corners) {

            left_camera.pushBackImagePoints();
            right_camera.pushBackImagePoints();
            chessboard.pushBackObjectPoints();
            cout<<" ----- Success";
            valid_boards++;

        }

        cout<<endl;

    }

    cout<<"\nCompleted examining all Chessboards. \n";
    cout<<"No. of valid chessboards found: "<<valid_boards<<endl;

}



void Camera::extractCorners(const int& board_number, const string& orientation, const int& sub_pixel_window = 12) {

    image = imread(chessboard.path + orientation + "frame" +to_string(board_number)+ ".bmp", IMREAD_GRAYSCALE);

    found_corners=0;
    found_corners = findChessboardCorners(image, chessboard.boardSize(), corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);

    if (found_corners)
        cornerSubPix(image, corners, Size(sub_pixel_window,sub_pixel_window), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

}


inline
void Camera::pushBackImagePoints() { imagepoints.push_back(corners); }


inline
void Chessboard::pushBackObjectPoints() { objectpoints.push_back(obj); }



void Camera::calibrate() {

    calibrateCamera(objectpoints, imagepoints, image.size(), camera_matrix, distortion_coeffs, rotation_vec, translation_vec);

}



double Camera::computeReprojectionError() {

    vector<Point2f> reprojected_imagepoints;
    int total_points=0;
    double total_error =0,error;
    vector<float> per_view_errors;

    per_view_errors.resize(objectpoints.size());

    for( int point=0; point<(int)objectpoints.size(); point++)
    {
        projectPoints(Mat(objectpoints[point]), rotation_vec[point], translation_vec[point], camera_matrix, distortion_coeffs, reprojected_imagepoints);
        error = norm(Mat(imagepoints[point]),Mat(reprojected_imagepoints),NORM_L2);
        int num_points = (int)objectpoints[point].size();
        per_view_errors[point] = (float)sqrt(error * error /num_points);
        total_error += error * error;
        total_points += num_points;

    }

    average_error = total_error/total_points;

    return average_error;
}



void StereoCamera::calibrateStereoCamera() {

    rms_error = stereoCalibrate(objectpoints, left_camera.imagepoints, right_camera.imagepoints, left_camera.camera_matrix, left_camera.distortion_coeffs, right_camera.camera_matrix, right_camera.distortion_coeffs, left_camera.image.size(), rotation_matrix, translation_matrix, essential_matrix, fundamental_matrix, CV_CALIB_SAME_FOCAL_LENGTH);

    cout<<"\nThe Stereo-Camera is successfully calibrated with a total RMS error of "<<rms_error<<endl;

}
