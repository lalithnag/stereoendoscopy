
# Stereoendoscopy

This repo gives an overview of depth-mapping for stereo-endoscopy applications along with a short code snippet of stereo-calibration. This project was implemented in C++ with OpenCV Libraries.

## The Stereo-endoscope

Conventional endoscopy is a non-surgical technique used to look inside the human body. A recent development is the stereo endoscope which contains a stereo-camera with a known baseline.
The advantage of using a stereo-endoscope is that the images can be rendered in stereo-3D, in which case the surgeon views the monitor with 3D polaroid glasses. Another advantage is that the data from the stereo-camera can be used for reconstructing the depth of the images - in near real-time. This is of a huge advantage to the surgeons, as it helps them to understand otherwise unseen geometries and hence use this information to make better surgical decisions.

## Stereo-reconstruction

Without going into specific details about the clinical application of my project, I will rather talk about the stereo-reconstruction process, which can be applied in a wide variety of applications.

Stereo-reconstruction contains the following pipeline:

![Stereo pipieline](https://github.com/lalithnag/stereoendoscopy/blob/master/stereopipeline.png)

### Pre-processing

Once you've acquired the data as a stereo-pair, you have to split the stereo-images into left and right images. For this, you need to take care if the images are overlaid or just adjacent to each other.

### Stereo-calibration

For each of the camera images, we have to perform calibration and acquire the camera parameters. This is because the camera is not perfect and causes distortion effects - radial and tangential distortion. You can think of calibration as the process of understanding how exactly this camera distorts an image, so that we can again *undistort* it. This is the process of finding the intrinsic and extrinsic matrix of each camera.

OpenCV has libraries that perform these functions. I have attached a code snippet from the StereoCalibration module, from where you will get an idea of the whole process.

<p>First you need to find corners of the chessboard : </p>
<pre><code>
void Camera::extractCorners(const int& board_number, const string& orientation, const int& sub_pixel_window = 12) {

    image = imread(chessboard.path + orientation + "frame" +to_string(board_number)+ ".bmp", IMREAD_GRAYSCALE);

    found_corners=0;
    found_corners = findChessboardCorners(image, chessboard.boardSize(), corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);

    if (found_corners)
        cornerSubPix(image, corners, Size(sub_pixel_window,sub_pixel_window), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

}
</code></pre>

<p>Next, you need to calibrate each camera to find out the respective intrinsic and extrinsic params : </p>
<pre><code>
void Camera::calibrate() {

    calibrateCamera(objectpoints, imagepoints, image.size(), camera_matrix, distortion_coeffs, rotation_vec, translation_vec);

}
</code></pre>

Now that we know the params of each camera, we also need to compute how each camera is placed relative to each other, i.e by how much it is translated and rotated with respect to the other camera. This is given by the elementary and fundamental matrix.

<p>The E and F matrix can be computed using the OpenCV function stereoCalibrate : </p>
<pre><code>
void StereoCamera::calibrateStereoCamera() {

    rms_error = stereoCalibrate(objectpoints, left_camera.imagepoints, right_camera.imagepoints, left_camera.camera_matrix, left_camera.distortion_coeffs, right_camera.camera_matrix, right_camera.distortion_coeffs, left_camera.image.size(), rotation_matrix, translation_matrix, essential_matrix, fundamental_matrix, CV_CALIB_SAME_FOCAL_LENGTH);

    cout<<"\nThe Stereo-Camera is successfully calibrated with a total RMS error of "<<rms_error<<endl;

}
</code></pre>

### Stereo-rectification

Once you have obtained all the calibration parameters of the camera, we now have to use these params to undistort and rectify the image. The output of this process is a pair of row-rectified images - this means the rows of the left and right image pair coincide with each other. This makes it super easy to compare the corresponding pixels and determine the pixel shift and in turn compute the disparity.

![Stereo rectification](https://github.com/lalithnag/stereoendoscopy/blob/master/stereorectification.png)

### Disparity

Once you row-rectify the images using acquired parameters, you have to compute disparity. There are many algorithms to compute disparity, but the popular (and easy) ones are Stereo-Block Matching (SBM) and Stereo-Global Block Matching (SGBM).

<p>A glimpse on how to implement disparity using SGBM</p>
<pre><code>
Ptr<StereoSGBM> sgbm = StereoSGBM::create(min_disparity, num_disparities, sad_window_size);

    sgbm->setDisp12MaxDiff(pixel_difference);
    sgbm->setSpeckleRange(speckle_range);
    sgbm->setSpeckleWindowSize(speckle_window);
    sgbm->setUniquenessRatio(uniqueness_ratio);
    sgbm->setPreFilterCap(pre_filter_cap);

    Mat left_image = imread(image_path+"/leftframe"+to_string(frame)+".bmp", IMREAD_GRAYSCALE);
    Mat right_image = imread(image_path+"/rightframe"+to_string(frame)+".bmp", IMREAD_GRAYSCALE);

    sgbm->compute(left_image, right_image, map16Sg);

    normalize(map16Sg, map8Ug, 0, 255, CV_MINMAX, CV_8U);
</code></pre>

### 3D Visualisation

Huzzah! If you made it till here, you've just got started :stuck_out_tongue:. So we've managed to get a disparity map, but most probably it is riddled with illumination blobs, speckle noise and false disparities that look super weird when seen in 3D. Needless to say, this is a non-trivial problem and requires application of image processing techniques. I will not get into those within the scope of this project.


## Built With

* [OpenCV](https://opencv.org/) - Computer Vision Library
* [C++](http://www.cplusplus.com/) - Fast, object-oriented code
* [XCODE](https://developer.apple.com/xcode/) - IDE supported by MacOS

## Authors

* [Lalith Nag](https://github.com/lalithnag). Drop me a line if you want to know more or you're stuck at some place! :smiley: I will put up more elaborate blog posts soon. Until then, you can email me at lalith.sharan@ovgu.de

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* A **huge** thanks to Dr. Sandy Engelhardt :bow:, my project guide and mentor for introducing me to Computer Vision and helping me through this project.
* This project was undertaken as part of the course _Computer Assisted Surgeries_ of the *Medical Systems Engineering* course at the *Otto-von-Guericke University*, Magdeburg.
