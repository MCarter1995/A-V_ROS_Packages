#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <glob.h>
#include <string.h>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

std::vector<std::string> glob(const std::string& pattern) {

	// glob struct resides on the stack
	glob_t glob_result;
	memset(&glob_result, 0, sizeof(glob_result));

	// do the glob operation
	int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
	if (return_value != 0) {
		globfree(&glob_result);
		stringstream ss;
		ss << "glob() failed with return_value " << return_value << endl;
		throw std::runtime_error(ss.str());
	}

	// collect all the filenames into a std::list<std::string>
	vector<string> filenames;
	for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
		filenames.push_back(string(glob_result.gl_pathv[i]));
	}

	// cleanup
	globfree(&glob_result);

	// done
	return filenames;
}

std::string tail(std::string const& source, size_t const length) {
	if (length >= source.size()) { return source; }
	return source.substr(source.size() - length);
}

int main( int argc, char** argv ) {
	cout << "OpenCV version : " << CV_VERSION << endl;

	bool run_calibration = false;
	string intrinsic_filename = "Data/intrinsics.xml";
	string extrinsic_filename = "Data/extrinsics.xml";

	if (run_calibration) {
		vector<string> l_imglist = glob("calib_data/left-*.jpg");
		vector<string> r_imglist = glob("calib_data/right-*.jpg");
		vector<string> im_numbers;
		vector<string> goodImgList;

		float squareSize = 1.0f;
		Size boardSize;
		boardSize.width = 8;
		boardSize.height = 6;

		int num_img = l_imglist.size();

		for (int i = 0; i < num_img; i++) {
			im_numbers.push_back(tail(l_imglist[i], 10));
		}

		string r_filepath = "calib_data/right-";
		string l_filepath = "calib_data/left-";

		const int maxScale = 2;
		// ARRAY AND VECTOR STORAGE:

		vector<vector<Point2f> > imagePoints[2];
		vector<vector<Point3f> > objectPoints;
		Size imageSize;

		int i, j, k, nimages = num_img;

		imagePoints[0].resize(nimages);
		imagePoints[1].resize(nimages);
		vector<string> goodImageList;

		for ( i = j = 0; i < nimages; i++ ){
			for ( k = 0; k < 2; k++ )
			{
				Mat img ;
				if (k == 0) {
					img = imread(r_filepath + im_numbers[i], IMREAD_GRAYSCALE );
				} else if (k == 1) {
					img = imread(l_filepath + im_numbers[i], IMREAD_GRAYSCALE );

				}
				//cv::flip(img,img,1); //PFC flip cal images 20.03.19 if required
				if (img.empty())
					break;
				imageSize = img.size();

				bool found = false;
				vector<Point2f>& corners = imagePoints[k][j];
				for ( int scale = 1; scale <= maxScale; scale++ )
				{
					Mat timg;
					if ( scale == 1 )
						timg = img;
					else
						resize(img, timg, Size(), scale, scale);
					found = findChessboardCorners(timg, boardSize, corners,
					                              CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
					if ( found )
					{
						if ( scale > 1 )
						{
							Mat cornersMat(corners);
							cornersMat *= 1. / scale;
						}
						break;
					}
				}
				if (true) //PFC was (displayCorners )
				{
					if (k == 0) {
						cout << r_filepath << im_numbers[i] << endl;
					} else if (k == 1) {
						cout << l_filepath + im_numbers[i] << endl;
					}
				}
				else
					putchar('.');
				if ( !found )
					break;
				cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
				             TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
				                          30, 0.01));
			}
			if ( k == 2 ){
				goodImageList.push_back(im_numbers[i]);
				j++;
			}
		}
		cout << j << " pairs have been successfully detected.\n";
		nimages = j;


		imagePoints[0].resize(nimages);
		imagePoints[1].resize(nimages);
		objectPoints.resize(nimages);

		for ( i = 0; i < nimages; i++ ){
			for ( j = 0; j < boardSize.height; j++ ){
				for ( k = 0; k < boardSize.width; k++ ){
					objectPoints[i].push_back(Point3f(k * squareSize, j * squareSize, 0));
				}
			}
		}

		cout << "Running stereo calibration ...\n";

		Mat cameraMatrix[2], distCoeffs[2];
		Mat rvecs[2], tvecs[2];
		Mat _rvecs[2], _tvecs[2];
		fisheye::calibrate(objectPoints, imagePoints[0], imageSize,
		                   cameraMatrix[0], distCoeffs[0],
		                   _rvecs[0], _tvecs[0],
		                   fisheye::CALIB_FIX_SKEW + fisheye::CALIB_RECOMPUTE_EXTRINSIC,
		                   TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));
		fisheye::calibrate(objectPoints, imagePoints[1], imageSize,
		                   cameraMatrix[1], distCoeffs[1],
		                   _rvecs[1], _tvecs[1],
		                   fisheye::CALIB_FIX_SKEW + fisheye::CALIB_RECOMPUTE_EXTRINSIC,
		                   TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));

		for (k = 0; k < 2; k++) {
			rvecs[k].reserve(_rvecs[k].rows);
			tvecs[k].reserve(_tvecs[k].rows);
			for (i = 0; i < int(objectPoints.size()); i++) {
				rvecs[k].push_back(_rvecs[k].row(i));
				tvecs[k].push_back(_tvecs[k].row(i));
			}
		}
		Mat R, T, E, F;

		double rms = fisheye::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		                                      cameraMatrix[0], distCoeffs[0],
		                                      cameraMatrix[1], distCoeffs[1],
		                                      imageSize, R, T,
		                                      fisheye::CALIB_USE_INTRINSIC_GUESS +
		                                      fisheye::CALIB_FIX_K3 + fisheye::CALIB_FIX_K4,
		                                      TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));

		cout << "done with RMS error=" << rms << endl;

		Mat R1, R2, P1, P2, Q;
		Rect validRoi[2];
		float balance = 0.0f;
		float fov_scale = 1.0f;
		fisheye::stereoRectify(cameraMatrix[0], distCoeffs[0],
		                       cameraMatrix[1], distCoeffs[1],
		                       imageSize, R, T, R1, R2, P1, P2, Q,
		                       CALIB_ZERO_DISPARITY, imageSize, balance, fov_scale);

		// save intrinsic parameters
		//PFC Saves to local Repo folder
		FileStorage fs("Data/intrinsics.xml", FileStorage::WRITE);
		if ( fs.isOpened() )
		{
			fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			   "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
			fs.release();
		}
		else
			cout << "Error: can not save the intrinsic parameters\n";

		//PFC Saves to local Repo folder
		fs.open("Data/extrinsics.xml", FileStorage::WRITE);
		if ( fs.isOpened() )
		{
			fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
			fs.release();
		}
		else
			cout << "Error: can not save the extrinsic parameters\n";

		// OpenCV can handle left-right
		// or up-down camera arrangements
		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
	}else if ( !intrinsic_filename.empty() ) {
		// reading intrinsic parameters
		FileStorage fs(intrinsic_filename, FileStorage::READ);
		if (!fs.isOpened()) {
			printf("Failed to open file %s\n", intrinsic_filename.c_str());
			return -1;
		}

		Mat M1, D1, M2, D2;
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;


		fs.open(extrinsic_filename, FileStorage::READ);
		if (!fs.isOpened()) {
			printf("Failed to open file %s\n", extrinsic_filename.c_str());
			return -1;
		}
		Mat R, T, R1, P1, R2, P2;

		fs["R"] >> R;
		fs["T"] >> T;

		Mat rmap[2][2];
		int cameraWidth = 1920;
		int cameraHeight = 1080;

		Size imageSize;
		imageSize.width = cameraWidth;
		imageSize.height = cameraHeight;

		//Precompute maps for cv::remap()
		fisheye::initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		fisheye::initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

		bool display = true;

		if (display) {
			enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
			int alg = STEREO_SGBM; //PFC always do SGBM - colour
			int SADWindowSize, numberOfDisparities;
			bool no_display;
			float scale;

			Ptr<StereoBM> bm = StereoBM::create(16, 9);
			Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

			numberOfDisparities = 96;
			SADWindowSize = 5;
			scale = 1;
			int color_mode = alg == STEREO_BM ? 0 : -1;
			numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((imageSize.width / 8) + 15) & -16;

			VideoCapture right(2);
			VideoCapture left(1);



			right.set(CV_CAP_PROP_FRAME_WIDTH, cameraWidth);
			right.set(CV_CAP_PROP_FRAME_HEIGHT, cameraHeight);

			left.set(CV_CAP_PROP_FRAME_WIDTH, cameraWidth);
			left.set(CV_CAP_PROP_FRAME_HEIGHT, cameraHeight);

			right.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
			left.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));

			Mat r_img, r_cimg, r_rimg;
			Mat l_img, l_cimg, l_rimg;
			Mat disp, disp8;
			Mat view;

			namedWindow("View", WINDOW_NORMAL);
			resizeWindow("View", 1920, 1080 / 2);

			namedWindow("disparity", 0);
			resizeWindow("View", 1920 / 2, 1080 / 2);

			bool p_type = true;

			/*bm->setROI1(validRoi[0]);
			bm->setROI2(validRoi[1]);*/
			bm->setPreFilterCap(31);
			bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
			bm->setMinDisparity(0);
			bm->setNumDisparities(numberOfDisparities);
			bm->setTextureThreshold(10);
			bm->setUniquenessRatio(15);
			bm->setSpeckleWindowSize(100);
			bm->setSpeckleRange(32);
			bm->setDisp12MaxDiff(1);

			sgbm->setPreFilterCap(63);
			int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
			sgbm->setBlockSize(sgbmWinSize);

			int cn = l_img.channels();

			sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
			sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
			sgbm->setMinDisparity(0);
			sgbm->setNumDisparities(numberOfDisparities);
			sgbm->setUniquenessRatio(10);
			sgbm->setSpeckleWindowSize(100);
			sgbm->setSpeckleRange(32);
			sgbm->setDisp12MaxDiff(1);
			sgbm->setMode(StereoSGBM::MODE_SGBM);

			for (;;)
			{
				right.grab();
				left.grab();

				right.retrieve(r_img);
				left.retrieve(l_img);

				cvtColor(r_img, r_img, COLOR_RGB2GRAY);
				cvtColor(l_img, l_img, COLOR_RGB2GRAY);

				//r_cimg = r_img(validRoi[0]);
				//l_cimg = l_img(validRoi[1]);

				remap(r_img, r_rimg, rmap[0][0], rmap[0][1], INTER_LINEAR);
				remap(l_img, l_rimg, rmap[1][0], rmap[1][1], INTER_LINEAR);

				bm->compute(l_rimg, r_rimg, disp);
				disp.convertTo(disp8, CV_8U);

				hconcat(r_rimg, l_rimg, view);
				imshow("View", view);

				if (p_type == false) {
					string ty =  type2str( r_rimg.type() );
					cout << "Matrix: " << ty.c_str() << r_rimg.cols << r_rimg.rows << endl;
					p_type = true;
				}
				imshow("disparity", disp8);

				if (waitKey(10) >= 0) break;
			}
		}
	}
	
	return 0;
}

