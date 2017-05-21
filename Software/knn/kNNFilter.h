#ifndef _KNN_FILTER_h_
#define _KNN_FILTER_h_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <iostream>
#include <sstream>

using namespace std;

class kNNFilter {

public:
	std::vector<cv::Point> ptContour;           // contour
	cv::Rect boundingRect;                      // bounding rect for contour
	float fltArea;                              // area of contour

	char detectVictim(cv::Mat matTestingNumbers);

	inline bool checkIfContourIsValid() {
		if(fltArea < MIN_CONTOUR_AREA) {return false;} //area too small for contour
		if(boundingRect.width >= (boundingRect.height+5) || (boundingRec.width*1.5) < boundingRect.height) {return false;} //params
		return true;
	}
	inline static bool sortByBoundingRectXPosition(const kNNFilter& cwdLeft, const kNNFilter& cwdRight) { return(cwdLeft.boundingRect.x < cwdRight.boundingRect.x);}

private:
	std::vector<kNNFilter> allContoursWithData;
	std::vector<kNNFilter> validContoursWithData;
	cv::Mat matClassificationInts;      // we will read the classification numbers into this variable as though it is a vector
	cv::Mat matTrainingImagesAsFlattenedFloats;         // we will read multiple images into this single image variable as though it is a vector
	cv::Mat matGrayscale; 
	cv::Mat matBlurred;
	cv::Mat matThresh;
	cv::Mat matThreshCopy;
	std::vector<std::vector<cv::Point>> ptContours;        // declare a vector for the contours
	std::vector<cv::Vec4i> v4iHierarchy;                    // declare a vector for the hierarchy (we won't use this in this program but this may be helpful for reference)
	std::string strFinalString;
};

#endif