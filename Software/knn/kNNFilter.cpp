//Modified for Robot use from https://github.com/MicrocontrollersAndMore/OpenCV_3_License_Plate_Recognition_Cpp

#include "kNNFilter.h"

char kNNFilter::detectVictim(cv::Mat matTestingNumbers) {

	cv::FileStorage fsClassifications("/home/alex/projects/rcj-code/Software/knn/classifications.xml", cv::FileStorage::READ);
	if (fsClassifications.isOpened() == false) {
		printf("classifications.xml is non-existent\n");
		return('0');
	}

	fsClassifications["classifications"] >> matClassificationInts;      // read classifications section into Mat classifications variable
	fsClassifications.release();

	cv::FileStorage fsTrainingImages("/home/alex/projects/rcj-code/Software/knn/images.xml", cv::FileStorage::READ);
	if (fsTrainingImages.isOpened() == false) {
		printf("images.xml is non-existent\n");
		return('0');
	}
	printf("XML's found.\n");

	fsTrainingImages["images"] >> matTrainingImagesAsFlattenedFloats;           // read images section into Mat training images variable
	fsTrainingImages.release();
	printf("Filled var with image\n");		
	
	cv::Ptr<cv::ml::KNearest> kNearest(cv::ml::KNearest::create());            // instantiate the KNN object
	kNearest->train(matTrainingImagesAsFlattenedFloats, cv::ml::ROW_SAMPLE, matClassificationInts);
	printf("KNN Object Instantiated\n");

	cv::cvtColor(matTestingNumbers, matGrayscale, CV_BGR2GRAY);
	cv::GaussianBlur(matGrayscale, matBlurred, cv::Size(5, 5), 0);
	cv::adaptiveThreshold(matBlurred, matThresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
	printf("Masks Initialized\n");

	matThreshCopy = matThresh.clone();              // make a copy of the thresh image, this in necessary b/c findContours modifies the image
	cv::findContours(matThreshCopy, ptContours, v4iHierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	printf("Contours found.\n");

	for (int i = 0; i < ptContours.size(); i++) {               // for each contour
		kNNFilter contourWithData;                                                    // instantiate a contour with data object
		contourWithData.ptContour = ptContours[i];                                          // assign contour to contour with data
		contourWithData.boundingRect = cv::boundingRect(contourWithData.ptContour);         // get the bounding rect
		contourWithData.fltArea = cv::contourArea(contourWithData.ptContour);               // calculate the contour area
		allContoursWithData.push_back(contourWithData);                                     // add contour with data object to list of all contours with data
	}
	printf("Contours searched.\n");

	for (int i = 0; i < allContoursWithData.size(); i++) {                      // for all contours
		if (allContoursWithData[i].checkIfContourIsValid()) {                   // check if valid
			validContoursWithData.push_back(allContoursWithData[i]);            // if so, append to valid contour list
		}
	}
	printf("Contour's validated.\n");
	// sort contours from left to right
	std::sort(validContoursWithData.begin(), validContoursWithData.end(), kNNFilter::sortByBoundingRectXPosition);
	printf("Contour's Sorted by X Pos.\n");

	for (int i = 0; i < validContoursWithData.size(); i++) {            // for each contour
		cv::rectangle(matTestingNumbers, validContoursWithData[i].boundingRect, cv::Scalar(0, 255, 0), 2);
		cv::Mat matROI = matThresh(validContoursWithData[i].boundingRect);          // get ROI image of bounding rect
		cv::Mat matROIResized;
		cv::resize(matROI, matROIResized, cv::Size(RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT));     // resize image, this will be more consistent for recognition and storage
		cv::Mat matROIFloat;
		matROIResized.convertTo(matROIFloat, CV_32FC1);             // convert Mat to float, necessary for call to find_nearest
		cv::Mat matROIFlattenedFloat = matROIFloat.reshape(1, 1);
		cv::Mat matCurrentChar(0, 0, CV_32F);
		kNearest->findNearest(matROIFlattenedFloat, 1, matCurrentChar);     // finally we can call find_nearest !!!
		float fltCurrentChar = (float)matCurrentChar.at<float>(0, 0);
		strFinalString = strFinalString + char(int(fltCurrentChar));        // append current char to full string
	}
	printf("Contours' ROI checked and converted to string of chars.\n");

	for(int i = 0; i < strFinalString.size(); i++) { //if anything corresponds correctly, return it? logic flawed but will do...
		printf("Char: %c\n", strFinalString[i]);
		if(toupper(strFinalString[i]) == 'H' || toupper(strFinalString[i]) == 'S' || toupper(strFinalString[i]) == 'U') {
			return toupper(strFinalString[i]);
		}
	}

	return('0');
}


