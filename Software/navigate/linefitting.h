#ifndef _LINEFITTING_H_
#define _LINEFITTING_H_

#include "navigate_defs.h"
#include <string>
#include <vector>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include "navigate_defs.h"

class NAVIGATE_EXPORT LineFitAlgo {
public:
	// constructor
	LineFitAlgo();
	// destructor
	~LineFitAlgo();

	bool readImage(unsigned char *filename);
	bool filterImage();
	bool edgeDetection();
	bool lineFit();
	bool polygonFit();

private:
	cv::Mat mImage;

};

///////////////////////////////////////////////////////////////////////
#endif