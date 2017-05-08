#ifndef _LINEFITTING_H_
#define _LINEFITTING_H_

#include "navigate_defs.h"
#include <string>
#include <vector>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include "navigate_defs.h"

#define BUF_LF_SIZE 360

class NAVIGATE_EXPORT LineFitAlgo {
public:
	typedef struct {
		int32_t d;
		double x;
		double y;
	} Pt2D;

	typedef struct {
		int32_t timestamp;
		char ctrl;
		int32_t angle;
		int32_t d[4];
	} TimeDist;

	typedef struct {
		int32_t adjust_angle;
		Pt2D pt[4];
	} AdjustPt2D;


	// constructor
	LineFitAlgo();
	// destructor
	~LineFitAlgo();

	void update(int32_t half_samples, int32_t angle_separation);  
	bool readImage(const char *filename);
	bool readData(TimeDist &td);
	bool parseData(TimeDist *datalist);
	bool readDataFile(const char *filename);
	bool convert2Vec();
	bool filterImage();
	bool edgeDetection();
	bool lineFit();
	bool polygonFit();

	void printoutData();
	void displayPoints();

	inline void setlineDetectTol(double tor) { mEpsilon = tor; }
	inline double getLineDetectTol(void) { return mEpsilon;}
	
	inline void setValidLinePts(int32_t num) { mLineThresh = num;}
	inline int32_t getValidLinePts(void) { return mLineThresh;}

protected:

	void resetData();
private:
	cv::Mat mImage;
	TimeDist mTimeDist_rr[BUF_LF_SIZE];
	TimeDist mTimeDist_rl[BUF_LF_SIZE];
	AdjustPt2D mPts[BUF_LF_SIZE];
	std::vector<cv::Point2f> mAvgPts;
	std::vector<cv::Point2f> mLines;
	std::map<int32_t, std::vector<cv::Point2f>> mFittedLines;
	int32_t mShortThresh;
	int32_t mLongThresh;
	int32_t mAngleSep;
	int32_t mHalf_samples;
	int32_t m_SampleCnt;
	int32_t m_cur_angle;
	double mDistOffset;
	double mAngleOffset;
	int32_t mXmin, mYmin;
	int32_t mXmax, mYmax;
	double mEpsilon;
	int32_t mLineThresh;
};

///////////////////////////////////////////////////////////////////////
#endif