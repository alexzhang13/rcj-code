#ifndef _LINEFITTING_H_
#define _LINEFITTING_H_

#include "navigate_defs.h"
#include <string>
#include <vector>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include "navigate_defs.h"
#include "cell.h"
#include "floormap.h"

#define BUF_LF_SIZE 360

class NAVIGATE_EXPORT LineFitAlgo {
public:
	typedef struct {
		int32_t d;
		double x;
		double y;
	} Pt2D;

	typedef struct {
		Pt2D pt;
		int32_t status;
	} DataStat;

	typedef struct {
		int32_t timestamp;
		char ctrl;
		int32_t angle;
		int32_t d[4]; // N, E, S, W
		int32_t status[4]; //normal: 1, too far: 0
	} TimeDist;

	typedef struct {
		int32_t adjust_angle;
		std::vector<DataStat> data[4]; // N, E, S, W
	} AdjustPt2D;

	typedef struct {
		MazeCell::NavDir direction;
		float local_angle; // with respect to vehicle right (x)
		float global_angle; // with respect to east (x)
		MazeCell::Position_2D pos;
		MazeCell::xyCoord xypos;
		MazeCell cur_cell;
		std::vector<MazeCell> local_cell_list;
	} DetectedCells;

	typedef struct {
		float k;
		float intercept;
		float local_angle;
		int32_t side; //0: upward, 1: right, 2: downward, 3: left //0: left, 1:right, 2: upward, 3: downward
		float distance;
	} LineDescript;

	typedef struct {
		LineDescript aline;
		std::vector<cv::Point2f> pts;
	} FittedLine;

	// constructor
	LineFitAlgo();
	// destructor
	~LineFitAlgo();

	void setRobotStatus(int32_t cell_index, MazeCell::NavDir direction, MazeCell::Position_2D pos);
	bool readDataFile(const char *filename);
	void update(int32_t half_samples, int32_t angle_separation);  
	bool readData(TimeDist &td);
	bool run();
	bool updateCellConfigs();
	void debugPrints();

	void printoutData();

	inline void setPosition(int32_t xPos, int32_t yPos) {mXpos = xPos; mYpos = yPos;}
	inline void setlineDetectTol(double tor) { mEpsilon = tor; }
	inline double getLineDetectTol(void) { return mEpsilon;}
	
	inline void setValidLinePts(int32_t num) { mLineThresh = num;}
	inline int32_t getValidLinePts(void) { return mLineThresh;}

	inline DetectedCells *getDetectedCells() { return &mDetectedCells;}

protected:

	void resetData();
	void setWallProp(MazeCell &cell, int32_t *wall_status);
	bool parseData(TimeDist *datalist);
	bool convert2Vec();
	bool lineFit();
	void displayAllPoints();
	void displayFittedPoints();

private:
	cv::Mat mImage_all_pts;
	cv::Mat mImage_avg_pts;
	std::vector<TimeDist*> mTimeDistrr_vec; //vector of points for rr
	std::vector<TimeDist*> mTimeDistrl_vec; //vector of points
	TimeDist* mTimeDist_rr;
	TimeDist* mTimeDist_rl;
	AdjustPt2D mPts[BUF_LF_SIZE];
	std::vector<cv::Point2f> mAvgPts;
	std::vector<cv::Point2f> mLines;
	std::map<int32_t, FittedLine> mFittedLines;
	int32_t mShortThresh;
	int32_t mLongThresh;
	int32_t mAngleSep;
	int32_t mHalf_samples;
	int32_t m_SampleCnt;
	int32_t m_scan_angle;
	double mDistOffset;
	double mAngleOffset;
	double mOffset2BotCenter[2];
	int32_t mXmin, mYmin;
	int32_t mXmax, mYmax;
	int32_t mXpos, mYpos;
	double mEpsilon;
	int32_t mLineThresh;
	float mAngleThresh;
	DetectedCells mDetectedCells;
	FILE *m_hf;
	// will be added in code later on
	std::map<int32_t, AdjustPt2D> m_collected_mPts;
	std::map<int32_t, std::vector<cv::Point2f>> m_collected_mAvgPts;
};

///////////////////////////////////////////////////////////////////////
#endif