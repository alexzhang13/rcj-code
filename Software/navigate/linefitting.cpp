#include "linefitting.h"

LineFitAlgo::LineFitAlgo()
{
	resetData();
	mShortThresh = 255;
	mLongThresh = 1300;
	mAngleSep = 0;
	m_SampleCnt = 0;
	m_cur_angle = -1;
	mDistOffset = 13.75f; // mm 
	mAngleOffset = -1.5; //-1.8; // degrees
	mEpsilon = 20.0;
	mLineThresh = 25;
	mAvgPts.clear();
	mFittedLines.clear();
}

LineFitAlgo::~LineFitAlgo()
{
	resetData();
}

void LineFitAlgo::resetData()
{
	int32_t i, j;
	for(i = 0; i < BUF_LF_SIZE; i++) {
		mTimeDist_rr[i].timestamp = -1;
		mTimeDist_rr[i].angle = -1;
		mTimeDist_rr[i].d[0] = 0;
		mTimeDist_rr[i].d[1] = 0;
		mTimeDist_rr[i].d[2] = 0;
		mTimeDist_rr[i].d[3] = 0;
	}

	for(i = 0; i < BUF_LF_SIZE; i++) {
		mTimeDist_rl[i].timestamp = -1;
		mTimeDist_rl[i].angle = -1;
		mTimeDist_rl[i].d[0] = 0;
		mTimeDist_rl[i].d[1] = 0;
		mTimeDist_rl[i].d[2] = 0;
		mTimeDist_rl[i].d[3] = 0;
	}

	for(i = 0; i < BUF_LF_SIZE; i++) {
		mPts[i].adjust_angle = -1;
		for(j = 0; j < 4; j++) {
			mPts[i].pt[j].d = 0;
			mPts[i].pt[j].x = 0.0;
			mPts[i].pt[j].y = 0.0;
		}
	}
}

void LineFitAlgo::update(int32_t half_samples, int32_t angle_separation)
{
	mAngleSep = angle_separation;
	mHalf_samples = half_samples;
	return;
}

bool LineFitAlgo::readImage(const char *filename)
{

	return true;
}

// sensor data order is: right, left, forwward and backward
// correspondingly, offset = 0, 180, 90, 270
bool LineFitAlgo::readData(TimeDist &td)
{
	int32_t index;
	bool rotate_right = false;
	bool rotate_left = false;

	int32_t angle = td.angle;
	if(angle == m_cur_angle) // angle not updated
		return false;
	else {
		if(angle > m_cur_angle)
			rotate_right = true;
		else if(angle < m_cur_angle)
			rotate_left = false;
		m_cur_angle = angle;
		m_SampleCnt++;
	}

	index = angle/mAngleSep;

	if(rotate_right) {
		mTimeDist_rr[index].timestamp = td.timestamp;
		mTimeDist_rr[index].angle = td.angle;
		mTimeDist_rr[index].d[0] = td.d[0];
		mTimeDist_rr[index].d[1] = td.d[1];
		mTimeDist_rr[index].d[2] = td.d[2];
		mTimeDist_rr[index].d[3] = td.d[3];
	}
	else {
		mTimeDist_rl[index].timestamp = td.timestamp;
		mTimeDist_rl[index].angle = td.angle;
		mTimeDist_rl[index].d[0] = td.d[0];
		mTimeDist_rl[index].d[1] = td.d[1];
		mTimeDist_rl[index].d[2] = td.d[2];
		mTimeDist_rl[index].d[3] = td.d[3];
	}

	if(m_SampleCnt == mHalf_samples*2)
		m_SampleCnt = 0;
	return true;
}

// sensor data order is: right, left, forwward and backward
// correspondingly, offset = 0, 180, 90, 270
bool LineFitAlgo::parseData(TimeDist *datalist)
{
	int32_t i, j;
	int32_t offset[4] = {360, 180, 90, 270};
	int32_t adjust_angle;
	int32_t thresh;

	for(j =0; j < 360/mAngleSep; j++) {
		for(i = 0; i < 4; i++) 
		{
			adjust_angle = offset[i]-datalist[j].angle;
			if(i == 0 || i == 1)
				thresh = mShortThresh;
			else if( i == 2 || i == 3)
				thresh = mLongThresh;
			if(adjust_angle >=0) {
				mPts[adjust_angle%360].adjust_angle = adjust_angle%360;
				if((datalist[j].d[i] >= thresh || datalist[j].timestamp == -1 || datalist[j].d[i] == 0)) {
					mPts[adjust_angle%360].pt[i].x = 0;
					mPts[adjust_angle%360].pt[i].y = 0;
					mPts[adjust_angle%360].pt[i].d = 0;
				}
				else
					mPts[adjust_angle%360].pt[i].d = datalist[j].d[i];
			}
			else {
				mPts[(adjust_angle + 360)].adjust_angle = (adjust_angle+360)%360;
				if((datalist[j].d[i] >= thresh || datalist[j].timestamp == -1 || datalist[j].d[i] == 0)) {
					mPts[(adjust_angle+360)].pt[i].x = 0;
					mPts[(adjust_angle+360)].pt[i].y = 0;
					mPts[(adjust_angle+360)].pt[i].d = 0;
				}
				else
					mPts[(adjust_angle+360)].pt[i].d = datalist[j].d[i];
			}
		}
	}

	return true;
}

bool LineFitAlgo::readDataFile(const char *filename)
{
	TimeDist td;
	bool ret = false;
	FILE *hf = fopen(filename, "r+");
	if(hf == NULL)
		return false;

	while(1) { 
		resetData();
		for(int32_t i = 0; i < mHalf_samples*2; i++) {
			if(feof(hf)) {
				ret = true;
				break;
			}
			fscanf(hf, "%d %c %d %d %d %d %d", &td.timestamp, &td.ctrl, &td.angle, &td.d[0], &td.d[1], &td.d[2], &td.d[3]);

			readData(td);
		}

		if(ret)
			break;

		parseData(mTimeDist_rr);
		parseData(mTimeDist_rl);
		convert2Vec();
		lineFit();
		printoutData();
		displayPoints();
		mAvgPts.clear();
		mLines.clear();
		mFittedLines.clear();
	}

	fclose(hf);
	return true;
}

bool LineFitAlgo::convert2Vec()
{
	int32_t i, j;
	double xmin =  1.0e9;
	double xmax = -1.0e9; 
	double ymin = 1.0e9;
	double ymax = -1.0e9;

	for(i = 0; i < 360; i+=mAngleSep) {
		for(j = 0; j < 4; j++) {
			if(mPts[i].pt[j].d > 0) {
				mPts[i].pt[j].x =cos(((double)mPts[i].adjust_angle+mAngleOffset)*3.1415926/180)*(mPts[i].pt[j].d+mDistOffset);
				mPts[i].pt[j].y =sin(((double)mPts[i].adjust_angle+mAngleOffset)*3.1415926/180)*(mPts[i].pt[j].d+mDistOffset);

				xmin = std::min(xmin, mPts[i].pt[j].x);
				xmax = std::max(xmax, mPts[i].pt[j].x);
				ymin = std::min(ymin, mPts[i].pt[j].y);
				ymax = std::max(ymax, mPts[i].pt[j].y);
			}
		}
	}

	mXmin = int32_t(xmin);
	mYmin = int32_t(ymin);
	mXmax = int32_t(xmax);
	mYmax = int32_t(ymax);

	assert(mXmax > mXmin);
	assert(mYmax > mYmin);

	cv::Point2f pt;
	int32_t cnt;
	for(i = 0; i < 360; i+=mAngleSep) {
		cnt = 0;
		pt.x = 0.0f;
		pt.y = 0.0f;
		for(j = 0; j < 4; j++) {
			if(mPts[i].pt[j].d > 0) {
				pt.x += (float)mPts[i].pt[j].x;
				pt.y += (float)mPts[i].pt[j].y;
				cnt++;
			}
		}
		if(cnt > 0) {
			pt.x /= float(cnt);
			pt.y /= float(cnt);
			mAvgPts.push_back(pt);
		}
		else {
			pt.x = 0.0f;
			pt.y = 0.0f;
		}
	}

	return true;
}


bool LineFitAlgo::filterImage()
{

	return true;
}

bool LineFitAlgo::edgeDetection()
{

	return true;
}

bool LineFitAlgo::lineFit()
{
	int32_t i, j;
	float A, B, C;
	float m, n, r;
	float dist;
	if(!mAvgPts.size())
		return false;

	double epsilon = mEpsilon;
	bool closed = false;
	cv::approxPolyDP(mAvgPts, mLines, epsilon, closed);

	if(mLines.size() < 1)
		return false;

	// check how many points are close to a line
	for(j = 0; j < mLines.size()-1; j++) {
		cv::Point2f p0 = mLines[j];
		cv::Point2f p1 = mLines[j+1];
		A = p1.x - p0.x;
		B = p1.y - p0.y;
		C = p1.x*p0.y - p1.y*p0.x;

		for(i = 0; i < mAvgPts.size(); i++) {
			m = mAvgPts[i].x;
			n = mAvgPts[i].y;
			r = sqrt(A*A + B*B);
			dist = (B*m - A*n + C)/r;
			if(abs(dist) < epsilon) {
				mFittedLines[j].push_back(mAvgPts[i]);
			}
		}
	}

	return true;
}

bool LineFitAlgo::polygonFit()
{

	return true;
}

void LineFitAlgo::displayPoints()
{
	int32_t i, j;
	int32_t w = ((mXmax - mXmin + 7)/8)*8 + 32;
	int32_t h = ((mYmax - mYmin + 8)/8)*8 + 32;
	int32_t xoffset = -mXmin + 16;
	int32_t yoffset = -mYmin + 16;

	int thickness = -1;
	int lineType = 8;
	mImage.release();
	mImage = cv::Mat(h, w,  CV_8UC3);
	mImage.setTo(0xFF);

#if 0
	// display points
	for(i = 0; i < mAvgPts.size(); i++) {
		cv::Point center = cv::Point(mAvgPts[i].x + xoffset, h-(mAvgPts[i].y+yoffset));
		cv::circle( mImage, center,3, cv::Scalar( 0, 0, 255 ), thickness, lineType );
	}
#else
	for(i = 0; i < mLines.size()-1; i++) {
		for(j = 0; j < mFittedLines[i].size(); j++) {
			cv::Point center = cv::Point(mFittedLines[i][j].x + xoffset, h-(mFittedLines[i][j].y+yoffset));
			cv::circle( mImage, center,3, cv::Scalar( (i*40)%255, 0, 255 ), thickness, lineType );
		}
	}

#endif

	// display lines
	cv::Point2f p0, p1;
	for(i = 0; i < mLines.size()-1; i++) {
		if(mFittedLines[i].size() > mLineThresh) {
			p0.x = mLines[i].x + xoffset;
			p0.y = h-(mLines[i].y + yoffset);
			p1.x = mLines[i+1].x + xoffset;
			p1.y = h-(mLines[i+1].y + yoffset);
			cv::line(mImage, p0, p1, cv::Scalar( 255, 0, 0 ), 1, 8);
		}
	}

	// display origin
	cv::Point center = cv::Point(xoffset, h-yoffset);
	cv::circle( mImage, center,5, cv::Scalar( 0, 255, 0 ), 2, lineType );

	cv::imshow("distance sensors", mImage);
	cv::waitKey(500);
	return;
}

void LineFitAlgo::printoutData()
{
	int32_t i,j;
	for(i = 0; i < mAvgPts.size(); i++) {
		printf("%f %f\n", mAvgPts[i].x, mAvgPts[i].y);
	}

	return;
}