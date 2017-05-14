#include "linefitting.h"

LineFitAlgo::LineFitAlgo()
{
	resetData();
	mShortThresh = 255;
	mLongThresh = 1300;
	mAngleSep = 0;
	m_SampleCnt = 0;
	m_scan_angle = -1;
	mDistOffset = 13.75f; // mm 
	mAngleOffset = -1.5; //-1.8; // degrees
	mEpsilon = 20.0;
	mLineThresh = 10; //25;
	mAngleThresh = 35.0f;
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

	mAvgPts.clear();
	mLines.clear();
	std::map<int32_t, FittedLine>::iterator it; 
	for(it = mFittedLines.begin(); it != mFittedLines.end(); it++) {
		LineFitAlgo::FittedLine fl = it->second;
		fl.pts.clear();
	}
	mFittedLines.clear();
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
	if(angle == m_scan_angle) // angle not updated
		return false;
	else {
		if(angle > m_scan_angle)
			rotate_right = true;
		else if(angle < m_scan_angle)
			rotate_left = false;
		m_scan_angle = angle;
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

		MazeCell::Position_2D pos;
		pos.x = pos.y = 0;
		setRobotStatus(MazeCell::navNorth, pos);
		parseData(mTimeDist_rr);
		parseData(mTimeDist_rl);
		convert2Vec();
		lineFit();
		updateCellConfigs();
		// printoutData();
		displayPoints();
		mAvgPts.clear();
		mLines.clear();
		std::map<int32_t, FittedLine>::iterator it; 
		for(it = mFittedLines.begin(); it != mFittedLines.end(); it++) {
			LineFitAlgo::FittedLine fl = it->second;
			fl.pts.clear();
		}
		mFittedLines.clear();
	}

	fclose(hf);
	return true;
}


void LineFitAlgo::setRobotStatus(MazeCell::NavDir direction, MazeCell::Position_2D pos)
{
	mDetectedCells.direction = direction;
	mDetectedCells.pos = pos;
	mDetectedCells.global_angle = 0.0f;
	mDetectedCells.local_cell_list.clear();
	return;
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
		r = sqrt(A*A + B*B);
		mFittedLines[j].aline.k = B/A;
		mFittedLines[j].aline.intercept = p0.y - mFittedLines[j].aline.k * p0.x;
		mFittedLines[j].aline.local_angle = atan(mFittedLines[j].aline.k) * 180.0f /3.1415926f;
		mFittedLines[j].aline.side = -1;
		mFittedLines[j].aline.distance = fabs(C/r);
		if(fabs(mFittedLines[j].aline.local_angle) < mAngleThresh || fabs(fabs(mFittedLines[j].aline.local_angle) - 180.0f) < mAngleThresh) {
			if(mFittedLines[j].aline.intercept > 0.0f)
				mFittedLines[j].aline.side = 0; // upward
			else
				mFittedLines[j].aline.side = 2; // downward
		}
		else if(fabs(fabs(mFittedLines[j].aline.local_angle) - 90.0f) < mAngleThresh) {
			if(mFittedLines[j].aline.intercept/mFittedLines[j].aline.k > 0.0f)
				mFittedLines[j].aline.side = 3; // left
			else
				mFittedLines[j].aline.side = 1; // right
		}

		for(i = 0; i < mAvgPts.size(); i++) {
			m = mAvgPts[i].x;
			n = mAvgPts[i].y;
			dist = (B*m - A*n + C)/r;
			if(abs(dist) < epsilon) {
				mFittedLines[j].pts.push_back(mAvgPts[i]);
			}
		}
		printf("j = %d, angle %f, dist = %f, side = %d, pts = %d\n", j, mFittedLines[j].aline.local_angle, mFittedLines[j].aline.distance, 
			mFittedLines[j].aline.side, mFittedLines[j].pts.size());
	}

	return true;
}

// check if cell_list size is 0. If it is, no information is obtained.
bool LineFitAlgo::updateCellConfigs()
{
	int32_t i, j;
	cv::Point2f p0, p1;

	float orient_x = 0.0f;
	float orient_y = 0.0f;
	float distance[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	int32_t cnt[4] = {0,0,0,0};
	std::vector<int32_t> wall_status[4];
	int32_t adjusted_wall_status[4] = {-1, -1, -1, -1};

	// no line detected
	if(mLines.size() < 1)
		return false;

	for(i = 0; i < mLines.size()-1; i++) {
		if(mFittedLines[i].pts.size() > mLineThresh) {
			if(mFittedLines[i].aline.side == 0)  // horizontal up
			{
				orient_x+= mFittedLines[i].aline.local_angle;
				distance[0] += mFittedLines[i].aline.distance;
				cnt[0]++;
			}
			else if(mFittedLines[i].aline.side == 2) // horizontal down
			{
				orient_x+= mFittedLines[i].aline.local_angle;
				distance[2] += mFittedLines[i].aline.distance;
				cnt[2]++;
			}
			else if(mFittedLines[i].aline.side == 3) // vertical left
			{
				float angle = mFittedLines[i].aline.local_angle;
				distance[3] += mFittedLines[i].aline.distance;
				if(angle < 0.0f)
					angle = 180.0f + angle;
				orient_y += angle;
				cnt[3]++;
			}
			else if(mFittedLines[i].aline.side == 1) // vertical right
			{
				float angle = mFittedLines[i].aline.local_angle;
				distance[1] += mFittedLines[i].aline.distance;
				if(angle < 0.0f)
					angle = 180.0f + angle;
				orient_y += angle;
				cnt[1]++;
			}
		}
	}

	orient_x = orient_x/(float)(cnt[0] + cnt[2]);
	orient_y = orient_y/(float)(cnt[1] + cnt[3]);
	for(i = 0;  i < 4; i++)
		distance[i] /= float(cnt[i]);
	// we are in trouble, not sure what happened
	if(fabs(fabs(orient_y - orient_x) - 90.0f) > 10.0f)
		return false;

	MazeCell curCell;
	mDetectedCells.cur_cell.setCellGrid(mDetectedCells.pos.x, mDetectedCells.pos.y);
	MazeCell::NavDir direct = mDetectedCells.direction; // global direction

	for(i = 0; i < 4; i++) {
		if(distance[i] > 0.0f) 
		{
			float a_dist = distance[i];
			while(a_dist - curCell.getCellWidth() >=0.0f) {
				wall_status[i].push_back(1);
				a_dist-= curCell.getCellWidth();
			}
			wall_status[i].push_back(0);
		}
	}

	for(i = 0; i < 4; i++) {
		if(wall_status[i].size() > 0)
			adjusted_wall_status[i] = wall_status[(i+(int32_t)direct)%4][0];
	}

	// current cell update
	setWallProp(curCell, adjusted_wall_status);
	mDetectedCells.cur_cell = curCell;

	// local cell detection and update
	for(i = 0; i < 4; i++) {
		int j = 1;
		adjusted_wall_status[0] = adjusted_wall_status[1] = -1;
		adjusted_wall_status[2] = adjusted_wall_status[3] = -1;
		if(wall_status[i].size() > 0) {
			while(wall_status[i].size() > j) {
				MazeCell newcell;
				MazeCell::NavDir newdir = MazeCell::NavDir((i+(int32_t)direct)%4);
				int32_t x, y;
				switch(newdir) {
				case MazeCell::navNorth:
					adjusted_wall_status[0] = wall_status[i][j];
					adjusted_wall_status[2] = wall_status[i][j-1];
					y = mDetectedCells.pos.y + j;
				break;
				case MazeCell::navEast:
					adjusted_wall_status[1] = wall_status[i][j];
					adjusted_wall_status[3] = wall_status[i][j-1];
					x = mDetectedCells.pos.x + j;
				break;
				case MazeCell::navSouth:
					adjusted_wall_status[2] = wall_status[i][j];
					adjusted_wall_status[0] = wall_status[i][j-1];
					y = mDetectedCells.pos.y + j;
				break;
				case MazeCell::navWest:
					adjusted_wall_status[3] = wall_status[i][j];
					adjusted_wall_status[1] = wall_status[i][j-1];
					x = mDetectedCells.pos.x + j;
				break;

				}
				j++;
			}
		}
	}

	return true;
}

void LineFitAlgo::setWallProp(MazeCell &curCell, int32_t *wall_status)
{
	int32_t i;

	for(i = MazeCell::navNorth; i < MazeCell::navDirections; i++) {
		MazeCell::NavDir direct = (MazeCell::NavDir)i;
		if(direct == MazeCell::navNorth) {
			if(wall_status[direct] == 1)
				curCell.setWallNorth(MazeCell::WallProp::MOpen);
			else if(wall_status[direct] == 0)
				curCell.setWallNorth(MazeCell::WallProp::MWall);
		}
		else if(direct == MazeCell::navEast) {
			if(wall_status[direct] == 1)
				curCell.setWallEast(MazeCell::WallProp::MOpen);
			else if(wall_status[direct] == 0)
				curCell.setWallEast(MazeCell::WallProp::MWall);
		}
		else if(direct == MazeCell::navSouth) {
			if(wall_status[direct] == 1)
				curCell.setWallSouth(MazeCell::WallProp::MOpen);
			else if(wall_status[direct] == 0)
				curCell.setWallSouth(MazeCell::WallProp::MWall);
		}
		else if(direct == MazeCell::navWest) {
			if(wall_status[direct] == 1)
				curCell.setWallWest(MazeCell::WallProp::MOpen);
			else if(wall_status[direct] == 0)
				curCell.setWallWest(MazeCell::WallProp::MWall);
		}
	}

	return;
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
		for(j = 0; j < mFittedLines[i].pts.size(); j++) {
			cv::Point center = cv::Point(mFittedLines[i].pts[j].x + xoffset, h-(mFittedLines[i].pts[j].y+yoffset));
			cv::circle( mImage, center,3, cv::Scalar( (i*40)%255, 0, 255 ), thickness, lineType );
		}
	}

#endif

	// display lines
	cv::Point2f p0, p1;
	for(i = 0; i < mLines.size()-1; i++) {
		if(mFittedLines[i].pts.size() > mLineThresh) {
			p0.x = mLines[i].x + xoffset;
			p0.y = h-(mLines[i].y + yoffset);
			p1.x = mLines[i+1].x + xoffset;
			p1.y = h-(mLines[i+1].y + yoffset);
			cv::line(mImage, p0, p1, cv::Scalar( 255, 0, 0 ), 1, 8);
			printf("Line# = %d, angle %f, dist = %f, side = %d, pts = %d\n", i, mFittedLines[i].aline.local_angle, mFittedLines[i].aline.distance,
				mFittedLines[i].aline.side, mFittedLines[i].pts.size());
	cv::imshow("distance sensors", mImage);
	cv::waitKey(0);
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