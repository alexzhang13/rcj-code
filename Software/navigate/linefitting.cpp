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
	mAngleOffset = 10.0; //-1.5; //-1.8; // degrees
	mEpsilon = 20.0;
	mLineThresh = 10; //25;
	mAngleThresh = 35.0f;
	mOffset2BotCenter[0] = 9.525f; // dx
	mOffset2BotCenter[1] = 31.75f; // dy
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
		mPts[i].adjust_angle = 0;
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

// sensor data order is: forward, right, downward, left
// correspondingly, offset = 90, 360, 270, 180
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

		mTimeDist_rr[index].status[0] = mTimeDist_rr[index].status[1] = 1;
		mTimeDist_rr[index].status[2] = mTimeDist_rr[index].status[3] = 1;
	
		if(mTimeDist_rr[index].d[0] >= mLongThresh) {
			mTimeDist_rr[index].status[0] = 0; // overflow
		}
		if(mTimeDist_rr[index].d[2] >= mLongThresh) {
			mTimeDist_rr[index].status[2] = 0; // overflow
		}
		if(mTimeDist_rr[index].d[1] >= mShortThresh) {
			mTimeDist_rr[index].status[1] = 0; // overflow
		}
		if(mTimeDist_rr[index].d[3] >= mShortThresh) {
			mTimeDist_rr[index].status[3] = 0; // overflow
		}
	}
	else {
		mTimeDist_rl[index].timestamp = td.timestamp;
		mTimeDist_rl[index].angle = td.angle;
		mTimeDist_rl[index].d[0] = td.d[0];
		mTimeDist_rl[index].d[1] = td.d[1];
		mTimeDist_rl[index].d[2] = td.d[2];
		mTimeDist_rl[index].d[3] = td.d[3];

		mTimeDist_rl[index].status[0] = mTimeDist_rl[index].status[1] = 1;
		mTimeDist_rl[index].status[2] = mTimeDist_rl[index].status[3] = 1;

		if(mTimeDist_rl[index].d[0] >= mLongThresh) {
			mTimeDist_rl[index].status[0] = 0; // overflow
		}
		if(mTimeDist_rl[index].d[2] >= mLongThresh) {
			mTimeDist_rl[index].status[2] = 0; // overflow
		}
		if(mTimeDist_rl[index].d[1] >= mShortThresh) {
			mTimeDist_rl[index].status[1] = 0; // overflow
		}
		if(mTimeDist_rl[index].d[3] >= mShortThresh) {
			mTimeDist_rl[index].status[3] = 0; // overflow
		}
	}

	if(m_SampleCnt == mHalf_samples*2)
		m_SampleCnt = 0;
	return true;
}

// sensor data order is: forward:long, right:short, downward:long, left:short
// correspondingly, offset = 90, 360, 270, 180
bool LineFitAlgo::parseData(TimeDist *datalist)
{
	int32_t i, j;
	int32_t offset[4] = {90, 360, 270, 180};
	int32_t adjust_angle;
	int32_t thresh;
	DataStat datas;

	for(j =0; j < 360/mAngleSep; j++) {
		if(datalist[j].angle != -1) {
			for(i = 0; i < 4; i++) 
			{
				datas.pt.d = datalist[j].d[i];
				datas.status = datalist[j].status[i];
				adjust_angle = offset[i]-datalist[j].angle;

				if(adjust_angle >=0) {
					mPts[adjust_angle%360].adjust_angle = adjust_angle%360;
					mPts[adjust_angle%360].data[i].push_back(datas);
				}
				else {
					mPts[(adjust_angle+360)].adjust_angle = (adjust_angle+360)%360;
					mPts[(adjust_angle+360)].data[i].push_back(datas);
				}
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

		resetData();

		for(int32_t i = 0; i < mHalf_samples*2+1; i++) {
			if(feof(hf)) {
				ret = true;
				break;
			}
			fscanf(hf, "%d %c %d %d %d %d %d", &td.timestamp, &td.ctrl, &td.angle, &td.d[0], &td.d[1], &td.d[2], &td.d[3]);

			readData(td);
		}
		fclose(hf);

		return true;
}

bool LineFitAlgo::run()
{
	parseData(mTimeDist_rr);
	parseData(mTimeDist_rl);
	convert2Vec();
	lineFit();
	updateCellConfigs();


	return true;
}

void LineFitAlgo::debpgPrints()
{
	// printoutData();
	displayPoints();
	mAvgPts.clear();
	mLines.clear();
	std::map<int32_t, FittedLine>::iterator it; 
	for(it = mFittedLines.begin(); it != mFittedLines.end(); it++) {
		LineFitAlgo::FittedLine fl = it->second;
		fl.pts.clear();
	}
	return;
}

void LineFitAlgo::setRobotStatus(int32_t cell_index, MazeCell::NavDir direction, MazeCell::Position_2D pos)
{
	mDetectedCells.direction = direction;
	mDetectedCells.pos = pos;
	mDetectedCells.local_angle = 0.0f;
	mDetectedCells.cur_cell.setCellNum(cell_index);
	mDetectedCells.local_cell_list.clear();
	return;
}


bool LineFitAlgo::convert2Vec()
{
	int32_t i, j, k;
	double xmin =  1.0e9;
	double xmax = -1.0e9; 
	double ymin = 1.0e9;
	double ymax = -1.0e9;

	for(i = 0; i < 360; i+=mAngleSep) {
		for(j = 0; j < 4; j++) {
			for(k = 0; k < mPts[i].data[j].size(); k++) {
				if(mPts[i].data[j][k].status == 1) {
					mPts[i].data[j][k].pt.x =cos(((double)mPts[i].adjust_angle+mAngleOffset)*3.1415926/180)*(mPts[i].data[j][k].pt.d+mDistOffset) - mOffset2BotCenter[0];
					mPts[i].data[j][k].pt.y =sin(((double)mPts[i].adjust_angle+mAngleOffset)*3.1415926/180)*(mPts[i].data[j][k].pt.d+mDistOffset) - mOffset2BotCenter[1];
				}
				else {
					if(j == 0 || j == 2) {
						mPts[i].data[j][k].pt.x = (double)mLongThresh;
						mPts[i].data[j][k].pt.y = (double)mLongThresh;
					}
					else {
						mPts[i].data[j][k].pt.x = 0.0;
						mPts[i].data[j][k].pt.y = 0.0;
					}
				}
				xmin = std::min(xmin, mPts[i].data[j][k].pt.x);
				xmax = std::max(xmax, mPts[i].data[j][k].pt.x);
				ymin = std::min(ymin, mPts[i].data[j][k].pt.y);
				ymax = std::max(ymax, mPts[i].data[j][k].pt.y);
			}
		}
	}

	for(i = 0; i < 360; i+=mAngleSep) {
		printf("%d: ", mPts[i].adjust_angle);
		for(j = 0; j < 4; j++) {
			for(k = 0; k < mPts[i].data[j].size(); k++) {
				printf("%d, ", mPts[i].data[j][k].pt.d);
			}
		}
		printf("\n");
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
			if(mPts[i].data[j].size() > 0) {
				cnt += mPts[i].data[j][k].status;
				pt.x += (float)mPts[i].data[j][k].pt.x;
				pt.y += (float)mPts[i].data[j][k].pt.y;
			}
		}

		if(cnt > 0) {
			pt.x /= float(cnt);
			pt.y /= float(cnt);
			mAvgPts.push_back(pt);
		}
		else if(j == 0 || j == 2) {
			pt.x = mLongThresh;
			pt.y = mLongThresh;
			mAvgPts.push_back(pt);
		}
		else if(j == 1 || j == 3) {
			pt.x = mShortThresh;
			pt.y = mShortThresh;
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

	for(j = 0; j < mFittedLines.size(); j++) {
		mFittedLines[j].pts.clear();
	}
	mFittedLines.clear();

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
			else if(mFittedLines[i].aline.side == 1) // vertical right
			{
				float angle = mFittedLines[i].aline.local_angle;
				distance[1] += mFittedLines[i].aline.distance;
				if(angle < 0.0f)
					angle = 180.0f + angle;
				orient_y += angle;
				cnt[1]++;
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
		}
	}

	if(cnt[0] + cnt[2] > 0)
		orient_x = orient_x/(float)(cnt[0] + cnt[2]);
	else
		orient_x = 0.0f;

	if(cnt[1] + cnt[3] > 0)
		orient_y = orient_y/(float)(cnt[1] + cnt[3]);
	else
		orient_y = 90.0f;

	for(i = 0;  i < 4; i++) {
		if(cnt[i]  > 0)
			distance[i] /= float(cnt[i]);
		else
			distance[i] = -1; // at least one side is open
	}
	// we are in trouble, not sure what happened
	if(fabs(fabs(orient_y - orient_x) - 90.0f) > 10.0f)
		return false;

	float xo, yo;
	float avg_orient = (orient_y + (90.0f - orient_x))/2;
	mDetectedCells.cur_cell.setCellGrid(mDetectedCells.pos.x, mDetectedCells.pos.y);
	mDetectedCells.cur_cell.setCenterXY(mDetectedCells.pos.x*mDetectedCells.cur_cell.getCellWidth(), mDetectedCells.pos.y*mDetectedCells.cur_cell.getCellWidth());
	mDetectedCells.cur_cell.setNavDirection(mDetectedCells.direction);
	mDetectedCells.local_angle = avg_orient;
	mDetectedCells.global_angle = avg_orient - (float)mDetectedCells.direction * 90.0f;
	MazeCell::NavDir direct = mDetectedCells.direction; // global direction

	float short_distance[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	for(i = 0; i < 4; i++) {
		if(distance[i] > 0.0f) 
		{
			short_distance[i] = distance[i];
			float a_dist = distance[i];
			while(a_dist - mDetectedCells.cur_cell.getCellWidth() >=0.0f) {
				wall_status[i].push_back(1);
				a_dist-= mDetectedCells.cur_cell.getCellWidth();
				if(a_dist > 0.0f && a_dist < mDetectedCells.cur_cell.getCellWidth())
					short_distance[i] = a_dist;
			}
			if(distance[i] < mLongThresh)
				wall_status[i].push_back(0);
		}
	}

	for(i = 0; i < 4; i++) {
		if(wall_status[(i+(int32_t)direct)%4].size() > 0)
			adjusted_wall_status[i] = wall_status[(i+(int32_t)direct)%4][0];
	}

	if(wall_status[2].size() + wall_status[0].size() > 0)
		xo = (wall_status[2].size()*short_distance[2] + wall_status[0].size()*(mDetectedCells.cur_cell.getCellWidth()-short_distance[0]))/(wall_status[2].size()+ wall_status[0].size());
	else
		xo = 0.0f;
	if(wall_status[1].size() + wall_status[3].size() > 0)
		yo = (wall_status[1].size()*short_distance[1] + wall_status[3].size()*(mDetectedCells.cur_cell.getCellWidth()-short_distance[3]))/(wall_status[1].size()+wall_status[3].size());
	else
		yo = 0.0f;

	// with respect to the center of the cell
	xo -= mDetectedCells.cur_cell.getCellWidth()/2.0f;
	yo -= mDetectedCells.cur_cell.getCellWidth()/2.0f;

	mDetectedCells.xypos.x = xo;
	mDetectedCells.xypos.y = yo;

	// current cell update
	setWallProp(mDetectedCells.cur_cell, adjusted_wall_status);

	// local cell detection and update
	for(i = 0; i < 4; i++) {
		int j = 0;
		adjusted_wall_status[0] = adjusted_wall_status[1] = -1;
		adjusted_wall_status[2] = adjusted_wall_status[3] = -1;
		if(wall_status[i].size() > 0) {
			while(j < wall_status[i].size()) {
				int32_t x, y, xo, yo;
				int k;
				MazeCell newcell;
				MazeCell::NavDir newdir = MazeCell::NavDir((i+(int32_t)direct)%4);
				switch(newdir) {
				case MazeCell::navNorth:
					x = mDetectedCells.pos.x;
					if(wall_status[i][j] > 0)
						y = mDetectedCells.pos.y + j + 1;
					else
						y = mDetectedCells.pos.y + j;
					
					mDetectedCells.cur_cell.getCellGrid(xo, yo);
					if(x == xo && y == yo) {
						mDetectedCells.cur_cell.setWallNorth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						j++;
						continue;
					}
							
					for(k = 0; k < mDetectedCells.local_cell_list.size(); k++) {
						mDetectedCells.local_cell_list[k].getCellGrid(xo, yo);
						if(x == xo && y == yo)
							break;
					};
					// find a match
					if(k < mDetectedCells.local_cell_list.size()) {
						mDetectedCells.local_cell_list[k].setWallNorth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
					}
					else {
						newcell.setCellGrid(x,y);
						newcell.setCenterXY(x*newcell.getCellWidth(), y*newcell.getCellWidth());
						newcell.setWallSouth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						mDetectedCells.local_cell_list.push_back(newcell);
					}
					break;
				case MazeCell::navEast:
					if(wall_status[i][j] > 0)
						x = mDetectedCells.pos.x + j + 1;
					else
						x = mDetectedCells.pos.x + j;
					y = mDetectedCells.pos.y;

					mDetectedCells.cur_cell.getCellGrid(xo, yo);
					if(x == xo && y == yo) {
						mDetectedCells.cur_cell.setWallEast(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						j++;
						continue;
					}

					for(k = 0; k < mDetectedCells.local_cell_list.size(); k++) {
						mDetectedCells.local_cell_list[k].getCellGrid(xo, yo);
						if(x == xo && y == yo)
							break;
					};
					// find a match
					if(k < mDetectedCells.local_cell_list.size()) {
						mDetectedCells.local_cell_list[k].setWallEast(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
					}
					else {
						newcell.setCellGrid(x,y);
						newcell.setCenterXY(x*newcell.getCellWidth(), y*newcell.getCellWidth());
						newcell.setWallWest(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						mDetectedCells.local_cell_list.push_back(newcell);
					}
					break;
				case MazeCell::navSouth:
					x = mDetectedCells.pos.x;
					if(wall_status[i][j] > 0)
						y = mDetectedCells.pos.y - j - 1;
					else
						y = mDetectedCells.pos.y - j;

					mDetectedCells.cur_cell.getCellGrid(xo, yo);

					if(x == xo && y == yo) {
						mDetectedCells.cur_cell.setWallSouth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						j++;
						continue;
					}

					for(k = 0; k < mDetectedCells.local_cell_list.size(); k++) {
						mDetectedCells.local_cell_list[k].getCellGrid(xo, yo);
						if(x == xo && y == yo)
							break;
					};
					// find a match
					if(k < mDetectedCells.local_cell_list.size()) {
						mDetectedCells.local_cell_list[k].setWallSouth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
					}
					else {
						newcell.setCellGrid(x,y);
						newcell.setCenterXY(x*newcell.getCellWidth(), y*newcell.getCellWidth());
						newcell.setWallNorth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						mDetectedCells.local_cell_list.push_back(newcell);
					}
					break;
				case MazeCell::navWest:
					if(wall_status[i][j] > 0)
						x = mDetectedCells.pos.x - j -1;
					else
						x = mDetectedCells.pos.x - j;
					y = mDetectedCells.pos.y;

					mDetectedCells.cur_cell.getCellGrid(xo, yo);
					if(x == xo && y == yo) {
						mDetectedCells.cur_cell.setWallWest(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						j++;
						continue;
					}
					for(k = 0; k < mDetectedCells.local_cell_list.size(); k++) {
						mDetectedCells.local_cell_list[k].getCellGrid(xo, yo);
						if(x == xo && y == yo)
							break;
					};
					// find a match
					if(k < mDetectedCells.local_cell_list.size()) {
						mDetectedCells.local_cell_list[k].setWallWest(MazeCell::MOpen);
					}
					else {
						newcell.setCellGrid(x,y);
						newcell.setCenterXY(x*newcell.getCellWidth(), y*newcell.getCellWidth());
						newcell.setWallEast(MazeCell::MOpen);
						mDetectedCells.local_cell_list.push_back(newcell);
					}
					break;
				} // switch
				j++;
			}

		}
		else {
				int32_t x, y, xo, yo;
				int k;
				MazeCell newcell;
				MazeCell::NavDir newdir = MazeCell::NavDir((i+(int32_t)direct)%4);
				switch(newdir) {
				case MazeCell::navNorth:
					x = mDetectedCells.pos.x;
					y = mDetectedCells.pos.y + j + 1;
					mDetectedCells.cur_cell.setWallNorth(MazeCell::MOpen);

					for(k = 0; k < mDetectedCells.local_cell_list.size(); k++) {
						mDetectedCells.local_cell_list[k].getCellGrid(xo, yo);
						if(x == xo && y == yo)
							break;
					};
					// find a match
					if(k < mDetectedCells.local_cell_list.size()) {
						mDetectedCells.local_cell_list[k].setWallNorth(MazeCell::MOpen);
					}
					else {
						newcell.setCellGrid(x,y);
						newcell.setCenterXY(x*newcell.getCellWidth(), y*newcell.getCellWidth());
						newcell.setWallSouth(MazeCell::MOpen);
						mDetectedCells.local_cell_list.push_back(newcell);
					}
					break;
				case MazeCell::navEast:
					if(wall_status[i][j] > 0)
						x = mDetectedCells.pos.x + j + 1;
					else
						x = mDetectedCells.pos.x + j;
					y = mDetectedCells.pos.y;

					mDetectedCells.cur_cell.getCellGrid(xo, yo);
					if(x == xo && y == yo) {
						mDetectedCells.cur_cell.setWallEast(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						j++;
						continue;
					}

					for(k = 0; k < mDetectedCells.local_cell_list.size(); k++) {
						mDetectedCells.local_cell_list[k].getCellGrid(xo, yo);
						if(x == xo && y == yo)
							break;
					};
					// find a match
					if(k < mDetectedCells.local_cell_list.size()) {
						mDetectedCells.local_cell_list[k].setWallEast(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
					}
					else {
						newcell.setCellGrid(x,y);
						newcell.setCenterXY(x*newcell.getCellWidth(), y*newcell.getCellWidth());
						newcell.setWallWest(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						mDetectedCells.local_cell_list.push_back(newcell);
					}
					break;
				case MazeCell::navSouth:
					x = mDetectedCells.pos.x;
					if(wall_status[i][j] > 0)
						y = mDetectedCells.pos.y - j - 1;
					else
						y = mDetectedCells.pos.y - j;

					mDetectedCells.cur_cell.getCellGrid(xo, yo);

					if(x == xo && y == yo) {
						mDetectedCells.cur_cell.setWallSouth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						j++;
						continue;
					}

					for(k = 0; k < mDetectedCells.local_cell_list.size(); k++) {
						mDetectedCells.local_cell_list[k].getCellGrid(xo, yo);
						if(x == xo && y == yo)
							break;
					};
					// find a match
					if(k < mDetectedCells.local_cell_list.size()) {
						mDetectedCells.local_cell_list[k].setWallSouth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
					}
					else {
						newcell.setCellGrid(x,y);
						newcell.setCenterXY(x*newcell.getCellWidth(), y*newcell.getCellWidth());
						newcell.setWallNorth(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						mDetectedCells.local_cell_list.push_back(newcell);
					}
					break;
				case MazeCell::navWest:
					if(wall_status[i][j] > 0)
						x = mDetectedCells.pos.x - j -1;
					else
						x = mDetectedCells.pos.x - j;
					y = mDetectedCells.pos.y;

					mDetectedCells.cur_cell.getCellGrid(xo, yo);
					if(x == xo && y == yo) {
						mDetectedCells.cur_cell.setWallWest(wall_status[i][j] > 0? MazeCell::MOpen : MazeCell::MWall);
						j++;
						continue;
					}
					for(k = 0; k < mDetectedCells.local_cell_list.size(); k++) {
						mDetectedCells.local_cell_list[k].getCellGrid(xo, yo);
						if(x == xo && y == yo)
							break;
					};
					// find a match
					if(k < mDetectedCells.local_cell_list.size()) {
						mDetectedCells.local_cell_list[k].setWallWest(MazeCell::MOpen);
					}
					else {
						newcell.setCellGrid(x,y);
						newcell.setCenterXY(x*newcell.getCellWidth(), y*newcell.getCellWidth());
						newcell.setWallEast(MazeCell::MOpen);
						mDetectedCells.local_cell_list.push_back(newcell);
					}
					break;
				} // switch
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