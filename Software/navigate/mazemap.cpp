#include "mazemap.h"
#include <assert.h>

MazeMaps::MazeMaps()
{
	//m_cur_floor_staircell_index = -1;
	//m_cur_floor_latest_checkpoint = -1;
	m_home_floor = 0;
	m_home_cell_index = 0;
	m_cur_cell_index = 0;
	m_cur_floor = 0;
	m_floors = 2;

	for(int32_t i = 0; i < 2; i++) {
		m_floor[i].grid_w = 0;
		m_floor[i].grid_h = 0;
		m_floor[i].checkpt = 0;
		m_floor[i].victims = 0;
		m_floor[i].nmovables = 0;
		m_floor[i].obstables = 0;
		m_floor[i].stair = false;
		m_floor[i].wall_percent = 0.2f;
		m_route_trace[i].clear();
	}
	m_disp_img[0].release();
	m_disp_img[1].release();
	m_disp_cell_size = 45;
}

MazeMaps::~MazeMaps()
{
	m_disp_img[0].release();
	m_disp_img[1].release();
}


void MazeMaps::updateParams(int32_t home_floor, int32_t numofFloors)
{
	m_home_floor = home_floor;
	m_floors = numofFloors;
	assert(m_home_floor < 2);
	assert(m_floors <= 2);
}

// because of stair cell is not in the mxn grid, we need to recalculate the grid size and 
// add in extra cell
int32_t MazeMaps::displayPhysicalMap(int32_t floor_num, std::vector<GreedyDijkstra::DistInfo>* trace)
{
	int32_t i, j;
	int32_t display_width, display_height;
	int32_t x,y, indx;

	m_floor[floor_num].grid_w = m_floormap[floor_num].getGridHsize();
	m_floor[floor_num].grid_h = m_floormap[floor_num].getGridVsize();
	m_floormap[floor_num].setGridHsize(m_floor[floor_num].grid_w);
	m_floormap[floor_num].setGridVsize(m_floor[floor_num].grid_h);
	display_width = m_floor[floor_num].grid_w * m_disp_cell_size;
	display_height = m_floor[floor_num].grid_h * m_disp_cell_size;
	m_floormap[floor_num].setMapHsize((float)display_width);
	m_floormap[floor_num].setMapVsize((float)display_height);

	m_disp_img[floor_num] = cv::Mat(display_height, display_width,  CV_8UC3);
	m_disp_img[floor_num].setTo(0x0);
	for(i = 0; i < m_floormap[floor_num].getCellSize(); i++) {
		MazeCell *c =  m_floormap[floor_num].getCell(i);
		indx = c->getCellNum();
		c->getCellGrid(x,y);
		printf("%d (%d %d), %d %d %d %d %d\n", indx, x, y, c->getVisitStatus(), c->getWallNorth(), c->getWallEast(), c->getWallSouth(), c->getWallWest());
		fillRectangle(m_disp_img[floor_num], floor_num, c);
		drawRectangle(m_disp_img[floor_num], floor_num, c);
	}

	if(trace != NULL) {
		int32_t len = (int32_t)(*trace).size();
		for(i = 0; i < len; i++) {
			GreedyDijkstra::DistInfo acell = (*trace)[i];
			for(j = 0; j < acell.waypts.size(); j++) {
				drawLines(m_disp_img[floor_num], floor_num, &acell.waypts);
			}
		}
	}

	std::string name = "Floor " + std::to_string(floor_num);
	cv::namedWindow(name);
	cv::imshow(name, m_disp_img[floor_num]);
	cv::waitKey(10);

	return 0;
}

int32_t MazeMaps::savePhysicalMap(const char *out_dir, const char* name, int32_t floor_num)
{
	std::string imgname = std::string(out_dir) + std::string(name) + "_" + std::to_string(floor_num) + ".bmp";
	if(!m_disp_img[floor_num].empty())
		cv::imwrite(imgname, m_disp_img[floor_num]);
	else
		return -1;
	return 0;
}

void MazeMaps::drawLines(cv::Mat &img, int32_t floor_num, std::vector<int32_t> *trace_pts)
{
	if(trace_pts == NULL || (*trace_pts).size() <=1)
		return;

	int32_t thickness = 3;
	int32_t lineType = 8;
	cv::Point2i pt[4];
	cv::Point2i pt_center[2];
	cv::Scalar color = cv::Scalar(0.0,255.0,255.0);

	for(int32_t i = 0; i < (*trace_pts).size()-1; i++) {
		MazeCell *c0 =  m_floormap[floor_num].getCell((*trace_pts)[i]);
		MazeCell *c1 =  m_floormap[floor_num].getCell((*trace_pts)[i+1]);
		c0->getCellGrid(pt_center[0].x,pt_center[0].y);
		c1->getCellGrid(pt_center[1].x,pt_center[1].y);

		pt[0].x = pt_center[0].x * m_disp_cell_size + m_disp_cell_size/2;
		pt[0].y = m_disp_img[floor_num].rows - (pt_center[0].y * m_disp_cell_size+m_disp_cell_size/2) + 1;
		pt[1].x = pt_center[1].x * m_disp_cell_size + m_disp_cell_size/2;
		pt[1].y = m_disp_img[floor_num].rows - (pt_center[1].y * m_disp_cell_size+m_disp_cell_size/2) + 1;// top right corner

		cv::line( img, pt[0], pt[1], color, thickness, lineType );
	}

	return;
}

void MazeMaps::drawRectangle(cv::Mat &img, int32_t floor_num, MazeCell *cell)
{
	if(cell == NULL)
		return;

	int32_t half_csize = (int32_t)m_disp_cell_size/2;
	int32_t thickness = 2;
	int32_t lineType = 8;
	cv::Point2i pt[4];
	cv::Point2i pt_center;
	cv::Scalar color[4]; //NESW

	cell->getCellGrid(pt_center.x,pt_center.y);
	pt_center.x = pt_center.x * m_disp_cell_size + half_csize;
	pt_center.y = m_disp_img[floor_num].rows - (pt_center.y * m_disp_cell_size + half_csize) -1;

	pt[0].x = pt_center.x - half_csize;
	pt[0].y = pt_center.y + half_csize;// top left corner
	pt[1].x = pt_center.x + half_csize;
	pt[1].y = pt_center.y + half_csize;// top right corner
	pt[2].x = pt_center.x + half_csize;
	pt[2].y = pt_center.y - half_csize;// low right corner
	pt[3].x = pt_center.x - half_csize;
	pt[3].y = pt_center.y - half_csize;// low left corner

	getWallColor(cell->getWallNorth(), color[2]);
	getWallColor(cell->getWallEast(), color[1]);
	getWallColor(cell->getWallSouth(), color[0]);
	getWallColor(cell->getWallWest(), color[3]);

	for(int32_t i = 0; i < 4; i++) {
		cv::line( img, pt[i], pt[(i+1)%4], color[i], thickness, lineType );
	}

	return;
}

void MazeMaps::fillRectangle(cv::Mat &img, int32_t floor_num, MazeCell *cell)
{
	if(cell == NULL)
		return;

	int32_t half_csize = (int32_t)m_disp_cell_size/2;
	int32_t thickness = -1;
	int32_t lineType = 8;
	cv::Point2i pt[2];
	cv::Point2i pt_center;
	cv::Scalar color; //NESW

	cell->getCellGrid(pt_center.x,pt_center.y);
	pt_center.x = pt_center.x * m_disp_cell_size + half_csize;
	pt_center.y = m_disp_img[floor_num].rows - (pt_center.y * m_disp_cell_size + half_csize)-1;

	pt[0].x = pt_center.x - half_csize + 2;
	pt[0].y = pt_center.y + half_csize - 2;// top left corner
	pt[1].x = pt_center.x + half_csize - 2;
	pt[1].y = pt_center.y - half_csize + 2;// low right corner

	std::string indx = std::to_string(cell->getCellNum());
	getCellColor(cell->getCellType(), color);

	cv::rectangle( img, pt[0], pt[1], color, thickness, lineType );

	cv::putText(img, indx, pt_center, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(60,60,250), 1, CV_AA);
	return;
}

int32_t MazeMaps::getWallColor(MazeCell::WallProp wt, cv::Scalar &color)
{
	cv::Scalar clr; // b, g, r
	switch(wt) {
	case MazeCell::MWall:
		clr = cv::Scalar(0.0,0.0,0.0);
		break;
	case MazeCell::MBlack:
		clr = cv::Scalar(64.0,64.0,64.0);
		break;
	case MazeCell::MObstacle:
		clr = cv::Scalar(128.0,128.0,128.0);
		break;	
	case MazeCell::MStair:
		clr = cv::Scalar(0.0,0.0,255.0);
		break;	
	case MazeCell::MOpen:
	default:
		clr = cv::Scalar(255.0,255.0,255.0);
		break;
	case MazeCell::MUnknown:
		clr = cv::Scalar(0.0,128.0,0.0);
	}
	color =  clr;
	return 0;
}

int32_t MazeMaps::getCellColor(MazeCell::CellType ct, cv::Scalar &color)
{
	cv::Scalar clr = cv::Scalar(255.0,255.0,255.0);

	if(ct.GHome)
		clr = cv::Scalar(0.0,0.0,255.0);
	else if(ct.GCheckPt)
		clr = cv::Scalar(200.0,200.0,200.0);
	else if(ct.GVictim)
		clr = cv::Scalar(128.0,0.0,0.0);
	else if(ct.GStair)
		clr = cv::Scalar(128.0,0.0,128.0);
	else if(ct.GNMovable)
		clr = cv::Scalar(0.0,0.0,0.0);
	else if(ct.GObstacle)
		clr = cv::Scalar(0.0,128.0,0.0);

	color = clr;
	return 0;
}

//http://www.grinninglizard.com/tinyxmldocs/index.html
int32_t MazeMaps::writeXmlMap(const char* out_dir, const char* name)
{
	int32_t i, indx, ix, iy;
	float cx, cy;
	std::string xmlname = std::string(out_dir) + std::string(name) + ".xml";
	
	TiXmlDocument xmlmap_doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	xmlmap_doc.LinkEndChild( decl );  

	TiXmlElement* root = new TiXmlElement("mazemap");
	xmlmap_doc.LinkEndChild(root);

	TiXmlElement* floors = new TiXmlElement("floors");
	root->LinkEndChild(floors);
	TiXmlElement* floor0 = new TiXmlElement("floor0");
	root->LinkEndChild(floor0);
	TiXmlElement* floor1 = new TiXmlElement("floor1");
	root->LinkEndChild(floor1);

	// floors
	floors->SetAttribute("levels", m_floors);
	floors->SetAttribute("home", m_home_floor);
	floors->SetAttribute("current", m_cur_floor);

	// floor 0
	TiXmlElement* floorinfo_0 = new TiXmlElement("floorinfo");
	floor0->LinkEndChild(floorinfo_0);
	floorinfo_0->SetAttribute("width", m_floor[0].grid_w);
	floorinfo_0->SetAttribute("height", m_floor[0].grid_h);
	floorinfo_0->SetDoubleAttribute("cellsize", (double)m_floormap[0].getCell(0)->getCellWidth());
	if(m_floormap[0].getHomeCell())
		floorinfo_0->SetAttribute("home", m_floormap[0].getHomeCell()->getCellNum());
	else
		floorinfo_0->SetAttribute("home", -1);
	if( m_floormap[0].getStairCell())
		floorinfo_0->SetAttribute("stair", m_floormap[0].getStairCell()->getCellNum());
	else
		floorinfo_0->SetAttribute("stair", -1);
	if(m_floormap[0].getCheckPtList()->size() > 0) {
		size_t listsize = m_floormap[0].getCheckPtList()->size();
		floorinfo_0->SetAttribute("checkpt", (*m_floormap[0].getCheckPtList())[listsize-1]);
	}
	else
		floorinfo_0->SetAttribute("checkpt",-1);

	for(i = 0; i < m_floormap[0].getCellSize(); i++) {
		MazeCell *c =  m_floormap[0].getCell(i);
		indx = c->getCellNum();
		printf("cell = %d\n", indx);
		c->getCellGrid(ix,iy);
		c->getCenterXY(cx,cy);
		TiXmlElement* cellmap = new TiXmlElement("cell");
		floor0->LinkEndChild(cellmap);
		cellmap->SetAttribute("id", indx);
		cellmap->SetAttribute("i", ix);
		cellmap->SetAttribute("j", iy);
		cellmap->SetAttribute("cx", std::to_string(cx).c_str());
		cellmap->SetAttribute("cy", std::to_string(cy).c_str());
		cellmap->SetAttribute("North", c->getWallNorth());
		cellmap->SetAttribute("East", c->getWallEast());
		cellmap->SetAttribute("South", c->getWallSouth());
		cellmap->SetAttribute("West", c->getWallWest());
		cellmap->SetAttribute("VisitStatus", c->getVisitStatus());
		if(c->getHomeCell())
			cellmap->SetAttribute("HomeCell", c->getHomeCell());
		if(c->getCheckPt())
			cellmap->SetAttribute("CheckPt", c->getCheckPt());
		if(c->getObstacle())
			cellmap->SetAttribute("Obstacle", c->getObstacle());
		if(c->getVictim())
			cellmap->SetAttribute("Victim", c->getVictim());
		if(c->getNonMovable())
			cellmap->SetAttribute("NonMovable", c->getNonMovable());
		if(c->getStairCell())
			cellmap->SetAttribute("Stair", c->getStairCell());
	}

	// floor 1
	if(m_floors > 1) {
		TiXmlElement* floorinfo_1 = new TiXmlElement("floorinfo");
		floor1->LinkEndChild(floorinfo_1);
		floorinfo_1->SetAttribute("width", m_floor[1].grid_w);
		floorinfo_1->SetAttribute("height", m_floor[1].grid_h);
		floorinfo_1->SetDoubleAttribute("cellsize", (double)m_floormap[1].getCell(0)->getCellWidth());
	if(m_floormap[1].getHomeCell())
		floorinfo_1->SetAttribute("home", m_floormap[1].getHomeCell()->getCellNum());
	else
		floorinfo_1->SetAttribute("home", -1);
	if( m_floormap[1].getStairCell())
		floorinfo_1->SetAttribute("stair", m_floormap[1].getStairCell()->getCellNum());
	else
		floorinfo_1->SetAttribute("stair", -1);
	if(m_floormap[1].getCheckPtList()->size() > 0) {
		size_t listsize = m_floormap[1].getCheckPtList()->size();
		floorinfo_1->SetAttribute("checkpt", (*m_floormap[1].getCheckPtList())[listsize-1]);
	}
	else
		floorinfo_1->SetAttribute("checkpt",-1);

		for(i = 0; i < m_floormap[1].getCellSize(); i++) {
			MazeCell *c =  m_floormap[1].getCell(i);
			indx = c->getCellNum();
			c->getCellGrid(ix,iy);
			c->getCenterXY(cx,cy);
			TiXmlElement* cellmap = new TiXmlElement("cell");
			floor1->LinkEndChild(cellmap);
			cellmap->SetAttribute("id", indx);
			cellmap->SetAttribute("i", ix);
			cellmap->SetAttribute("j", iy);
			cellmap->SetAttribute("cx", std::to_string(cx).c_str());
			cellmap->SetAttribute("cy", std::to_string(cy).c_str());
			cellmap->SetAttribute("North", c->getWallNorth());
			cellmap->SetAttribute("East", c->getWallEast());
			cellmap->SetAttribute("South", c->getWallSouth());
			cellmap->SetAttribute("West", c->getWallWest());
			cellmap->SetAttribute("VisitStatus", c->getVisitStatus());
			if(c->getHomeCell())
				cellmap->SetAttribute("HomeCell", c->getHomeCell());
			if(c->getCheckPt())
				cellmap->SetAttribute("CheckPt", c->getCheckPt());
			if(c->getObstacle())
				cellmap->SetAttribute("Obstacle", c->getObstacle());
			if(c->getVictim())
				cellmap->SetAttribute("Victim", c->getVictim());
			if(c->getNonMovable())
				cellmap->SetAttribute("NonMovable", c->getNonMovable());
			if(c->getStairCell())
				cellmap->SetAttribute("Stair", c->getStairCell());
		}
	}

	xmlmap_doc.SaveFile(xmlname.c_str());  
	return 0;
}

int32_t MazeMaps::readXmlMap(const char* out_dir, const char* name)
{
	TiXmlDocument xmldoc;
	std::string xmlname = std::string(out_dir) + std::string(name) + ".xml";
	bool loadOkay = xmldoc.LoadFile(xmlname.c_str());
	if(!loadOkay) 
		return -1;

	TiXmlNode* node = 0;
	TiXmlNode* floornode = 0;
	TiXmlNode* cellnode = 0;
	TiXmlElement *mazeMapElement = NULL;
	TiXmlElement *floorMapElement = NULL;
	TiXmlElement *cellElement = NULL;

	int32_t id, i, j;
	float cx, cy;
	int32_t floor_num;
	float cellsize;
	int32_t North, East, South, West;
	int32_t VisitStatus;
	int32_t Victim, CheckPt, NonMovable, Obstacle, Stair, Home;
	int32_t status;

	for( node = xmldoc.IterateChildren( 0 ); node; node = xmldoc.IterateChildren( node ) )
	{
		printf("%s\n", node->Value());
		mazeMapElement = node->ToElement(); // mazemap
		if(mazeMapElement == NULL)
			continue;

		for( floornode = mazeMapElement->IterateChildren( 0 ); floornode; floornode = mazeMapElement->IterateChildren( floornode ) )
		{
			printf("%s\n", floornode->Value());
			floorMapElement = floornode->ToElement(); // floors, floor0 and floor1
			if(strcmp(floornode->Value(), "floors") == 0) {
				floorMapElement->QueryIntAttribute("levels", &m_floors);
				floorMapElement->QueryIntAttribute("home", &m_home_floor);
				floorMapElement->QueryIntAttribute("current", &m_cur_floor);
				continue;
			}
			if(strcmp(floornode->Value(), "floor0") == 0) {
				floor_num = 0;
			}
			else if(strcmp(floornode->Value(), "floor1") == 0) {
				floor_num = 1;
			}
			for( cellnode = floorMapElement->IterateChildren( 0 ); cellnode; cellnode = floorMapElement->IterateChildren( cellnode ) )
			{
				int32_t homecell,staircell, checkptcell;
				printf("%s\n", cellnode->Value()); // floorinfo and cell
				cellElement = cellnode->ToElement();
				if(strcmp(cellnode->Value(), "floorinfo") == 0) {
					cellElement->QueryIntAttribute("width", &m_floor[floor_num].grid_w);
					cellElement->QueryIntAttribute("height", &m_floor[floor_num].grid_h);
					cellElement->QueryFloatAttribute("cellsize", &cellsize);
					m_floormap[floor_num].setGridHsize(m_floor[floor_num].grid_w);
					m_floormap[floor_num].setGridVsize(m_floor[floor_num].grid_h);
					cellElement->QueryIntAttribute("home", &homecell);
					cellElement->QueryIntAttribute("stair", &staircell);
					cellElement->QueryIntAttribute("checkpt", &checkptcell);
					if(homecell > 0) {
						m_floormap[floor_num].setHomeCellFlag(true);
						m_floormap[floor_num].setHomeCellNum(homecell);
					}
					if(staircell > 0) {
						m_floormap[floor_num].setStairCellIndex(staircell);
					}
					if(checkptcell > 0) {
						m_floormap[floor_num].setLatestChkPtCellIndex(checkptcell);
					}

					m_floormap[floor_num].resetMap();
					for(j = 0; j < m_floor[floor_num].grid_h; j++) {
						for(i = 0; i < m_floor[floor_num].grid_w; i++) {
							m_floormap[floor_num].addNewCell(i,j, cellsize);
						}
					}
				}
				else if(strcmp(cellnode->Value(), "cell") == 0) {
					cellElement->QueryIntAttribute("id", &id);
					printf("%d\n", id);
					MazeCell *cell = m_floormap[floor_num].getCell(id);
					cell->setCellWidth(cellsize);
					cellElement->QueryIntAttribute("i", &i);
					cellElement->QueryIntAttribute("j", &j);
					cell->setCellGrid(i,j);
					cellElement->QueryFloatAttribute("cx", &cx);
					cellElement->QueryFloatAttribute("cy", &cy);
					cell->setCenterXY(cx,cy);
					cellElement->QueryIntAttribute("North", &North);
					cell->setWallNorth((MazeCell::WallProp)North);
					cellElement->QueryIntAttribute("East", &East);
					cell->setWallEast((MazeCell::WallProp)East);
					cellElement->QueryIntAttribute("South", &South);
					cell->setWallSouth((MazeCell::WallProp)South);
					cellElement->QueryIntAttribute("West", &West);
					cell->setWallWest((MazeCell::WallProp)West);
					cellElement->QueryIntAttribute("VisitStatus", &VisitStatus);
					if(VisitStatus == 0) {
						cell->setVisitStatus(MazeCell::NotFound);
						m_floormap[floor_num].getUnknownList()->push_back(cell->getCellNum());
					}
					else if(VisitStatus == 1) {
						cell->setVisitStatus(MazeCell::TobeVisited);
						m_floormap[floor_num].getToBeVisitedList()->push_back(cell->getCellNum());
					}
					else if(VisitStatus == 2) {
						cell->setVisitStatus(MazeCell::Visited);
						m_floormap[floor_num].getVisitedList()->push_back(cell->getCellNum());
					}
					else if(VisitStatus == -1)
						cell->setVisitStatus(MazeCell::Prohibited);
					
					status = cellElement->QueryIntAttribute("Victim", &Victim);
					if(status == TIXML_SUCCESS && Victim == 1) {
						cell->setVictim(Victim > 0);
						m_floormap[floor_num].getVisitedList()->push_back(cell->getCellNum());
					}
					status = cellElement->QueryIntAttribute("CheckPt", &CheckPt);
					if(status == TIXML_SUCCESS && CheckPt == 1) {
						cell->setCheckPt(CheckPt > 0);
						m_floormap[floor_num].getCheckPtList()->push_back(cell->getCellNum());
					}
					status = cellElement->QueryIntAttribute("NonMovable", &NonMovable);
					if(status == TIXML_SUCCESS && NonMovable == 1)
						cell->setNonMovable(NonMovable > 0);
					status = cellElement->QueryIntAttribute("Obstacle", &Obstacle);
					if(status == TIXML_SUCCESS && Obstacle == 1)
						cell->setObstacle(Obstacle > 0);
					status = cellElement->QueryIntAttribute("Stair", &Stair);
					if(status == TIXML_SUCCESS && Stair == 1) {
						cell->setStairCell(Stair > 0);
						m_floormap[floor_num].setStairCell(cell);
					}
					status = cellElement->QueryIntAttribute("Home", &Home);
					if(status == TIXML_SUCCESS && Home == 1) {
						cell->setHomeCell(Home > 0);
						m_floormap[floor_num].setHomeCell(cell);
					}
					Victim = CheckPt = NonMovable = Obstacle = Stair = Home = 0;
					cell->enableCellValid(true);
				}
			}
			m_floormap[floor_num].updateCellArray();
		}
	}

	return 0;
}
