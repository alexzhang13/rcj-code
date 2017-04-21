#ifndef _NAVIGATE_MAP_H_
#define _NAVIGATE_MAP_H_


#ifdef __cplusplus
extern "C" {
#endif

	// physical measures of the vehicle 
	typedef struct {
		double x; // local east direction  
		double y; // local north direction
		double z; // local upward
	} Point3D;

	// vehicle centered map or fixed map
	typedef enum {
		Scrolling = 0,	// the map scrolls underneath the vehicle,
		// which remains at the centre
		Fixed = 1	// the map remains fixed, and the vehicle moves around it	
	} MapType;

	// map configuration based on resolution and grid size
	typedef struct {
		float CellSize;  // length of cell side in metres
		unsigned int NumCellsX;  // total number of cells in the map vertically
		unsigned int NumCellsY;  // total number of cells in the map horizontally
	} MapSize;

	// obstacle type
	typedef enum {
		mRed = 0xFF, // wall
		mBlue = 0xFE, // object
		mBlack = 0xFD, // hole
		mYellow = 0xEF, // ramp
		mGreen = 0x80, // free enter
		mGray = 0x0, // Unknown
	} Obstacle_type;


	typedef struct {
		MapType mMaptype; // map type
		MapSize mMapProp; // map property
		unsigned char **mMap; // two dimensional map
		Point3D mOrigin;		  // The centre of the map when it is initialized
		Point3D mVehiclePosition; // The current position of the vehicle
		Point3D mVehiclePose;     // the current pose of the vehicle, from external response
		unsigned int mOriginIndex; // The index to the cell containing the origin
		int mVehicleIndex; // the index to the cell containing the vehicle
	} MapState;

	// external API functions 
	void initMap(MapState *mapstate, const float cellsize, const unsigned int numcells_x, const unsigned int numcells_y);
	bool loadMapTxt(MapState *mapstate, char *filename);
	bool loadMapColor(MapState *mapstate, char *imagename);
	void closeMap(MapState *mapstate);
	bool updateParams(MapState *mapstate); 

	/* 
	Initializes the vehicle position on the map, setting both the map's origin, mOrigin,
	and the vehicle's position, mVehiclePosition, to the value of the input vector
	Translation, and also sets the Vehicle's pose, mVehiclePose to be equal to the Rotation
	argument. Rotation is a unit orientation vector showing the head direction of the vehicle. 

	*/
	bool initVehiclePosition(MapState *mapstate, const Point3D Translation, const Point3D Rotation);

	// returns the total number of cells in the map
	const unsigned int getNumCells(MapState *mapstate);
	// returns the length of the side of a map cell
	const float getCellSize(MapState *mapstate);
	unsigned char **get2dMap(MapState *mapstate);

	/* 
	Takes an integer index 0 < Index < CGridMap::GetNumCells()
	and returns the corresponding map index.  For a "Fixed" map,
	the return value is simply the argument passed in, provided that
	value is in the appropriate range.  For a scrolling map, the index is
	adjusted by the center offset and passed through the mod operation to 
	ensure that it lies in the appropriate range

	(note that the scrolling map always has the vehicle located on the 
	middle cell).
	*/
	const int GetCellIndex(MapState *mapstate, const int Index);

	// print the map in color
	void printMap_color(MapState *mapstate);

	// print out the map in text format
	void printMap_txt(MapState *mapstate);


#ifdef __cplusplus
}
#endif

#endif // _NAVIGATE_MAP_H_