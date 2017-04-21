#include "navigateMap.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>


// external API functions 
void initMap(MapState *mapstate, const float cellsize, const unsigned int numcells_x, const unsigned int numcells_y)
{
	unsigned int i, j;
	if(!mapstate)
		return;

	mapstate->mMapProp.CellSize = cellsize;
	mapstate->mMapProp.NumCellsX = numcells_x;
	mapstate->mMapProp.NumCellsY = numcells_y;
	mapstate->mMaptype = Fixed;
	mapstate->mOriginIndex = (unsigned int)(numcells_x*numcells_y/2);

	// allocate the map
	mapstate->mMap = (unsigned char **) malloc(numcells_y*sizeof(unsigned char*));
	for(j = 0; j < numcells_y; j++) {
		mapstate->mMap[j] = (unsigned char *) malloc(numcells_x*sizeof(unsigned char));
	}

	assert(mapstate->mMap != 0);

	for(j = 0; j < numcells_y; j++) {
		for(i = 0; i < numcells_x; i++) {
			mapstate->mMap[j][i] = mGray;
		}
	}
	return;
}


bool loadMapTxt(MapState *mapstate, char *filename)
{
	int i, j;
	if(!mapstate || !filename)
		return false;

	FILE *hf = fopen(filename, "r");
	if(!hf)
		return false;

	unsigned char **map = mapstate->mMap;
	/* map index:
	i: cnt%numCell_x
	j: cnt/numCell_y
	map[j][i] is assigned by the value from the file
	*/
	int cnt= 0;


	while(1)
	{
		// read in file
		// save to map

		if( feof(hf) )
		{ 
			break ;
		}
		cnt++;
	}

	fclose(hf);
	return true;
}

bool loadMapColor(MapState *mapstate, char *imagename)
{
	if(!mapstate || !imagename)
		return false;


	// not implemented
	return true;
}

void closeMap(MapState *mapstate)
{
	unsigned int j;
	if(!mapstate)
		return;

	// free the map
	for(j = 0; j < mapstate->mMapProp.NumCellsY; j++) {
		 free(mapstate->mMap[j]);
	}
	free(mapstate->mMap);
	mapstate->mMap = 0;
}

bool initVehiclePosition(MapState *mapstate, const Point3D Translation, const Point3D Rotation)
{
	if(!mapstate)
		return false;


	return true;
}


bool updateParams(MapState *mapstate)
{
	// not implemented yet
	return false;
}

// returns the total number of cells in the map
const unsigned int getNumCells(MapState *mapstate)
{
	if(!mapstate)
		return (unsigned int) (-1);
	unsigned int numCellsX = mapstate->mMapProp.NumCellsX;
	unsigned int numCellsY = mapstate->mMapProp.NumCellsY;
	return numCellsX * numCellsY;
}

// returns the length of the side of a map cell
const float getCellSize(MapState *mapstate)
{
	if(!mapstate)
		return -1.0f;
	return mapstate->mMapProp.CellSize;
}

unsigned char **get2dMap(MapState *mapstate)
{
	if(!mapstate)
		return 0;

	return mapstate->mMap;
}

// print the map in color
void printMap_color(MapState *mapstate)
{

}

// print out the map in text format
void printMap_txt(MapState *mapstate)
{

}