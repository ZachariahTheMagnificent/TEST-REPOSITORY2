#include "HeatMap.h"
#include "HMMath.h"

// Cell struct (for heatmap matrix cells)
Vec2 Cell::heatMapCenterWS;
Vec2 Cell::heatMapSize_WorldUnits;
Vec2 Cell::heatMapResolution;

// KillRecord struct
bool KillRecord::DiedFacing() {
	//Vec2 pathToKiller = killerXY - victimXY;
	//Normal directionToKiller(pathToKiller);
	//Rotator deathRotation = directionToKiller.ToRotator();
	//Rotator deathRotationOffset = deathRotation - victimRotation;
	//return deathRotationOffset < 0.785398; // in radians

	// alt
	Vec2 pathToKiller = killerXY - victimXY;
	Normal directionToKiller = pathToKiller;
	Rotator victimRot(victimRotation);
	const Normal victimFacingDir = victimRot.ToVector();
	const float dp = victimFacingDir.Dot(directionToKiller);
	return abs(dp) > MIN_FACING_DP;
}

// TODO: DONT NEED TO NORMALIZE DOT OR OUTER PRODUCTS!!!!
// TODO: CHECK PROPER DIRECTION OF OUTER PRODUCT!!!
bool KillRecord::DiedObstructed(std::vector<GeoObject>& obstacles) {
	bool isObstructed = false;

	Vec2 pathToVictim = victimXY - killerXY;
	Normal dirToVictim(pathToVictim); // auto normalized on construction

	// Setup left and right bounds to check overlap with
	Normal leftBound = dirToVictim;
	leftBound.Swizzle(); // rotate 90 degrees
	Normal rightBound = leftBound;

	leftBound.x *= PLAYER_RADIUS;
	leftBound.y *= PLAYER_RADIUS;
	rightBound.x *= -PLAYER_RADIUS;
	rightBound.y *= -PLAYER_RADIUS;

	// Add path to victim to compute left and right bounds of killer's FOV
	leftBound.x += pathToVictim.x;
	leftBound.y += pathToVictim.y;
	rightBound.x += pathToVictim.x;
	rightBound.y += pathToVictim.y;

	leftBound.Normalize();
	rightBound.Normalize();

	// Iterate through all obstacles (use QUADTREE) and see if any of them are 
	// between the victim and the killer (same as a CAPSULE SWEEP)
	for (auto& obst : obstacles) {
		if (obst.type == CIRCLE) {
			// (For solid perfect circles, checking their position against victim position
			// should be enough here, because from any such position, they can't "sneak" any 
			// part of their geometry between killer and victim)

			// 1. Filter objects behind the killer
			Vec2 pathToObstacle = obst.center - killerXY;
			Normal dirToObst = pathToObstacle; // auto normalized on construction // TODO NO NEED NORMALIZE - add into Vec2 and move this down!!!!
			if (dirToObst.Dot(dirToVictim) < 0) {
				return false;
			}

			// 2. Filter objects behind the victim
			if (pathToObstacle.SizeSquared() > pathToVictim.SizeSquared()) {
				return false;
			}

			// 3. If these checks have passed, do actual checks for overlap with cone of vision
			//  a) If circle center to the right of left bound or left of right bound, succeed immediately
			Vec2 killerToCirc(obst.center - killerXY);
			Normal dirToCirc(killerToCirc); // TODO add outerproduct in Vec2
			if ((leftBound.OuterProduct(dirToCirc) > 0 &&
				rightBound.OuterProduct(dirToCirc) < 0))
				return true;

			const int radiusSquared = obst.shapeData.circ.radius * obst.shapeData.circ.radius;

			// if circle is to the left of left bound
			if ((leftBound.OuterProduct(dirToCirc) > 0)) {
				leftBound.Swizzle();
				const float projectedMagnitude = leftBound.x * killerToCirc.x + leftBound.y * killerToCirc.y; // DP
				leftBound.x *= projectedMagnitude;
				leftBound.y *= projectedMagnitude;
				Vec2 edgeToCirc(killerToCirc.x - leftBound.x, killerToCirc.y - leftBound.y); // TODO: FIX VECTOR CLASSSES!!!!
				return edgeToCirc.SizeSquared() < radiusSquared;
			}
			else {
				// it must be ro the right of right bound...
				rightBound.Swizzle();
				const float projectedMagnitude = rightBound.x * killerToCirc.x + rightBound.y * killerToCirc.y; // DP
				rightBound.x *= projectedMagnitude;
				rightBound.y *= projectedMagnitude;
				Vec2 edgeToCirc(killerToCirc.x - rightBound.x, killerToCirc.y - rightBound.y); // TODO: FIX VECTOR CLASSSES!!!!
				return edgeToCirc.SizeSquared() < radiusSquared;
			}
		}
		else {
			// Bounds checking for RECTANGLE
			// 1. If center is within bounds, dont need to check all points
			Normal pathToObst(obst.center - killerXY);
			if ((leftBound.OuterProduct(pathToObst) > 0 &&
				rightBound.OuterProduct(pathToObst) < 0))
				return true;

			// 2. Calculate all points of rectangle
			// CONFIRM - ok to lose accuracy here???
			Vec2 halfWidthHeight(obst.shapeData.rect.width >> 1, obst.shapeData.rect.height >> 1);

			Vec2 lowerLeft = obst.center - halfWidthHeight;
			Vec2 upperRight = obst.center + halfWidthHeight;

			Vec2 upperLeft = upperRight;
			upperLeft.x -= halfWidthHeight.x;
			upperLeft.x -= halfWidthHeight.x;

			Vec2 lowerRight = lowerLeft;
			lowerRight.x += halfWidthHeight.x;
			lowerRight.x += halfWidthHeight.x;

			// Calculate paths from killer to points
			lowerLeft -= killerXY;
			upperRight -= killerXY;
			upperLeft -= killerXY;
			lowerRight -= killerXY;

			// TODO: DOUBLE CHECK is (-) RIGHT or LEFT?
			// 3. Do outer product with left and right bounds to see if they are within killer Cone of vision
			// Are any points BOTH to the RIGHT of left bound and LEFT of right bound?
			if ((leftBound.OuterProduct(Normal(lowerLeft)) > 0 &&
				rightBound.OuterProduct(Normal(lowerLeft)) < 0) ||

				(leftBound.OuterProduct(Normal(lowerRight)) > 0 &&
				rightBound.OuterProduct(Normal(lowerRight)) < 0) ||

				(leftBound.OuterProduct(Normal(upperLeft)) > 0 &&
				rightBound.OuterProduct(Normal(upperLeft)) < 0) ||

				(leftBound.OuterProduct(Normal(upperRight)) > 0 &&
				rightBound.OuterProduct(Normal(upperRight)) < 0)) {
				return true;
			}
		}
	}

	return false;
}

// HeatMapGenerator class (Main workhorse of the program)
void HeatMapGenerator::ReverseInt(int* x) {
	std::swap(((char*)x)[0], ((char*)x)[3]);
	std::swap(((char*)x)[1], ((char*)x)[2]);
}

int HeatMapGenerator::LoadData(const std::string &fKillsDataBase, const std::string &fLevelGeometry) {
	// 3. Parse the Kills files using binary stream
	// TEST PARSING KILL DATABASE FILE
	std::ifstream killsFile(fKillsDataBase, std::ios::binary); // open file in binary mode
	if (killsFile.fail()) { throw "BAD KILL BD FILENAME"; return false; }
	char first4Letters[4];
	killsFile.read(first4Letters, sizeof(int));
	bool bShouldReverseBytes = *(char*)first4Letters != *(char*)"ABCD";
	// TODO: Make a lookup table to assign the EXACT order of bytes that they want

	KillRecord currentRecord;
	const int sizetoRead = sizeof(KillRecord);
	char* startAddress = reinterpret_cast<char*>(&currentRecord);

	while (killsFile.read(startAddress, sizetoRead)) {
		// iterate through currentRecord struct and reverse each byte
		if (bShouldReverseBytes) {
			const int* end = (int*)&currentRecord + (sizeof(currentRecord) / sizeof(int));
			int* start = (int*)&currentRecord;

			for (; start < end; start++)
				ReverseInt(start);
		}

		records.push_back(currentRecord);
		numRecords++;
	}

	// Load Geometry File
	std::ifstream geoFile(fLevelGeometry, std::ios::binary); // open file in binary mode
	if (geoFile.fail()) { throw "BAD GEO FILENAME"; return false; }

	char first4Letters2[4];
	geoFile.read(first4Letters2, sizeof(int));
	bShouldReverseBytes = *(char*)first4Letters2 != *(char*)"ABCD";

	GeoObject currentObj;
	while (geoFile.read((char*)(&(currentObj.type)), sizeof(char))) {
		// 1. Determine size to read and read DIFFERENTLY accordingly
		currentObj.type &= 1;
		int sizetoRead = currentObj.type == CIRCLE ? 12 : 16;
		char* destAddress = (char*)&currentObj + sizeof(currentObj.type);
		geoFile.read(destAddress, sizetoRead);

		// 2. If necessary, iterate through all the REMAINING ints in GeoObject struct and reverse its bytes
		if (bShouldReverseBytes) {
			int* start = (int*)destAddress; 
			const int numIntsToSwitch = sizetoRead / (sizeof(int));
			const int* end = start + numIntsToSwitch; // NOTE - for now we IGNORE last 4 bytes if its a circle (just padding anyway!) //((sizeof(currentObj) - 1) / sizeof(int));

			for (; start < end; start++)
				ReverseInt(start);
		}

		// 3. Store the record in our list
		obstacles.push_back(currentObj);
		numObstacles++;
	}
}

void HeatMapGenerator::GenerateMapMatrix() {
	// 1. Define starting point in WORLD SPACE
	Vec2 heatMapOrigin_WS = Cell::MapSpace_To_WorldSpace(Vec2(0, 0));

	// 2. Definte cell size
	Vec2 cellSize(Cell::heatMapSize_WorldUnits.x / Cell::heatMapResolution.x,
						Cell::heatMapSize_WorldUnits.y / Cell::heatMapResolution.y);

	// 3. Populate matrix and increment as necessary
	cellMatrix.resize(Cell::heatMapResolution.y); // M Rows
	Vec2 cellPositionOffset(0, 0);

	for (std::vector<Cell>& row : cellMatrix) {
		row.resize(Cell::heatMapResolution.x); // N Columns

		for (Cell& cell : row) {
			cell.bottomLeft = (heatMapOrigin_WS + cellPositionOffset);
			cell.topRight = Vec2(heatMapOrigin_WS + cellPositionOffset) + cellSize;
			cellPositionOffset.x += cellSize.x;
		}

		cellPositionOffset.y += cellSize.y;
		cellPositionOffset.x = 0;
	}
}

void HeatMapGenerator::GenerateHeatMap() {
	for (KillRecord& record : records) {
		for (std::vector<Cell>& row : cellMatrix) {
			for (Cell& cell : row) {
				timeComplexity++;
				if (cell.Contains(record.victimXY)) {
					// 1. Count dead player
					deathToll++;
					cell.deadCount++;

					// 2. Count if facing
					if (record.DiedFacing()) {
						facingCount++;
						cell.facingCount++;
					}

					// 3. Count if in cover
					if (record.DiedObstructed(obstacles)) {
						coverCount++;
						cell.coverCount++;
					}
				}
			}
		}
	}
}

void HeatMapGenerator::PrintResults() {
	// Print totals (facing, and in cover)
	std::cout << facingCount << " " << coverCount << std::endl;

	// Print Death Toll HeatMap
	for (std::vector<Cell>& row : cellMatrix) {
		for (Cell& cell : row) {
			std::cout << cell.deadCount << ",";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	// Print Facing HeatMap
	for (std::vector<Cell>& row : cellMatrix) {
		for (Cell& cell : row) {
			std::cout << cell.facingCount << ",";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	// Print Cover HeatMap
	for (std::vector<Cell>& row : cellMatrix) {
		for (Cell& cell : row) {
			std::cout << cell.coverCount << ",";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

void HeatMapGenerator::Run() {
	// 1. Generate object matrix
	GenerateMapMatrix();

	// 2. Load Data
	if (!LoadData(fKillsDataBase, fLevelGeometry))
		std::cerr << "Failed to load data. Exiting...";

	// 3. Use it to generate Heatmap
	GenerateHeatMap();

	// 4. Print global stats and heatmap
	PrintResults();
}

HeatMapGenerator::HeatMapGenerator(const char* args[]) {
	// 1. Initialize global values
	// 2. Read all input arguments
	//std::string fKillsDataBase(args[0]);	// kills database
	//std::string fLevelGeometry(args[1]);	// level geometry
	//heatMapCenterWS.x = atoi(args[2]);			// map center x
	//heatMapCenterWS.y = atoi(args[3]);			// map center y
	//heatMapSize_WorldUnits.x = atoi(args[4]);			// map width
	//heatMapSize_WorldUnits.y = atoi(args[5]);			// map height
	//heatMapResolution.y = atoi(args[5]);		// resolution x
	//heatMapResolution.y = atoi(args[5]);		// resolution y

	// DEBUG MODE // DEBUG MODE // DEBUG MODE // DEBUG MODE // DEBUG MODE
	fKillsDataBase = "OutputData/DeathData_2/Kills_2.bin";	// kills database
	fLevelGeometry = "OutputData/DeathData_2/Buildings_2.bin";	// level geometry
	Cell::heatMapCenterWS.x = 25;				// map center x
	Cell::heatMapCenterWS.y = 25;				// map center y
	//Cell::heatMapSize_WorldUnits.x = 100;		// map width
	//Cell::heatMapSize_WorldUnits.y = 200;		// map height
	Cell::heatMapSize_WorldUnits.x = 20000;		// map width
	Cell::heatMapSize_WorldUnits.y = 30000;		// map height
	Cell::heatMapResolution.x = 5;			// resolution x
	Cell::heatMapResolution.y = 4;			// resolution y
}

// Program Entrypoint
int main(int numArgs, const char* args[])
{
	if (numArgs < 8) {
		//std::cerr << "Program requires exactly 8 arguments to run! Exiting...";
		//return -1;
	}

	HeatMapGenerator generator(args);
	generator.Run();

	std::cin.get();

	return 0;
}