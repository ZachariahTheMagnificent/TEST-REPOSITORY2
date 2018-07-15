#pragma once

#include "HMMath.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

struct Cell {
	// Heatmap coordinates start at its BOTTOM LEFT CORNER
	// Positive X goes RIGHT
	// Positive Y goes UP
	static Vec2 heatMapCenterWS;
	static Vec2 heatMapSize_WorldUnits;
	static Vec2 heatMapResolution;

	Vec2 bottomLeft, topRight;
	int deadCount = 0, facingCount = 0, coverCount = 0;

	// TODO: use winding order
	bool Contains(const Vec2& playerLocation) {
		return	playerLocation >= bottomLeft &&
			playerLocation < topRight;
	}

	// HeatMap origin is defined as BOTTOM LEFT corner of map!
	static Vec2 WorldSpace_To_MapSpace(const Vec2 wsCoords) {
		// bring coords into local map space, then bring them relative to the TOP LEFT of the map
		return (wsCoords - heatMapCenterWS)
			+ Vec2(heatMapSize_WorldUnits.x >> 1, heatMapSize_WorldUnits.y >> 1);
	}

	static Vec2 MapSpace_To_WorldSpace(const Vec2 wsCoords) {
		// bring coords relative to the TOP LEFT of the map, then bring them into local map space
		return (wsCoords - Vec2(heatMapSize_WorldUnits.x >> 1, heatMapSize_WorldUnits.y >> 1))
			+ heatMapCenterWS;
	}
};

struct GeoObject {
	// NOTE - made type an int to ENSURE size at compile time
	int type = CIRCLE; // 0 = circle, 1 = rectangle
	Vec2 center;

	union {
		struct {
			int radius;
			int padding;
		} circ;

		struct {
			int width, height;
		} rect;
	} shapeData;

	GeoObject()=default;
};

struct KillRecord {
	KillRecord() = default;

	Vec2 killerXY;
	int killerRotation;

	Vec2 victimXY;
	int victimRotation;
	int victimLifeTime;

	bool DiedFacing();
	bool DiedObstructed(std::vector<GeoObject>& obstacles);
};

class HeatMapGenerator {
public:
	HeatMapGenerator() = default;
	HeatMapGenerator(const char* args[]);
	void Run();
	int LoadData(const std::string &fKillsDataBase, const std::string &fLevelGeometry);
	void GenerateMapMatrix();
	void GenerateHeatMap();
	void PrintResults();
	void ReverseInt(int* x);

// GLOBALS
	std::vector<KillRecord> records;
	std::vector<GeoObject> obstacles;
	std::vector<std::vector<Cell>> cellMatrix;
	int numRecords = 0, numObstacles = 0;
	int deathToll = 0, facingCount = 0, coverCount = 0;

	// DEBUG MODE // DEBUG MODE // DEBUG MODE // DEBUG MODE // DEBUG MODE
	int timeComplexity = 0;
	std::string fKillsDataBase;
	std::string fLevelGeometry;
};