#pragma once

#include "Node.h"
#include "Point.h"

class Map {
private:
	int xlength;
	int ylength;
	Node** map;
public:
	Map(int xlength, int ylength): xlength(xlength), ylength(ylength) {
		map = new Node * [xlength];
		for (int i = 0; i < xlength; i++)
			map[i] = new Node[ylength];
	}
	~Map() {
		for (int i = 0; i < xlength; i++)
			delete[] map[i];
		delete[] map;
	}
	Node** getMap() { return map; }
};