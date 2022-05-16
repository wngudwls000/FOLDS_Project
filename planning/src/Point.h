#pragma once

#include "Node.h"

class Point{
private:
    int x,y;
    float key1;
    float key2;
public:
    Point(): x(inf), y(inf), key1(0), key2(0) {};
	Point(int x, int y) : x(x), y(y), key1(0), key2(0) {};
	int getX() { return x; }
	int getY() { return y; }
    void setX(int x) {this->x = x;}
    void setY(int y) {this->y = y;}
	float getKey1() { return key1; }
	float getKey2() { return key2; }

	void setKey1(float key_) { key1 = key_; }
	void setKey2(float key_) { key2 = key_; }
};
bool operator<(Point a,Point b) {
	if (a.getKey1() == b.getKey1())
		return a.getKey2() > b.getKey2();
	return a.getKey1() > b.getKey1();
}
