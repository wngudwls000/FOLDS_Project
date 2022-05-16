#pragma once

#include "Node.h"
#include "Point.h"

class Cell{
private:
    Node node;
    Point point;
public:
    Cell(Point point, Node node): point(point), node(node){}
    Point getPoint() {return point;}
    Node getNode() {return node;}
};