#ifndef SHAPE_HPP
#define SHAPE_HPP

#include <string>
#include <vector>

using std::string;
using std::vector;

class Shape {
public:
	int x;
	int y;
};

class Circle : public Shape {
public:
	int radius;
	
};

class Rect : public Shape {
public:
	int width;
	int height;
};

#endif