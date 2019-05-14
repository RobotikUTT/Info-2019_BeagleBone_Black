#include "pathfinder/map_storage.h"

using namespace std;

MapStorage::Vect2DBool MapStorage::buildAllowedPositions(int width, int height)
{
	for (unsigned int line = 0; line < height; line++)
	{
		this->allowedPos.emplace_back();
		for (unsigned int column = 0; column < width; column++)
			this->allowedPos[line].push_back(true);
	}
	
	ROS_DEBUG_STREAM("MapStorage: Done, map size is " << allowedPos.front().size() << "x" << allowedPos.size());
	return this->allowedPos;
}

MapStorage::Vect2DBool MapStorage::getAllowedPositions() const {
	return this->allowedPos;
}

int MapStorage::width() const {
	if (this->allowedPos.size() == 0) return 0;
	return this->allowedPos.front().size();
}

int MapStorage::height() const {
	return this->allowedPos.size();
}

bool MapStorage::isBlocked(int x, int y) const {
	if (!this->allowedPos[y][x]) {
		return true;
	}

	ros::Time now = ros::Time::now();
	for (auto& tmpshape : this->tempShapes) {
		// Wait 7 seconds before ignoring shape
		if (now - tmpshape.appearance < ros::Duration(7)) {
			if (this->isIn(tmpshape.shape, x, y)) {
				return true;
			}
		}
	}

	return false;
}

bool MapStorage::isIn(const ai_msgs::Shape& shape, int x, int y) const {
	if (shape.type == ai_msgs::Shape::RECT) {
		int width = shape.params[0];
		int height = shape.params[1];

		return x > (shape.x - width / 2) && x < (shape.x + width / 2) +
			y > (shape.y - height / 2) && y < (shape.y + height / 2);
	}
	
	// Apply circle
	else if (shape.type == ai_msgs::Shape::CIRCLE) {
		int dx = x - shape.x;
		int dy = y - shape.y;

		return dx * dx + dy * dy < shape.params[0];
	}

	return false;
}

void MapStorage::declareShape(ai_msgs::Shape shape, bool temporary) {
	if (!temporary) {
		applyShape(shape, this->allowedPos);
	} else {
		this->tempShapes.push_back(TemporaryShape(shape));
	}
}

void MapStorage::applyShape(ai_msgs::Shape& shape, MapStorage::Vect2DBool& grid) {
	// Apply centered rect
	if (shape.type == ai_msgs::Shape::RECT) {
		int width = shape.params[0];
		int height = shape.params[1];

		for (int x = shape.x - width / 2; x < width / 2 + shape.x; x ++) {
			for (int y = shape.y - height / 2; y < height / 2 + shape.y; y ++) {
				grid[y][x] = false;
			}
		}
	}
	
	// Apply circle
	else if (shape.type == ai_msgs::Shape::CIRCLE) {
		int radius = shape.params[0];

		for (int x = -radius; x < radius; x ++) {
			for (int y = -radius; y < radius; y ++) {
				// If x,y in circle
				if (x * x + y * y <= radius * radius) {
					grid[y + shape.y][x + shape.x] = false;
				}
			}
		}
	} else {
		ROS_ERROR_STREAM(shape.type << " shape type not recognized");
	}
}