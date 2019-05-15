#include "pathfinder/map_storage.h"

using namespace std;

MapStorage::MapStorage(std::shared_ptr<PosConvertor> convertor) : _convertor(convertor) {}

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

MapStorage::Vect2DBool& MapStorage::getAllowedPositions() {
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
	ROS_INFO_STREAM("INSERTING SHAPE");
	if (!temporary) {
		applyShape(shape, this->allowedPos);
	} else {
		this->tempShapes.push_back(TemporaryShape(shape));
	}
}

void MapStorage::applyShape(ai_msgs::Shape& shape, MapStorage::Vect2DBool& grid) {
	shape.x = this->_convertor->getInternalX(shape.x);
	shape.y = this->_convertor->getInternalY(shape.y);

	// Apply centered rect
	if (shape.type == ai_msgs::Shape::RECT) {
		if (shape.params.size() < 2) {
			ROS_ERROR_STREAM("Invalid rect shape received, missing height and width...");
			return;
		}

		int width = _convertor->getInternalX(shape.params[0]);
		int height = _convertor->getInternalY(shape.params[1]);

		for (int y = shape.y - height / 2; y < height / 2 + shape.y; y ++) {
			if (y < 0 || y >= grid.size()) continue;
			
			for (int x = shape.x - width / 2; x < width / 2 + shape.x; x ++) {
				if (x < 0 || x >= grid[y].size()) continue;
				grid[y][x] = false;
			}
		}
	}
	
	// Apply circle
	else if (shape.type == ai_msgs::Shape::CIRCLE) {
		if (shape.params.size() < 1) {
			ROS_ERROR_STREAM("Invalid circle shape received, missing radius...");
			return;
		}

		double radiusX = _convertor->getInternalX(shape.params[0]);
		double radiusY;
		
		// Use ellipse width/height
		if (shape.params.size() > 1) {
			radiusY = _convertor->getInternalY(shape.params[1]);
		} else { // or circle radius
			radiusY = _convertor->getInternalY(shape.params[0]);
		}

		for (int y = -radiusY; y < radiusY; y ++) {
			if (y + shape.y < 0 || y + shape.y >= grid.size()) continue;
			for (int x = -radiusX; x < radiusX; x ++) {
				if (x + shape.x < 0 || x + shape.x >= grid[y].size()) continue;
				// If x,y in ellipse
				if (x * x / (radiusX * radiusX) + y * y / (radiusY * radiusY) <= 1) {
					grid[y + shape.y][x + shape.x] = false;
				}
			}
		}
	} else {
		ROS_ERROR_STREAM(shape.type << " shape type not recognized");
	}
}

void MapStorage::display() const {
	std::stringstream res;
	for (int y = 0; y < this->allowedPos.size(); y ++) {
		res << "|";
		for (int x = 0; x < this->allowedPos[y].size(); x ++) {
			res << (this->allowedPos[y][x] ? " " : "X");
		}
		res << "|\n";
	}

	ROS_INFO_STREAM("[Map]\n" << res.str());
}