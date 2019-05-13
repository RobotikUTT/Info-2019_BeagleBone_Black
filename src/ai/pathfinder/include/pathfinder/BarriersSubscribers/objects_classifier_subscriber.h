#ifndef objects_classifier_SUBSCRIBER
#define objects_classifier_SUBSCRIBER

#include "abstract_barriers_subscriber.h"


#include <vector>

class Rectangle {

}

class Circle {

}

namespace Recognition
{
    class ObjectsClassifierSubscriber : public AbstractBarriersSubscriber
    {
    public:
        ObjectsClassifierSubscriber(const double& safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
    private:
        std::vector<Rectangle> lastRects;
        std::vector<Circle> lastCircles;
        
        void addRects(const std::vector<Rectangle>& rects);
        void addCircles(const std::vector<Circle>& circs);
        
        bool isInsideRect(const Rectangle& rect, const geometry_msgs::Pose2D& pos) const;
        bool isInsideCircle(const Circle& circ, const geometry_msgs::Pose2D& pos) const;
        
        double scalarProduct(std::pair<double, double> vA, std::pair<double, double> vB) const;
        double vectorProduct(std::pair<double, double> vA, std::pair<double, double> vB) const;
    };
} // namespace Recognition

#endif // objects_classifier_SUBSCRIBER
