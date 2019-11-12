#include "vex.h"

class RobotTracker{
  public:
    double startPositionOffsetMajor;
    double startPositionOffsetMinor;
    double startAngleOffsetClockwise;
    double closestEdgeDistance;
    double locx;
    double locy;
    double currentAngle;
    double fieldradius = 48;
    //Offsets are in relation to the centor of rotation
    void config(double SPOMajor, double SPOMinor, double SAOC, double CED){
      startPositionOffsetMajor = SPOMajor;
      startPositionOffsetMinor = SPOMinor;
      startAngleOffsetClockwise = angleConvertor(SAOC);
      closestEdgeDistance = CED;
      locx = -1*startPositionOffsetMajor;
      locy = -1*startPositionOffsetMinor;
      currentAngle = startAngleOffsetClockwise;
    }
    void updateAngle(double angle){
      currentAngle += angle;
    }
    void updateMovement(double distance){
      locx += cos(currentAngle)*distance;
      locy += sin(currentAngle)*distance;
    }
    double predictEdge(double distance){
      double theoreticalx = locx + cos(currentAngle)*distance;
      double theoreticaly = locy + sin(currentAngle)*distance;
      return fieldradius - sqrt(pow(theoreticalx,2) + pow(theoreticaly, 2));
    }
    double fromStart(){
      return edgeDistance() + fieldradius;
    }
    double edgeDistance(){
      return fieldradius - sqrt(pow(locx,2) + pow(locy, 2));
    }
  private:
    double angleConvertor(double angle){
      double newAngle = angle;
      if(angle >= 360){
        angle -= 360;
        newAngle = angleConvertor(angle);
      }
      else if(angle < 0){
        angle += 360;
        newAngle = angleConvertor(angle);
      }
      return newAngle;
    }
};