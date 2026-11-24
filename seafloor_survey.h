/*
 * STUDENT CODE - OPTIONAL - EDIT, REPLACE, OR DELETE
 */

#pragma once

#include <vector>

#include "mothership.h"

class SeafloorSurvey
{
public:
  explicit SeafloorSurvey(int robotcount); 
  void recordDebris(Location location); 

  // bool hasDebris() const; // EXAMPLE
  int getSignalStrength(Location location) const; 

private:
  int countInRange(int robot, int lo, int hi) const; 
  int robotcount; 
  std::vector<std::vector<int>> debrisbyrobot; 
};
