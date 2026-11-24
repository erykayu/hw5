/*
 * STUDENT CODE - OPTIONAL - EDIT, REPLACE, OR DELETE
 */

#include "seafloor_survey.h"
#include <algorithm> 

SeafloorSurvey::SeafloorSurvey(int robotcount): robotcount(robotcount), debrisbyrobot(robotcount)
{
}

void SeafloorSurvey::recordDebris(Location location)
{
  std::vector<int> &debris = debrisbyrobot[location.robot];
  if (debris.empty() || debris.back() < location.segment)
  {
    debris.push_back(location.segment); 
  }
}

int SeafloorSurvey::getSignalStrength(Location location) const
{
  const int lo = location.segment - SIGNAL_RANGE; 
  const int hi = location.segment + SIGNAL_RANGE;

  const int robot = location.robot; 
  const int left = (robot + robotcount - 1) % robotcount; 
  const int right = (robot + 1) % robotcount; 

  int total = countInRange(robot, lo, hi);
  if (left != robot) total += countInRange(left, lo, hi); 
  if (right != robot &&right != left) total+= countInRange(right, lo, hi);

  return total; 
}

int SeafloorSurvey::countInRange(int robot, int lo, int hi) const
{
  const std::vector<int> &debris = debrisbyrobot[robot];
  return static_cast<int>(std::upper_bound(debris.begin(), debris.end(), hi) - std::lower_bound(debris.begin(), debris.end(), lo));
}

/* EXAMPLE
bool SeafloorSurvey::hasDebris() const
{
  return !debrisLocations.empty();
}
*/