/*
 * STUDENT CODE - EDIT THIS FILE
 * Keep the class name and required interface.
 * Otherwise, add or change any data members or
 * member functions.
 */

#pragma once

#include "mothership.h"
#include "seafloor_survey.h"

// #include "examples/robot_selector.h" // EXAMPLE
#include "trees.h"
#include <vector>

struct robo_candidate
{
  int score; int sweepKey; int robot; 

  bool operator <(const robo_candidate &other) const
  {
    if (score != other.score) return score < other.score; 
    if (sweepKey != other.sweepKey) return sweepKey > other.sweepKey;

    return robot > other.robot; 
  }
}; 

class Search
{
public:
  explicit Search(Mothership &mothership); // REQUIRED

  SearchResult findShipwreck(); // REQUIRED

private:
  Mothership &mothership; // REQUIRED

  SeafloorSurvey seafloorSurvey; // OPTIONAL

  // int robotsExplored = 0; // EXAMPLE
  // RobotSelector robotSelector; // EXAMPLE

  // bool isReachable(int robot) const; // EXAMPLE

  std::vector<int> limit; 
  std::vector<int> queued_score; 
  std::vector<int> leadDepth; 
  std::vector<int> leadScore; 
  AVLSet<robo_candidate> frontier; 

  robo_candidate makeCandidate(int robot, int score) const; 
  void enqueue(int robot); 
  void dequeue(int robot);
  void refresh(int robot);  

  bool isPrimary(int robot) const;
  int scoreOf(int robot, int position) const; 
};
