/*
 * STUDENT CODE - EDIT THIS FILE
 */

#include "search.h"

#include <algorithm>

// define namespace

namespace{
  constexpr int sweep_chunk = 2000; 
  constexpr int switch_margin = 4; 
  constexpr int signal_threshold = 8; 
  constexpr int secondary_delay = 15000; 
}

Search::Search(Mothership &mothership)
    : mothership(mothership), seafloorSurvey(mothership.getRobotCount())
// , robotSelector(mothership, seafloorSurvey) /* EXAMPLE
{
}

bool Search::isPrimary(int robot) const
{
  return robot % 3 == 1 || robot == mothership.getRobotCount() - 1; 
}

int Search::scoreOf(int robot, int position) const
{
  int score = seafloorSurvey.getSignalStrength(Location{robot, position});
  if (position < leadDepth[robot] - SIGNAL_RANGE && leadScore[robot] > score) score = leadScore[robot];
  return score >= signal_threshold ? score : 0; 
}

robo_candidate Search::makeCandidate(int robot, int score) const
{
  const int position = mothership.getRobotPosition(robot);
  const int sweepKey = position + (isPrimary(robot) ? 0 :secondary_delay);
  return robo_candidate{score, sweepKey, robot};
}

// add enqueue dequeue

void Search::enqueue(int robot)
{
  const int score = scoreOf(robot, mothership.getRobotPosition(robot));
  queued_score[robot] = score; 
  frontier.insert(makeCandidate(robot, score)); 
}

void Search::dequeue(int robot)
{
  if (queued_score[robot] < 0) return; 

  frontier.erase(makeCandidate(robot, queued_score[robot]));
  queued_score[robot] = -1; 
}

void Search::refresh(int robot)
{
  if (queued_score[robot] >= 0) 
  {
    dequeue(robot);
    enqueue(robot);
  }
}

SearchResult Search::findShipwreck()
{
  /* TOY APPROACH - Depth First Search - (CHANGE OR REPLACE) */
  const int n = mothership.getRobotCount(); 

  // get the safe limit for each robot
  limit.assign(n, TETHER_LENGTH);
  for (const Location &abyss : mothership.getAbysses())
  {
    if (abyss.robot >= 0 && abyss.robot < n) limit[abyss.robot] = std::min(limit[abyss.robot], abyss.segment-1); 
  }

  queued_score.assign(n, -1);
  leadScore.assign(n, 0);
  leadDepth.assign(n, 0); 
  for (int robot = 0; robot < n; ++robot)
  {
    if (mothership.getRobotPosition(robot) < limit[robot]) enqueue(robot); 
  }

  int active = -1; int drivensinceswitch = 0; 

  while (true)
  {
    if (active < 0)
    {
      if (frontier.size() == 0) return SearchResult{false, Location{}}; 
      const int next = frontier.rbegin() -> robot; 
      dequeue(next); 
      if (next != mothership.getActiveRobot()) mothership.switchToRobot(next); 
      active = next; drivensinceswitch = 0; 
    }

    const int before = mothership.getRobotPosition(active); 
    const int step = std::min(MAX_SEGMENTS_PER_DRIVE, limit[active] - before);
    const DriveResult result = mothership.driveActiveRobot(step);
    const int position = mothership.getRobotPosition(active); 
    drivensinceswitch += result.segmentsMoved; 

    if (result.reason == DriveStopReason::ShipwreckFound) return SearchResult{true, Location{active, position}}; 
    
    if (result.reason == DriveStopReason::DebrisFound)
    {
      seafloorSurvey.recordDebris(Location{active, position});

      const int left = (active + n - 1) % n; 
      const int right = (active + 1) % n; 
      const int signal = seafloorSurvey.getSignalStrength(Location{active, position}); 

      if (signal >= signal_threshold)
      {
        if (left != active)
        {
          leadScore[left] = std::max(leadScore[left], signal);
          leadDepth[left] = std::max(leadDepth[left], position); 
        }
        if (right != active && right != left)
        {
          leadScore[right] = std::max(leadScore[right], signal);
          leadDepth[right] = std::max(leadDepth[right], position); 
        }
      }

      if (left != active) refresh(left); 
      if (right != active && right != left) refresh(right); 
    } 
    
    if (position >= limit[active])
    {
      active = -1;
      continue; 
    }

    if (frontier.size() == 0) continue; 

    const int mine = scoreOf(active, position);
    const int best = frontier.rbegin()->score; 

    bool leave = false; 
    if (best > 0 && best > mine + switch_margin) leave = true; //find. abetter lead
    else if (mine == 0 && best == 0) leave = drivensinceswitch >= sweep_chunk; 

    if (leave) 
    {
      enqueue(active); active = -1; 
    }
  }
}
