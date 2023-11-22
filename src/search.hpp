#pragma once

#include "pos_angle_setup.hpp"

struct SearchParams {
  // Maximum setup cost (in order to limit search space).
  int maxCost;
  // Final angle bounds (inclusive). Use e.g. angleMin=0xe000, angleMax=0x2000
  // for an angle range crossing 0.
  u16 angleMin;
  u16 angleMax;
  // Final position bounds.
  f32 xMin;
  f32 xMax;
  f32 zMin;
  f32 zMax;
  // Possible actions to choose from.
  std::vector<Action> actions;
};

// DFS-based setup search. Prints statistics to stderr. Output should be a
// function `bool f(const PosAngleSetup& setup, const std::vector<Action>&
// actions, int cost)` that is called for each setup that reaches the goal area.
// It should return true if the setup was successful (for statistics only).
template <typename Output>
void searchSetups(const SearchParams& params, const PosAngleSetup& start,
                  Output output);

// Like above, but with an additional filter function `bool f(const
// PosAngleSetup& setup)` that returns false if the setup so far should be
// pruned.
template <typename Filter, typename Output>
void searchSetups(const SearchParams& params, const PosAngleSetup& start,
                  Filter filter, Output output);

// Implementation details below

struct SearchStats {
  // Total number of nodes visited.
  unsigned long long tested = 0;
  // Number of nodes that reached the goal area.
  unsigned long long close = 0;
  // Number of nodes that reached the goal area and were successful.
  unsigned long long found = 0;
  // Time of last statistics print.
  time_t lastPrint;
};

template <typename Filter, typename Output>
void doSearch(const SearchParams& params, const PosAngleSetup& setup,
              Filter filter, Output output, SearchStats* stats,
              std::vector<Action>* path, int cost) {
  time_t now = time(nullptr);
  if (now - stats->lastPrint >= 1) {
    stats->lastPrint = now;
    fprintf(stderr, "tested=%llu close=%llu found=%llu actions=", stats->tested,
            stats->close, stats->found);
    for (Action action : *path) {
      fprintf(stderr, "%s,", actionName(action));
    }
    fprintf(stderr, "...\n");
  }

  Vec3f pos = setup.pos;
  u16 angle = setup.angle;

  bool inAngleRange;
  if (params.angleMin <= params.angleMax) {
    inAngleRange = params.angleMin <= angle && angle <= params.angleMax;
  } else {
    inAngleRange = params.angleMin <= angle || angle <= params.angleMax;
  }

  f32 xDist;
  if (pos.x < params.xMin) {
    xDist = params.xMin - pos.x;
  } else if (pos.x > params.xMax) {
    xDist = pos.x - params.xMax;
  } else {
    xDist = 0.0f;
  }

  f32 zDist;
  if (pos.z < params.zMin) {
    zDist = params.zMin - pos.z;
  } else if (pos.z > params.zMax) {
    zDist = pos.z - params.zMax;
  } else {
    zDist = 0.0f;
  }

  stats->tested++;
  if (inAngleRange && xDist == 0.0f && zDist == 0.0f) {
    stats->close++;
    if (output(setup, *path, cost)) {
      stats->found++;
    }
  }

  // Estimate minimum cost to reach goal area. A sidehop moves 12.75 units per
  // frame.
  // TODO: estimate based on angle too?
  int minCostToGoal = ceilf(sqrtf(SQ(xDist) + SQ(zDist)) / 12.75f);
  if (cost + minCostToGoal > params.maxCost) {
    return;
  }

  if (!filter(setup)) {
    return;
  }

  for (Action action : params.actions) {
    // TODO: generalize this and record entire position/angle history?
    if (!path->empty() &&
        ((action == TURN_ESS_LEFT && path->back() == TURN_ESS_RIGHT) ||
         (action == TURN_ESS_RIGHT && path->back() == TURN_ESS_LEFT))) {
      continue;
    }

    int newCost = cost + actionCost(action);
    if (newCost > params.maxCost) {
      continue;
    }

    PosAngleSetup newSetup(setup);
    if (!newSetup.performAction(action)) {
      continue;
    }

    if (newSetup.pos == pos && newSetup.angle == angle) {
      continue;
    }

    path->push_back(action);
    doSearch(params, newSetup, filter, output, stats, path, newCost);
    path->pop_back();
  }
}

template <typename Filter, typename Output>
void searchSetups(const SearchParams& params, const PosAngleSetup& start,
                  Filter filter, Output output) {
  SearchStats stats;
  stats.lastPrint = time(nullptr);

  std::vector<Action> path;
  path.reserve(params.maxCost);

  doSearch(params, start, filter, output, &stats, &path, 0);
}

template <typename Output>
void searchSetups(const SearchParams& params, const PosAngleSetup& start,
                  Output output) {
  searchSetups(
      params, start, [](const PosAngleSetup&) { return true; }, output);
}
