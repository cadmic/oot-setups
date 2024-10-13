#pragma once

#include <cstdio>
#include <ctime>
#include <limits>

#include "global.hpp"
#include "pos_angle_setup.hpp"

// To search an angle range spanning 0, use e.g. -0x2000 to 0x2000.
struct PosAngleRange {
  int angleMin = 0x0000;
  int angleMax = 0x0000;
  int angleStep = 0x10;
  f32 xMin = 0.0f;
  f32 xMax = 0.0f;
  f32 xStep = 0.1f;
  f32 zMin = 0.0f;
  f32 zMax = 0.0f;
  f32 zStep = 0.1f;
};

// Search a 2d position and angle space while printing status information to
// stderr. The function `f` should have the signature `bool f(u16 angle, f32 x,
// f32 z)`. It will be called for each point in the search space, and it should
// return true if the search found an interesting result (used for status
// printing only).
template <typename F>
void searchPosAngleRange(PosAngleRange range, F f) {
  unsigned long long tested = 0;
  unsigned long long found = 0;
  int angleMinFound = std::numeric_limits<int>::max();
  int angleMaxFound = std::numeric_limits<int>::lowest();
  f32 xMinFound = std::numeric_limits<f32>::max();
  f32 xMaxFound = std::numeric_limits<f32>::lowest();
  f32 zMinFound = std::numeric_limits<f32>::max();
  f32 zMaxFound = std::numeric_limits<f32>::lowest();
  time_t lastPrint = time(nullptr);

  for (int angle = range.angleMin; angle <= range.angleMax;
       angle += range.angleStep) {
    for (f32 x = range.xMin; x <= range.xMax; x += range.xStep) {
      for (f32 z = range.zMin; z <= range.zMax; z += range.zStep) {
        time_t now = time(nullptr);
        if (now - lastPrint >= 1) {
          lastPrint = now;
          fprintf(stderr, "tested:%llu found:%llu angle:%04x x:%9.3f z:%9.3f",
                  tested, found, (u16)angle, x, z);
          if (found > 0) {
            fprintf(stderr,
                    " amin:%04x amax:%04x xmin:%9.3f"
                    " xmax:%9.3f zmin:%9.3f zmax:%9.3f",
                    (u16)angleMinFound, (u16)angleMaxFound, xMinFound,
                    xMaxFound, zMinFound, zMaxFound);
          }
          fprintf(stderr, " ...\r");
        }

        tested++;
        if (f(angle, x, z)) {
          found++;
          angleMinFound = std::min(angleMinFound, angle);
          angleMaxFound = std::max(angleMaxFound, angle);
          xMinFound = std::min(xMinFound, x);
          xMaxFound = std::max(xMaxFound, x);
          zMinFound = std::min(zMinFound, z);
          zMaxFound = std::max(zMaxFound, z);
        }
      }
    }
  }

  fprintf(stderr, "tested:%llu found:%llu", tested, found);
  if (found > 0) {
    fprintf(stderr,
            " amin:%04x amax:%04x xmin:%9.3f"
            " xmax:%9.3f zmin:%9.3f zmax:%9.3f",
            (u16)angleMinFound, (u16)angleMaxFound, xMinFound, xMaxFound,
            zMinFound, zMaxFound);
  }
  fprintf(stderr, "\n");
}

struct SearchParams {
  // Collision to use for setups.
  Collision* col;
  // Bounds for position during the setup.
  Vec3f minBounds;
  Vec3f maxBounds;
  // Colliders that could push Link.
  std::vector<Collider> colliders;
  // List of initial positions and angles.
  std::vector<std::pair<Vec3f, u16>> starts;
  // Maximum setup cost (in order to limit search space).
  int maxCost = 50;
  // Final angle bounds (inclusive). Use e.g. angleMin=0xe000, angleMax=0x2000
  // for an angle range crossing 0.
  u16 angleMin = 0x0000;
  u16 angleMax = 0xffff;
  // Final position bounds.
  f32 xMin = -10000.0f;
  f32 xMax = 10000.0f;
  f32 zMin = -10000.0f;
  f32 zMax = 10000.0f;
  // Possible actions to choose from.
  std::vector<Action> actions;
};

// DFS-based setup search. Prints statistics to stderr. Output should be a
// function with the following signature:
//
// bool f(
//     Vec3f initialPos,
//     u16 initialAngle,
//     const PosAngleSetup& setup,
//     const std::vector<Action>& actions,
//     int cost);
//
// that is called for each
// setup that reaches the goal area. It should return true if the setup was
// successful (for statistics only).
template <typename Output>
void searchSetups(const SearchParams& params, Output output);

// Like above, but with an additional filter function with the signature
//
// bool f(
//     Vec3f initialPos,
//     u16 initialAngle,
//     const PosAngleSetup& setup,
//     const std::vector<Action>& actions,
//     int cost);
//
// that returns false if the setup so far should be pruned.
template <typename Filter, typename Output>
void searchSetups(const SearchParams& params, Filter filter, Output output);

// Returns the number of search shards for the given depth. There's one shard
// for each initial position and setup path up to the given depth.
int numShards(const SearchParams& params, int depth);

// Searches a single shard. The shard index is in the range [0, numShards).
template <typename Output>
void searchSetupsShard(const SearchParams& params, int depth, int shard,
                       Output output);

// Like above, but with a filter function.
template <typename Filter, typename Output>
void searchSetupsShard(const SearchParams& params, int depth, int shard,
                       Filter filter, Output output);

// Implementation details below

struct SearchState {
  unsigned long long tested = 0;  // Total number of nodes visited.
  unsigned long long close = 0;   // Number of nodes that reached the goal area.
  unsigned long long found = 0;   // Number of nodes that were successful.
  time_t lastPrint;               // Time of last statistics print.
  int startIndex;                 // Start index for current search.
  Vec3f startPos;                 // Start position for current search.
  u16 startAngle;                 // Start angle for current search.
  std::vector<Action> path;       // Current path.
  std::vector<Action> startActions;  // Start actions for all search paths.
};

template <typename Filter, typename Output>
void doSearch(const SearchParams& params, SearchState* state,
              const PosAngleSetup& setup, int cost, Filter filter,
              Output output) {
  time_t now = time(nullptr);
  if (now - state->lastPrint >= 1) {
    state->lastPrint = now;
    fprintf(stderr,
            "tested=%llu close=%llu found=%llu start=%d actions=%s ...\n",
            state->tested, state->close, state->found, state->startIndex,
            actionNames(state->path).c_str());
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

  // Estimate minimum cost to reach goal area. A sidehop moves 12.75 units per
  // frame.
  // TODO: estimate based on angle too?
  int minCostToGoal = ceilf(sqrtf(SQ(xDist) + SQ(zDist)) / 12.75f);
  if (cost + minCostToGoal > params.maxCost) {
    return;
  }

  if (!filter(state->startPos, state->startAngle, setup, state->path, cost)) {
    return;
  }

  state->tested++;
  if (inAngleRange && xDist == 0.0f && zDist == 0.0f) {
    state->close++;
    if (output(state->startPos, state->startAngle, setup, state->path, cost)) {
      state->found++;
    }
  }

  int k = state->path.size();
  for (Action action : params.actions) {
    if (k < state->startActions.size() && action != state->startActions[k]) {
      continue;
    }

    // TODO: generalize this and record entire position/angle history?
    if (k > 0 && ((action == ROTATE_ESS_LEFT &&
                   state->path.back() == ROTATE_ESS_RIGHT) ||
                  (action == ROTATE_ESS_RIGHT &&
                   state->path.back() == ROTATE_ESS_LEFT))) {
      continue;
    }

    int newCost = cost + (k > 0 ? actionCost(state->path.back(), action)
                                : actionCost(action));
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

    state->path.push_back(action);
    doSearch(params, state, newSetup, newCost, filter, output);
    state->path.pop_back();
  }
}

template <typename Filter, typename Output>
void searchSetups(const SearchParams& params, Filter filter, Output output) {
  SearchState state;
  state.lastPrint = time(nullptr);
  state.path.reserve(params.maxCost);

  for (int i = 0; i < params.starts.size(); i++) {
    state.startIndex = i;
    state.startPos = params.starts[i].first;
    state.startAngle = params.starts[i].second;

    PosAngleSetup setup(params.col, state.startPos, state.startAngle,
                        params.minBounds, params.maxBounds);
    for (const Collider& c : params.colliders) {
      setup.addCollider(c);
    }
    doSearch(params, &state, setup, 0, filter, output);
  }

  fprintf(stderr, "tested=%llu close=%llu found=%llu\n", state.tested,
          state.close, state.found);
}

template <typename Output>
void searchSetups(const SearchParams& params, Output output) {
  auto filter = [](Vec3f, u16, const PosAngleSetup&, const std::vector<Action>&,
                   int) { return true; };
  searchSetups(params, filter, output);
}

inline int numShards(const SearchParams& params, int depth) {
  int numShards = params.starts.size();
  for (int i = 0; i < depth; i++) {
    numShards *= params.actions.size();
  }
  return numShards;
}

template <typename Filter, typename Output>
void searchSetupsShard(const SearchParams& params, int depth, int shard,
                       Filter filter, Output output) {
  SearchState state;
  state.lastPrint = time(nullptr);
  state.path.reserve(params.maxCost);

  int n = shard;
  int numActions = params.actions.size();
  for (int i = 0; i < depth; i++) {
    state.startActions.push_back(params.actions[n % numActions]);
    n /= numActions;
  }
  std::reverse(state.startActions.begin(), state.startActions.end());

  if (n >= params.starts.size()) {
    fprintf(stderr, "shard %d must be less than %d\n", shard,
            numShards(params, depth));
    return;
  }

  int startIndex = n;
  state.startIndex = startIndex;
  state.startPos = params.starts[startIndex].first;
  state.startAngle = params.starts[startIndex].second;

  PosAngleSetup setup(params.col, state.startPos, state.startAngle,
                      params.minBounds, params.maxBounds);
  for (const Collider& c : params.colliders) {
    setup.addCollider(c);
  }
  doSearch(params, &state, setup, 0, filter, output);

  fprintf(stderr, "tested=%llu close=%llu found=%llu shard=%d\n", state.tested,
          state.close, state.found, shard);
}

template <typename Output>
void searchSetupsShard(const SearchParams& params, int depth, int shard,
                       Output output) {
  auto filter = [](Vec3f, u16, const PosAngleSetup&, const std::vector<Action>&,
                   int) { return true; };
  searchSetupsShard(params, depth, shard, filter, output);
}
