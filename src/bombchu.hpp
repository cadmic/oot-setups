#pragma once

#include "collision.hpp"
#include "global.hpp"

// Bombchu path simulation
struct Bombchu {
  Collision* col;
  Vec3f pos;
  Vec3s rot;
  Vec3f axisForwards;
  Vec3f axisUp;
  Vec3f axisLeft;
  CollisionPoly* floorPoly;

  // Initialize a bombchu at the given position and angle
  Bombchu(Collision* col, Vec3f pos, u16 angle);

  // Update the bombchu for one frame, returning false if it should explode
  // prematurely
  bool move();

 private:
  void updateFloorPoly(CollisionPoly* poly);
  bool lineTest(Vec3f posA, Vec3f posB, Vec3f* outPos, CollisionPoly** outPoly);
};
