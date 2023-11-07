#pragma once

#include <vector>

#include "global.hpp"
#include "skin_matrix.hpp"

#define BGCHECK_Y_MIN -32000

// https://wiki.cloudmodding.com/oot/Collision_Mesh_Format and z64bgcheck.h
struct CollisionPoly {
  u16 type;
  // Vertex indices. The first 3 bits are flags, the rest are the index.
  // Vertex coordinates are integers and can be converted losslessly to floats.
  u16 v1;
  u16 v2;
  u16 v3;
  // Approximate normal vector. Component values -0x7FFF to 0x7FFF are mapped to
  // -1.0 to 1.0 with a resolution of about 0.000031.
  u16 nx;
  u16 ny;
  u16 nz;
  // Approximate plane distance to the origin along the normal vector. This is
  // the same for all points in the triangle.
  u16 dist;
};

struct SurfaceType {
  u32 data[2];
};

struct CamData {
  u16 setting;
  s16 count;
  Vec3s* bgCamFuncData;
};

struct WaterBox {
  s16 xMin;
  s16 ySurface;
  s16 zMin;
  s16 xLength;
  s16 zLength;
  u32 properties;
};

struct CollisionHeader {
  Vec3s minBound;
  Vec3s maxBound;
  u16 numVertices;
  Vec3s* vertices;
  u16 numPolys;
  CollisionPoly* polys;
  SurfaceType* surfaceTypeList;
  CamData* bgCamList;
  u16 numWaterBoxes;
  WaterBox* waterBoxes;
};

void CollisionPoly_GetVertices(CollisionPoly* poly, Vec3s* vtxList,
                               Vec3f* dest);
Vec3f CollisionPoly_GetNormalF(CollisionPoly* poly);

struct Dyna {
  f32 minY;
  f32 maxY;
  std::vector<Vec3s> vertices;
  std::vector<CollisionPoly> walls;
  std::vector<CollisionPoly> floors;
  std::vector<CollisionPoly> ceilings;

  CollisionHeader* header;
  Vec3s* vtxList;
};

// Simulates z_bgcheck.c for a subset of collision polygons.
struct Collision {
  std::vector<CollisionPoly*> walls;
  std::vector<CollisionPoly*> floors;
  std::vector<CollisionPoly*> ceilings;

  CollisionHeader* header;
  PlayerAge age;
  Vec3s* vtxList;
  CollisionPoly* polyList;

  std::vector<Dyna> dynas;

  // Empty collision
  Collision(CollisionHeader* header, PlayerAge age);
  // Adds all triangles with a vertex within the given bounds
  Collision(CollisionHeader* header, PlayerAge age, Vec3f min, Vec3f max);

  // Prints collision polygons
  void printPolys();

  // Add a poly
  void addPoly(int polyId);

  void addDynapoly(CollisionHeader* header, Vec3f scale, Vec3s rot, Vec3f pos);

  // TODO: these functions need better names and more consistency

  // Run wall and ceiling checks, displacing the intended position
  Vec3f runChecks(Vec3f prevPos, Vec3f intendedPos, CollisionPoly** wallPoly,
                  CollisionPoly** floorPoly, int* dynaId, f32* floorHeight);
  Vec3f runChecks(Vec3f prevPos, Vec3f intendedPos);
  // Snap down to the nearest floor below the given position
  Vec3f findFloor(Vec3f pos, CollisionPoly** outPoly, int* dynaId);
  Vec3f findFloor(Vec3f pos);

  // Run line test for entities
  Vec3f entityLineTest(Vec3f pos, Vec3f target, bool checkWalls,
                       bool checkFloors, bool checkCeilings,
                       CollisionPoly** outPoly);
  // Run line test for camera
  Vec3f cameraLineTest(Vec3f pos, Vec3f target, CollisionPoly** outPoly);
  // Find floor for camera
  f32 cameraFindFloor(Vec3f pos, CollisionPoly** outPoly);

  // Get camera setting for floor poly
  int getCameraSetting(CollisionPoly* poly, int dynaId);
};
