#include "collision.hpp"

#include <algorithm>

#include "global.hpp"
#include "skin_matrix.hpp"
#include "sys_math3d.hpp"

#define COLPOLY_NORMAL_FRAC (1.0f / SHT_MAX)
#define COLPOLY_SNORMAL(x) ()

void CollisionPoly_GetVertices(CollisionPoly* poly, Vec3s* vtxList,
                               Vec3f* dest) {
  dest[0] = vtxList[poly->v1 & 0x1FFF];
  dest[1] = vtxList[poly->v2 & 0x1FFF];
  dest[2] = vtxList[poly->v3 & 0x1FFF];
}

Vec3f CollisionPoly_GetNormalF(CollisionPoly* poly) {
  return {
      (s16)poly->nx * COLPOLY_NORMAL_FRAC,
      (s16)poly->ny * COLPOLY_NORMAL_FRAC,
      (s16)poly->nz * COLPOLY_NORMAL_FRAC,
  };
}

f32 CollisionPoly_GetMinY(CollisionPoly* poly, Vec3s* vtxList) {
  Vec3f polyVerts[3];
  CollisionPoly_GetVertices(poly, vtxList, polyVerts);

  //! @bug Due to rounding errors, some polys with a slight slope have a y
  //! normal of 1.0f/-1.0f. As such, this optimization returns the wrong minimum
  //! y for a subset of these polys.
  if ((s16)poly->ny == 32767 || (s16)poly->ny == -32767) {
    return polyVerts[0].y;
  }

  return std::min(std::min(polyVerts[0].y, polyVerts[1].y), polyVerts[2].y);
}

s32 CollisionPoly_LineVsPoly(CollisionPoly* poly, Vec3s* vtxList, Vec3f posA,
                             Vec3f posB, Vec3f* planeIntersect) {
  f32 planeDistA = ((s16)poly->nx * posA.x + (s16)poly->ny * posA.y +
                    (s16)poly->nz * posA.z) *
                       COLPOLY_NORMAL_FRAC +
                   (s16)poly->dist;
  f32 planeDistB = ((s16)poly->nx * posB.x + (s16)poly->ny * posB.y +
                    (s16)poly->nz * posB.z) *
                       COLPOLY_NORMAL_FRAC +
                   (s16)poly->dist;
  f32 planeDistDelta = planeDistA - planeDistB;
  if ((planeDistA >= 0.0f && planeDistB >= 0.0f) ||
      (planeDistA < 0.0f && planeDistB < 0.0f) ||
      (planeDistA < 0.0f && planeDistB > 0.0f) || IS_ZERO(planeDistDelta)) {
    return false;
  }

  Vec3f polyVerts[3];
  CollisionPoly_GetVertices(poly, vtxList, polyVerts);

  Plane plane;
  plane.originDist = (s16)poly->dist;
  plane.normal = CollisionPoly_GetNormalF(poly);

  Math3D_LineSplitRatio(&posA, &posB, planeDistA / planeDistDelta,
                        planeIntersect);

  return (fabsf(plane.normal.x) > 0.5f &&
          Math3D_TriChkPointParaXDist(&polyVerts[0], &polyVerts[1],
                                      &polyVerts[2], &plane, planeIntersect->y,
                                      planeIntersect->z, 1.0f)) ||
         (fabsf(plane.normal.y) > 0.5f &&
          Math3D_TriChkPointParaYDist(&polyVerts[0], &polyVerts[1],
                                      &polyVerts[2], &plane, planeIntersect->z,
                                      planeIntersect->x, 1.0f)) ||
         (fabsf(plane.normal.z) > 0.5f &&
          Math3D_TriChkLineSegParaZDist(
              &polyVerts[0], &polyVerts[1], &polyVerts[2], &plane,
              planeIntersect->x, planeIntersect->y, 1.0f));
}

u32 SurfaceType_GetData(Collision* col, CollisionPoly* poly, s32 dataIdx) {
  return col->header->surfaceTypeList[poly->type].data[dataIdx];
}

u32 SurfaceType_IsSoft(Collision* col, CollisionPoly* poly) {
  return SurfaceType_GetData(col, poly, 0) >> 30 & 1;
}

void BgCheck_ComputeWallDisplacement(CollisionPoly* poly, f32* posX, f32* posZ,
                                     Vec3f normal, f32 invXZlength,
                                     f32 planeDist, f32 radius) {
  f32 displacement = (radius - planeDist) * invXZlength;

  *posX += displacement * normal.x;
  *posZ += displacement * normal.z;
}

bool BgCheck_CheckLineAgainstList(Collision* col,
                                  std::vector<CollisionPoly*>* polys,
                                  Vec3f posA, Vec3f* posB, f32* minDistSq,
                                  CollisionPoly** outPoly) {
  bool result = false;
  Vec3f posIntersect;

  for (CollisionPoly* poly : *polys) {
    // TODO: sort polys by min Y
    f32 minY = CollisionPoly_GetMinY(poly, col->vtxList);
    if (posA.y < minY && posB->y < minY) {
      continue;
    }

    if (CollisionPoly_LineVsPoly(poly, col->vtxList, posA, *posB,
                                 &posIntersect)) {
      f32 distSq = Math3D_Vec3fDistSq(&posA, &posIntersect);
      if (distSq < *minDistSq) {
        *minDistSq = distSq;
        *posB = posIntersect;
        *outPoly = poly;
        result = true;
      }
    }
  }
  return result;
}

bool BgCheck_CheckLineAgainstDynaList(Collision* col, Dyna* dyna,
                                      std::vector<CollisionPoly*>* polys,
                                      Vec3f posA, Vec3f* posB, f32* minDistSq,
                                      CollisionPoly** outPoly) {
  bool result = false;
  for (CollisionPoly* poly : *polys) {
    Vec3f polyIntersect;
    if (CollisionPoly_LineVsPoly(poly, dyna->vertices.data(), posA, *posB,
                                 &polyIntersect)) {
      f32 distSq = Math3D_Vec3fDistSq(&posA, &polyIntersect);
      if (distSq < *minDistSq) {
        *minDistSq = distSq;
        *posB = polyIntersect;
        *outPoly = poly;
        result = true;
      }
    }
  }
  return result;
}

bool BgCheck_CheckLineImpl(Collision* col, Vec3f posPrev, Vec3f posNext,
                           bool checkWalls, bool checkFloors,
                           bool checkCeilings, bool checkDyna,
                           Vec3f* posIntersect, CollisionPoly** outPoly,
                           int* outDynaId) {
  bool result = false;
  Vec3f posA = posPrev;
  Vec3f posB = posNext;
  f32 minDistSq = 1.0e38f;

  // bug? For scene collision, floors are checked before walls, while for
  // dynapoly, walls are checked before floors.
  if (checkFloors && BgCheck_CheckLineAgainstList(col, &col->floors, posA,
                                                  &posB, &minDistSq, outPoly)) {
    *outDynaId = -1;
    result = true;
  }

  if (checkWalls && BgCheck_CheckLineAgainstList(col, &col->walls, posA, &posB,
                                                 &minDistSq, outPoly)) {
    *outDynaId = -1;
    result = true;
  }

  if (checkCeilings &&
      BgCheck_CheckLineAgainstList(col, &col->ceilings, posA, &posB, &minDistSq,
                                   outPoly)) {
    *outDynaId = -1;
    result = true;
  }

  if (checkDyna) {
    for (int i = 0; i < col->dynas.size(); i++) {
      Dyna* dyna = &col->dynas[i];
      if (posA.y < dyna->minY || posA.y > dyna->maxY || posB.y < dyna->minY ||
          posB.y > dyna->maxY) {
        continue;
      }

      if (checkWalls &&
          BgCheck_CheckLineAgainstDynaList(col, dyna, &dyna->walls, posA, &posB,
                                           &minDistSq, outPoly)) {
        *outDynaId = i;
        result = true;
      }

      if (checkFloors &&
          BgCheck_CheckLineAgainstDynaList(col, dyna, &dyna->floors, posA,
                                           &posB, &minDistSq, outPoly)) {
        *outDynaId = i;
        result = true;
      }

      if (checkCeilings &&
          BgCheck_CheckLineAgainstDynaList(col, dyna, &dyna->ceilings, posA,
                                           &posB, &minDistSq, outPoly)) {
        *outDynaId = i;
        result = true;
      }
    }
  }

  *posIntersect = posB;
  return result;
}

bool BgCheck_SphVsStaticWall(Collision* col, Vec3f pos, f32 radius, f32* x,
                             f32* z, CollisionPoly** wallPoly) {
  bool result = false;
  Vec3f resultPos = pos;

  for (CollisionPoly* poly : col->walls) {
    Vec3f polyVerts[3];
    CollisionPoly_GetVertices(poly, col->vtxList, polyVerts);

    // TODO: sort by min Y
    if (pos.y < polyVerts[0].y && pos.y < polyVerts[1].y &&
        pos.y < polyVerts[2].y) {
      continue;
    }

    Vec3f normal = CollisionPoly_GetNormalF(poly);
    f32 normalXZ = sqrtf(SQ(normal.x) + SQ(normal.z));
    f32 planeDist = Math3D_DistPlaneToPos(normal.x, normal.y, normal.z,
                                          (s16)poly->dist, &resultPos);
    if (fabsf(planeDist) > radius) {
      continue;
    }

    f32 invNormalXZ = 1.0f / normalXZ;
    f32 zDist = fabsf(normal.z) * invNormalXZ;
    if (zDist < 0.4f) {
      continue;
    }

    // compute curPoly zMin/zMax
    f32 zMin =
        std::min(std::min(polyVerts[0].z, polyVerts[1].z), polyVerts[2].z) -
        radius;
    f32 zMax =
        std::max(std::max(polyVerts[0].z, polyVerts[1].z), polyVerts[2].z) +
        radius;

    if (resultPos.z < zMin || resultPos.z > zMax) {
      continue;
    }

    f32 intersect;
    if (Math3D_TriChkPointParaZIntersect(
            &polyVerts[0], &polyVerts[1], &polyVerts[2], normal.x, normal.y,
            normal.z, (s16)poly->dist, resultPos.x, pos.y, &intersect)) {
      if (fabsf(intersect - resultPos.z) <= radius / zDist) {
        if ((intersect - resultPos.z) * normal.z <= 4.0f) {
          BgCheck_ComputeWallDisplacement(poly, &resultPos.x, &resultPos.z,
                                          normal, invNormalXZ, planeDist,
                                          radius);
          result = true;
          *wallPoly = poly;
        }
      }
    }
  }

  for (CollisionPoly* poly : col->walls) {
    Vec3f polyVerts[3];
    CollisionPoly_GetVertices(poly, col->vtxList, polyVerts);

    // TODO: sort by min Y
    if (pos.y < polyVerts[0].y && pos.y < polyVerts[1].y &&
        pos.y < polyVerts[2].y) {
      continue;
    }

    Vec3f normal = CollisionPoly_GetNormalF(poly);
    f32 normalXZ = sqrtf(SQ(normal.x) + SQ(normal.z));
    f32 planeDist = Math3D_DistPlaneToPos(normal.x, normal.y, normal.z,
                                          (s16)poly->dist, &resultPos);
    if (fabsf(planeDist) > radius) {
      continue;
    }

    f32 invNormalXZ = 1.0f / normalXZ;
    f32 xDist = fabsf(normal.x) * invNormalXZ;
    if (xDist < 0.4f) {
      continue;
    }

    // compute curPoly xMin/xMax
    f32 xMin =
        std::min(std::min(polyVerts[0].x, polyVerts[1].x), polyVerts[2].x) -
        radius;
    f32 xMax =
        std::max(std::max(polyVerts[0].x, polyVerts[1].x), polyVerts[2].x) +
        radius;

    if (resultPos.x < xMin || resultPos.x > xMax) {
      continue;
    }

    f32 intersect;
    if (Math3D_TriChkPointParaXIntersect(
            &polyVerts[0], &polyVerts[1], &polyVerts[2], normal.x, normal.y,
            normal.z, (s16)poly->dist, pos.y, resultPos.z, &intersect)) {
      if (fabsf(intersect - resultPos.x) <= radius / xDist) {
        if ((intersect - resultPos.x) * normal.x <= 4.0f) {
          BgCheck_ComputeWallDisplacement(poly, &resultPos.x, &resultPos.z,
                                          normal, invNormalXZ, planeDist,
                                          radius);
          result = true;
          *wallPoly = poly;
        }
      }
    }
  }

  *x = resultPos.x;
  *z = resultPos.z;
  return result;
}

bool BgCheck_SphVsDynaWall(Collision* col, Vec3f pos, f32 radius, f32* x,
                           f32* z, CollisionPoly** wallPoly) {
  bool result = false;
  Vec3f resultPos = pos;

  for (Dyna& dyna : col->dynas) {
    if (pos.y < dyna.minY || pos.y > dyna.maxY) {
      continue;
    }

    for (CollisionPoly* poly : dyna.walls) {
      Vec3f polyVerts[3];
      CollisionPoly_GetVertices(poly, dyna.vertices.data(), polyVerts);

      // TODO: sort by min Y
      if (pos.y < polyVerts[0].y && pos.y < polyVerts[1].y &&
          pos.y < polyVerts[2].y) {
        continue;
      }

      Vec3f normal = CollisionPoly_GetNormalF(poly);
      f32 normalXZ = sqrtf(SQ(normal.x) + SQ(normal.z));
      f32 planeDist = Math3D_DistPlaneToPos(normal.x, normal.y, normal.z,
                                            (s16)poly->dist, &resultPos);
      if (fabsf(planeDist) > radius) {
        continue;
      }

      f32 invNormalXZ = 1.0f / normalXZ;
      f32 zDist = fabsf(normal.z) * invNormalXZ;
      if (zDist < 0.4f) {
        continue;
      }

      // compute curPoly zMin/zMax
      f32 zMin =
          std::min(std::min(polyVerts[0].z, polyVerts[1].z), polyVerts[2].z) -
          radius;
      f32 zMax =
          std::max(std::max(polyVerts[0].z, polyVerts[1].z), polyVerts[2].z) +
          radius;

      if (resultPos.z < zMin || resultPos.z > zMax) {
        continue;
      }

      f32 intersect;
      if (Math3D_TriChkPointParaZIntersect(
              &polyVerts[0], &polyVerts[1], &polyVerts[2], normal.x, normal.y,
              normal.z, (s16)poly->dist, resultPos.x, pos.y, &intersect)) {
        if (fabsf(intersect - resultPos.z) <= radius / zDist) {
          if ((intersect - resultPos.z) * normal.z <= 4.0f) {
            BgCheck_ComputeWallDisplacement(poly, &resultPos.x, &resultPos.z,
                                            normal, invNormalXZ, planeDist,
                                            radius);
            result = true;
            *wallPoly = poly;
          }
        }
      }
    }

    for (CollisionPoly* poly : dyna.walls) {
      Vec3f polyVerts[3];
      CollisionPoly_GetVertices(poly, dyna.vertices.data(), polyVerts);

      // TODO: sort by min Y
      if (pos.y < polyVerts[0].y && pos.y < polyVerts[1].y &&
          pos.y < polyVerts[2].y) {
        continue;
      }

      Vec3f normal = CollisionPoly_GetNormalF(poly);
      f32 normalXZ = sqrtf(SQ(normal.x) + SQ(normal.z));
      f32 planeDist = Math3D_DistPlaneToPos(normal.x, normal.y, normal.z,
                                            (s16)poly->dist, &resultPos);
      if (fabsf(planeDist) > radius) {
        continue;
      }

      f32 invNormalXZ = 1.0f / normalXZ;
      f32 xDist = fabsf(normal.x) * invNormalXZ;
      if (xDist < 0.4f) {
        continue;
      }

      // compute curPoly xMin/xMax
      f32 xMin =
          std::min(std::min(polyVerts[0].x, polyVerts[1].x), polyVerts[2].x) -
          radius;
      f32 xMax =
          std::max(std::max(polyVerts[0].x, polyVerts[1].x), polyVerts[2].x) +
          radius;

      if (resultPos.x < xMin || resultPos.x > xMax) {
        continue;
      }

      f32 intersect;
      if (Math3D_TriChkPointParaXIntersect(
              &polyVerts[0], &polyVerts[1], &polyVerts[2], normal.x, normal.y,
              normal.z, (s16)poly->dist, pos.y, resultPos.z, &intersect)) {
        if (fabsf(intersect - resultPos.x) <= radius / xDist) {
          if ((intersect - resultPos.x) * normal.x <= 4.0f) {
            BgCheck_ComputeWallDisplacement(poly, &resultPos.x, &resultPos.z,
                                            normal, invNormalXZ, planeDist,
                                            radius);
            result = true;
            *wallPoly = poly;
          }
        }
      }
    }
  }

  *x = resultPos.x;
  *z = resultPos.z;
  return result;
}

bool BgCheck_EntitySphVsWall(Collision* col, Vec3f posPrev, Vec3f posNext,
                             Vec3f* posResult, CollisionPoly** wallPoly) {
  f32 radius = col->age == PLAYER_AGE_CHILD ? 14.0f : 18.0f;
  f32 checkHeight = 26.0f;

  CollisionPoly* poly;
  int dynaId;

  *posResult = posNext;
  bool result = false;

  Vec3f delta = posNext - posPrev;
  f32 dx = delta.x;
  f32 dy = delta.y;
  f32 dz = delta.z;

  if (dx != 0.0f || dz != 0.0f) {
    // Ground clip condition
    if (checkHeight + dy < 5.0f) {
      //! @bug checkHeight is not applied to posPrev/posNext
      Vec3f posIntersect;
      if (BgCheck_CheckLineImpl(col, posPrev, posNext, true, true, false, true,
                                &posIntersect, &poly, &dynaId)) {
        result = true;
        *wallPoly = poly;
        Vec3f normal = CollisionPoly_GetNormalF(poly);
        // if poly is floor, push result underneath the floor
        if (normal.y > 0.5f) {
          *posResult = posIntersect - Vec3f(0, 1, 0);
        } else {  // poly is wall
          *posResult = normal * radius + posIntersect;
        }
      }
    } else {
      // if the radius is less than the distance travelled on the xz plane,
      // also test for floor collisions
      bool checkFloors = (SQ(radius) < (SQ(dx) + SQ(dz)));

      // perform a straight line test to see if a line at posNext.y +
      // checkHeight from posPrev.xz to posNext.xz passes through any wall
      // and possibly floor polys
      Vec3f checkLineNext = posNext;
      checkLineNext.y += checkHeight;
      Vec3f checkLinePrev = posPrev;
      checkLinePrev.y = checkLineNext.y;

      Vec3f posIntersect;
      if (BgCheck_CheckLineImpl(col, checkLinePrev, checkLineNext, true,
                                checkFloors, false, true, &posIntersect, &poly,
                                &dynaId)) {
        *wallPoly = poly;
        Vec3f normal = CollisionPoly_GetNormalF(poly);
        f32 nXZDist = sqrtf(SQ(normal.x) + SQ(normal.z));

        // if poly is not a "flat" floor or "flat" ceiling
        if (!IS_ZERO(nXZDist)) {
          // normalize nx, nz and multiply each by the radius to go back to
          // the other side of the wall
          Vec3f offset = normal * (radius * (1.0f / nXZDist)) + posIntersect;
          posResult->x = offset.x;
          posResult->z = offset.z;
          result = true;
        }
      }
    }
  }

  Vec3f sphCenter = *posResult;
  sphCenter.y += checkHeight;

  bool dynaResult = false;
  if (BgCheck_SphVsDynaWall(col, sphCenter, radius, &posResult->x,
                            &posResult->z, &poly)) {
    result = true;
    dynaResult = true;
    sphCenter = *posResult;
    sphCenter.y += checkHeight;
  }

  if (BgCheck_SphVsStaticWall(col, sphCenter, radius, &posResult->x,
                              &posResult->z, &poly)) {
    dynaId = -1;
    result = true;
    *wallPoly = poly;
  }

  if (dynaResult || dynaId != -1) {
    Vec3f posIntersect;
    if (BgCheck_CheckLineImpl(col, posPrev, *posResult, true, false, false,
                              false, &posIntersect, &poly, &dynaId)) {
      Vec3f normal = CollisionPoly_GetNormalF(poly);
      f32 nXZDist = sqrtf(SQ(normal.x) + SQ(normal.z));

      // if poly is not a "flat" floor or "flat" ceiling
      if (!IS_ZERO(nXZDist)) {
        // normalize nx, nz and multiply each by the radius to go back to
        // the other side of the wall
        Vec3f offset = normal * (radius * (1.0f / nXZDist)) + posIntersect;
        posResult->x = offset.x;
        posResult->z = offset.z;
      }
    }
  }

  return result;
}

bool BgCheck_RaycastDownStaticList(Collision* col,
                                   std::vector<CollisionPoly*>* polys,
                                   Vec3f pos, f32* floorHeight,
                                   CollisionPoly** floorPoly) {
  bool result = false;
  f32 yIntersect;
  for (CollisionPoly* poly : *polys) {
    Vec3f polyVerts[3];

    CollisionPoly_GetVertices(poly, col->vtxList, polyVerts);
    Vec3f normal = CollisionPoly_GetNormalF(poly);
    if (normal.y < 0) {
      continue;
    }

    if (Math3D_TriChkPointParaYIntersectInsideTri(
            &polyVerts[0], &polyVerts[1], &polyVerts[2], normal.x, normal.y,
            normal.z, (s16)poly->dist, pos.z, pos.x, &yIntersect, 1.0f)) {
      // if poly is closer to pos without going over
      if (yIntersect < pos.y && *floorHeight < yIntersect) {
        result = true;
        *floorHeight = yIntersect;
        *floorPoly = poly;
      }
    }
  }
  return result;
}

bool BgCheck_RaycastDownStatic(Collision* col, Vec3f pos, f32* floorHeight,
                               CollisionPoly** floorPoly, int* dynaId) {
  bool result = false;
  *floorHeight = BGCHECK_Y_MIN;

  if (BgCheck_RaycastDownStaticList(col, &col->floors, pos, floorHeight,
                                    floorPoly)) {
    *dynaId = -1;
    result = true;
  }

  if (BgCheck_RaycastDownStaticList(col, &col->walls, pos, floorHeight,
                                    floorPoly)) {
    *dynaId = -1;
    result = true;
  }

  if (*floorHeight != BGCHECK_Y_MIN && SurfaceType_IsSoft(col, *floorPoly)) {
    *floorHeight -= 1.0f;
  }

  return result;
}

bool BgCheck_RaycastDownDynaList(Collision* col, Dyna* dyna,
                                 std::vector<CollisionPoly*>* polys, Vec3f pos,
                                 f32* floorHeight, CollisionPoly** floorPoly) {
  bool result = false;
  for (CollisionPoly* poly : *polys) {
    Vec3f polyVerts[3];
    CollisionPoly_GetVertices(poly, dyna->vertices.data(), polyVerts);
    Vec3f normal = CollisionPoly_GetNormalF(poly);

    // BGCHECK_RAYCAST_DOWN_CHECK_GROUND_ONLY
    if (normal.y < 0.0f) {
      continue;
    }

    f32 yIntersect;
    if (Math3D_TriChkPointParaYIntersectDist(
            &polyVerts[0], &polyVerts[1], &polyVerts[2], normal.x, normal.y,
            normal.z, (s16)poly->dist, pos.z, pos.x, &yIntersect, 1.0f) &&
        yIntersect < pos.y && *floorHeight < yIntersect) {
      result = true;
      *floorHeight = yIntersect;
      *floorPoly = poly;
    }
  }
  return result;
}

bool BgCheck_RaycastDownDyna(Collision* col, Vec3f pos, f32* floorHeight,
                             CollisionPoly** floorPoly, int* dynaId) {
  bool result = false;
  for (int i = 0; i < col->dynas.size(); i++) {
    Dyna* dyna = &col->dynas[i];
    if (BgCheck_RaycastDownDynaList(col, dyna, &dyna->floors, pos, floorHeight,
                                    floorPoly)) {
      *dynaId = i;
      result = true;
    }

    // TODO: With BGCHECK_RAYCAST_DOWN_CHECK_WALLS_SIMPLE, dynapoly walls are
    // only checked if there is no floor detected i.e. we're over the void. This
    // is pretty rare but maybe it will be needed someday.

    // if (BgCheck_RaycastDownDynaList(col, dyna, &dyna->walls, pos, floorHeight,
    //                                 floorPoly)) {
    //   *dynaId = i;
    //   result = true;
    // }
  }
  return result;
}

bool BgCheck_RaycastDownImpl(Collision* col, Vec3f pos, f32* floorHeight,
                             CollisionPoly** floorPoly, int* dynaId) {
  bool result = false;
  *floorHeight = BGCHECK_Y_MIN;

  if (BgCheck_RaycastDownStatic(col, pos, floorHeight, floorPoly, dynaId)) {
    result = true;
  }

  if (BgCheck_RaycastDownDyna(col, pos, floorHeight, floorPoly, dynaId)) {
    result = true;
  }

  return result;
}

Collision::Collision(CollisionHeader* header, PlayerAge age) {
  this->header = header;
  this->age = age;
  this->vtxList = header->vertices;
  this->polyList = header->polys;
}

Collision::Collision(CollisionHeader* header, PlayerAge age, Vec3f min,
                     Vec3f max) {
  this->header = header;
  this->age = age;
  this->vtxList = header->vertices;
  this->polyList = header->polys;

  // TODO: what order?
  for (int polyId = 0; polyId < header->numPolys; polyId++) {
    CollisionPoly* poly = &header->polys[polyId];

    Vec3f polyVerts[3];
    CollisionPoly_GetVertices(poly, this->vtxList, polyVerts);

    for (int i = 0; i < 3; i++) {
      if (min.x <= polyVerts[i].x && polyVerts[i].x <= max.x &&
          min.y <= polyVerts[i].y && polyVerts[i].y <= max.y &&
          min.z <= polyVerts[i].z && polyVerts[i].z <= max.z) {
        addPoly(polyId);
        break;
      }
    }
  }
}

void printPoly(CollisionPoly* poly, Vec3s* vtxList, int index) {
  Vec3f v[3];
  CollisionPoly_GetVertices(poly, vtxList, v);
  Vec3f normal = CollisionPoly_GetNormalF(poly);
  s16 dist = poly->dist;
  f32 actualDist = -DOTXYZ(normal, v[0]);
  printf(
      "    index=%d v1=(%.0f, %.0f, %.0f) v2=(%.0f, %.0f, %.0f) v3=(%.0f, "
      "%.0f, %.0f) nx=%04x (%.4f) ny=%04x (%.4f) nz=%04x (%.4f) dist=%d "
      "actualDist=%.9g\n",
      index, v[0].x, v[0].y, v[0].z, v[1].x, v[1].y, v[1].z, v[2].x, v[2].y,
      v[2].z, poly->nx, normal.x, poly->ny, normal.y, poly->nz, normal.z, dist,
      actualDist);
}

void Collision::printPolys() {
  printf("scene collision:\n");
  printf("  walls:\n");
  for (CollisionPoly* poly : this->walls) {
    printPoly(poly, this->vtxList, poly - this->polyList);
  }

  printf("  floors:\n");
  for (CollisionPoly* poly : this->floors) {
    printPoly(poly, this->vtxList, poly - this->polyList);
  }

  printf("  ceilings:\n");
  for (CollisionPoly* poly : this->ceilings) {
    printPoly(poly, this->vtxList, poly - this->polyList);
  }

  for (int i = 0; i < this->dynas.size(); i++) {
    Dyna* dyna = &this->dynas[i];
    printf("dyna %d:\n", i);
    printf("  walls:\n");
    for (CollisionPoly* poly : dyna->walls) {
      printPoly(poly, dyna->vertices.data(), poly - dyna->polys.data());
    }

    printf("  floors:\n");
    for (CollisionPoly* poly : dyna->floors) {
      printPoly(poly, dyna->vertices.data(), poly - dyna->polys.data());
    }

    printf("  ceilings:\n");
    for (CollisionPoly* poly : dyna->ceilings) {
      printPoly(poly, dyna->vertices.data(), poly - dyna->polys.data());
    }
  }
}

void Collision::addPoly(int polyIndex) {
  CollisionPoly* poly = &this->polyList[polyIndex];

  if ((s16)poly->ny > (s16)(0.5f * SHT_MAX)) {
    this->floors.push_back(poly);
  } else if ((s16)poly->ny < (s16)(-0.8f * SHT_MAX)) {
    this->ceilings.push_back(poly);
  } else {
    this->walls.push_back(poly);
  }
}

int Collision::addDynapoly(CollisionHeader* header, Vec3f scale, Vec3s rot,
                            Vec3f pos) {
  this->dynas.push_back(Dyna());
  int dynaId = this->dynas.size() - 1;
  updateDynapoly(dynaId, header, scale, rot, pos);
  return dynaId;
}

void Collision::updateDynapoly(int dynaId, CollisionHeader* header, Vec3f scale, Vec3s rot,
                            Vec3f pos) {
  Dyna* dyna = &this->dynas[dynaId];
  dyna->header = header;

  dyna->vertices.clear();
  dyna->vertices.reserve(header->numVertices);

  dyna->polys.clear();
  dyna->polys.reserve(header->numPolys);

  dyna->floors.clear();
  dyna->walls.clear();
  dyna->ceilings.clear();

  MtxF mtx;
  SkinMatrix_SetTranslateRotateYXZScale(&mtx, scale.x, scale.y, scale.z, rot.x,
                                        rot.y, rot.z, pos.x, pos.y, pos.z);

  dyna->minY = 1.0e38f;
  dyna->maxY = -1.0e38f;

  for (int i = 0; i < header->numVertices; i++) {
    Vec3f vtx = Vec3f(header->vertices[i]);
    Vec3f vtxT;  // Vtx after mtx transform
    SkinMatrix_Vec3fMtxFMultXYZ(&mtx, &vtx, &vtxT);

    dyna->minY = std::min(dyna->minY, vtxT.y);
    dyna->maxY = std::max(dyna->maxY, vtxT.y);

    dyna->vertices.push_back(vtxT.toVec3s());
  }

  for (int i = 0; i < header->numPolys; i++) {
    dyna->polys.push_back(header->polys[i]);
    CollisionPoly* poly = &dyna->polys.back();

    Vec3f polyVerts[3];
    CollisionPoly_GetVertices(poly, dyna->vertices.data(), polyVerts);

    Vec3f normal;
    Math3D_SurfaceNorm(&polyVerts[0], &polyVerts[1], &polyVerts[2], &normal);
    f32 normMagnitude = Math3D_Vec3fMagnitude(&normal);

    if (!IS_ZERO(normMagnitude)) {
      normal = normal * (1.0f / normMagnitude);
      poly->nx = (s16)(normal.x * SHT_MAX);
      poly->ny = (s16)(normal.y * SHT_MAX);
      poly->nz = (s16)(normal.z * SHT_MAX);
    }

    poly->dist = -DOTXYZ(normal, polyVerts[0]);
    if (normal.y > 0.5f) {
      dyna->floors.push_back(poly);
    } else if (normal.y < -0.8f) {
      dyna->ceilings.push_back(poly);
    } else {
      dyna->walls.push_back(poly);
    }
  }
  std::reverse(dyna->floors.begin(), dyna->floors.end());
  std::reverse(dyna->ceilings.begin(), dyna->ceilings.end());
  std::reverse(dyna->walls.begin(), dyna->walls.end());
}

Vec3f Collision::runChecks(Vec3f prevPos, Vec3f intendedPos,
                           CollisionPoly** wallPoly, CollisionPoly** floorPoly,
                           int* dynaId, f32* floorHeight) {
  *wallPoly = NULL;
  *floorPoly = NULL;
  *floorHeight = -32000.0f;

  // Check walls
  Vec3f wallResult;
  if (BgCheck_EntitySphVsWall(this, prevPos, intendedPos, &wallResult,
                              wallPoly)) {
    intendedPos = wallResult;
  }

  // Check floors
  Vec3f checkPos = intendedPos;
  checkPos.y = prevPos.y + 50.0f;
  if (BgCheck_RaycastDownImpl(this, checkPos, floorHeight, floorPoly, dynaId)) {
    f32 floorHeightDiff = *floorHeight - intendedPos.y;
    if (floorHeightDiff >= 0.0f) {  // actor is on or below the ground
      intendedPos.y = *floorHeight;
    }
  }

  // TODO: check ceilings

  return intendedPos;
}

Vec3f Collision::runChecks(Vec3f prevPos, Vec3f intendedPos) {
  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;
  int dynaId;
  f32 floorHeight;
  return runChecks(prevPos, intendedPos, &wallPoly, &floorPoly, &dynaId,
                   &floorHeight);
}

Vec3f Collision::findFloor(Vec3f pos, CollisionPoly** outPoly, int* dynaId) {
  f32 floorHeight;
  *outPoly = NULL;
  *dynaId = -1;
  BgCheck_RaycastDownImpl(this, pos, &floorHeight, outPoly, dynaId);
  return Vec3f(pos.x, floorHeight, pos.z);
}

Vec3f Collision::findFloor(Vec3f pos) {
  CollisionPoly* poly;
  int dynaId;
  return findFloor(pos, &poly, &dynaId);
}

Vec3f Collision::entityLineTest(Vec3f pos, Vec3f target, bool checkWalls,
                                bool checkFloors, bool checkCeilings,
                                CollisionPoly** outPoly) {
  *outPoly = NULL;
  int dynaId;
  BgCheck_CheckLineImpl(this, pos, target, checkWalls, checkFloors,
                        checkCeilings, true, &target, outPoly, &dynaId);
  return target;
}

Vec3f Collision::cameraLineTest(Vec3f pos, Vec3f target,
                                CollisionPoly** outPoly) {
  *outPoly = NULL;
  int dynaId;
  BgCheck_CheckLineImpl(this, pos, target, true, true, true, true, &target,
                        outPoly, &dynaId);
  return target;
}

f32 Collision::cameraFindFloor(Vec3f pos, CollisionPoly** outPoly) {
  *outPoly = NULL;
  f32 floorHeight;
  int dynaId;
  BgCheck_RaycastDownImpl(this, pos, &floorHeight, outPoly, &dynaId);
  return floorHeight;
}

u16 Collision::getCameraSetting(CollisionPoly* poly, int dynaId) {
  CollisionHeader* header;
  if (dynaId == -1) {
    header = this->header;
  } else {
    header = this->dynas[dynaId].header;
  }

  int bgCamIndex = header->surfaceTypeList[poly->type].data[0] & 0xFF;
  return header->bgCamList[bgCamIndex].setting;
}
