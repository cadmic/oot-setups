#include "collision.hpp"

#include <algorithm>
#include <iostream>

#include "global.hpp"
#include "skin_matrix.hpp"
#include "sys_math3d.hpp"

#define COLPOLY_NORMAL_FRAC (1.0f / SHT_MAX)
#define COLPOLY_SNORMAL(x) ()

void printCollisionObj(CollisionHeader* collision) {
  for (int i = 0; i < collision->numVertices; i++) {
    Vec3s* v = &collision->vertices[i];
    std::cout << "v " << v->x << " " << v->y << " " << v->z << std::endl;
  }

  for (int i = 0; i < collision->numPolys; i++) {
    CollisionPoly* poly = &collision->polys[i];
    std::cout << "f " << (poly->v1 & 0x1FFF) + 1 << " "
              << (poly->v2 & 0x1FFF) + 1 << " " << (poly->v3 & 0x1FFF) + 1
              << std::endl;
  }
}

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
                                      std::vector<CollisionPoly>* polys,
                                      Vec3f posA, Vec3f* posB, f32* minDistSq,
                                      CollisionPoly** outPoly) {
  bool result = false;
  for (CollisionPoly& poly : *polys) {
    Vec3f polyIntersect;
    if (CollisionPoly_LineVsPoly(&poly, dyna->vertices.data(), posA, *posB,
                                 &polyIntersect)) {
      f32 distSq = Math3D_Vec3fDistSq(&posA, &polyIntersect);
      if (distSq < *minDistSq) {
        *minDistSq = distSq;
        *posB = polyIntersect;
        *outPoly = &poly;
        result = true;
      }
    }
  }
  return result;
}

bool BgCheck_CheckLineAgainstDyna(Collision* col, Vec3f posA, Vec3f* posB,
                                  bool checkFloors, bool checkWalls,
                                  f32* minDistSq, CollisionPoly** outPoly) {
  bool result = false;
  for (Dyna& dyna : col->dynas) {
    if (posA.y < dyna.minY || posA.y > dyna.maxY || posB->y < dyna.minY ||
        posB->y > dyna.maxY) {
      continue;
    }

    if (checkFloors &&
        BgCheck_CheckLineAgainstDynaList(col, &dyna, &dyna.floors, posA, posB,
                                         minDistSq, outPoly)) {
      result = true;
    }

    if (checkWalls &&
        BgCheck_CheckLineAgainstDynaList(col, &dyna, &dyna.walls, posA, posB,
                                         minDistSq, outPoly)) {
      result = true;
    }
  }
  return result;
}

bool BgCheck_CheckLineImpl(Collision* col, Vec3f posPrev, Vec3f posNext,
                           bool checkWalls, bool checkFloors, bool checkDyna,
                           Vec3f* posIntersect, CollisionPoly** outPoly) {
  bool result = false;
  Vec3f posA = posPrev;
  Vec3f posB = posNext;
  f32 minDistSq = 1.0e38f;
  if (checkFloors && BgCheck_CheckLineAgainstList(col, &col->floors, posA,
                                                  &posB, &minDistSq, outPoly)) {
    result = true;
  }

  if (BgCheck_CheckLineAgainstList(col, &col->walls, posA, &posB, &minDistSq,
                                   outPoly)) {
    result = true;
  }

  if (checkDyna &&
      BgCheck_CheckLineAgainstDyna(col, posA, &posB, checkFloors, checkWalls,
                                   &minDistSq, outPoly)) {
    result = true;
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

    for (CollisionPoly& wall : dyna.walls) {
      CollisionPoly* poly = &wall;
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

    for (CollisionPoly& wall : dyna.walls) {
      CollisionPoly* poly = &wall;
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
      CollisionPoly* poly;
      if (BgCheck_CheckLineImpl(col, posPrev, posNext, true, true, true,
                                &posIntersect, &poly)) {
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
      // if the radius is less than the distance travelled on the xz plane, also
      // test for floor collisions
      bool checkFloors = (SQ(radius) < (SQ(dx) + SQ(dz)));

      // perform a straight line test to see if a line at posNext.y +
      // checkHeight from posPrev.xz to posNext.xz passes through any wall and
      // possibly floor polys
      Vec3f checkLineNext = posNext;
      checkLineNext.y += checkHeight;
      Vec3f checkLinePrev = posPrev;
      checkLinePrev.y = checkLineNext.y;

      Vec3f posIntersect;
      CollisionPoly* poly;
      if (BgCheck_CheckLineImpl(col, checkLinePrev, checkLineNext, true,
                                checkFloors, true, &posIntersect, &poly)) {
        *wallPoly = poly;
        Vec3f normal = CollisionPoly_GetNormalF(poly);
        f32 nXZDist = sqrtf(SQ(normal.x) + SQ(normal.z));

        // if poly is not a "flat" floor or "flat" ceiling
        if (!IS_ZERO(nXZDist)) {
          // normalize nx, nz and multiply each by the radius to go back to the
          // other side of the wall
          Vec3f offset = normal * (radius * (1.0f / nXZDist)) + posIntersect;
          posResult->x = offset.x;
          posResult->z = offset.z;
          result = true;
        }
      }
    }
  }

  CollisionPoly* poly;
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

  bool staticResult = false;
  if (BgCheck_SphVsStaticWall(col, sphCenter, radius, &posResult->x,
                              &posResult->z, &poly)) {
    staticResult = true;
    result = true;
    *wallPoly = poly;
  }

  if (dynaResult || !staticResult) {
    Vec3f posIntersect;
    if (BgCheck_CheckLineImpl(col, posPrev, *posResult, true, false, false,
                              &posIntersect, &poly)) {
      Vec3f normal = CollisionPoly_GetNormalF(poly);
      f32 nXZDist = sqrtf(SQ(normal.x) + SQ(normal.z));

      // if poly is not a "flat" floor or "flat" ceiling
      if (!IS_ZERO(nXZDist)) {
        // normalize nx, nz and multiply each by the radius to go back to the
        // other side of the wall
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
                               CollisionPoly** floorPoly) {
  bool result = false;
  *floorHeight = BGCHECK_Y_MIN;

  if (BgCheck_RaycastDownStaticList(col, &col->floors, pos, floorHeight,
                                    floorPoly)) {
    result = true;
  }

  if (BgCheck_RaycastDownStaticList(col, &col->walls, pos, floorHeight,
                                    floorPoly)) {
    result = true;
  }

  return result;
}

bool BgCheck_RaycastDownDynaList(Collision* col, Dyna* dyna,
                                 std::vector<CollisionPoly>* polys, Vec3f pos,
                                 f32* floorHeight, CollisionPoly** floorPoly) {
  bool result = false;
  for (CollisionPoly& poly : *polys) {
    Vec3f polyVerts[3];
    CollisionPoly_GetVertices(&poly, dyna->vertices.data(), polyVerts);
    Vec3f normal = CollisionPoly_GetNormalF(&poly);

    f32 yIntersect;
    if (Math3D_TriChkPointParaYIntersectDist(
            &polyVerts[0], &polyVerts[1], &polyVerts[2], normal.x, normal.y,
            normal.z, (s16)poly.dist, pos.z, pos.x, &yIntersect, 1.0f) &&
        yIntersect < pos.y && *floorHeight < yIntersect) {
      result = true;
      *floorHeight = yIntersect;
      *floorPoly = &poly;
    }
  }
  return result;
}

bool BgCheck_RaycastDownDyna(Collision* col, Vec3f pos, f32* floorHeight,
                             CollisionPoly** floorPoly) {
  Dyna* floorDyna = NULL;
  for (Dyna& dyna : col->dynas) {
    if (BgCheck_RaycastDownDynaList(col, &dyna, &dyna.floors, pos, floorHeight,
                                    floorPoly)) {
      floorDyna = &dyna;
    }

    if (BgCheck_RaycastDownDynaList(col, &dyna, &dyna.walls, pos, floorHeight,
                                    floorPoly)) {
      floorDyna = &dyna;
    }
  }

  if (!floorDyna) {
    return false;
  }

  Vec3f origVtx[3];
  CollisionPoly_GetVertices(*floorPoly, floorDyna->vtxList, origVtx);

  Vec3f polyVtx[3];
  for (int i = 0; i < 3; i++) {
    SkinMatrix_Vec3fMtxFMultXYZ(&floorDyna->mtx, &origVtx[i], &polyVtx[i]);
  }

  Vec3f polyNorm;
  Math3D_SurfaceNorm(&polyVtx[0], &polyVtx[1], &polyVtx[2], &polyNorm);
  f32 magnitude = Math3D_Vec3fMagnitude(&polyNorm);

  if (!IS_ZERO(magnitude)) {
    polyNorm = polyNorm * (1.0f / magnitude);
    f32 polyDist = -DOTXYZ(polyNorm, polyVtx[0]);
    f32 intersect;
    if (Math3D_TriChkPointParaYIntersectInsideTri(
            &polyVtx[0], &polyVtx[1], &polyVtx[2], polyNorm.x, polyNorm.y,
            polyNorm.z, polyDist, pos.z, pos.x, &intersect, 1.0f)) {
      if (fabsf(intersect - *floorHeight) < 1.0f) {
        *floorHeight = intersect;
      }
    }
  }

  return true;
}

bool BgCheck_RaycastDownImpl(Collision* col, Vec3f pos, f32* floorHeight,
                             CollisionPoly** floorPoly) {
  bool result = false;
  *floorHeight = BGCHECK_Y_MIN;

  if (BgCheck_RaycastDownStatic(col, pos, floorHeight, floorPoly)) {
    result = true;
  }

  if (BgCheck_RaycastDownDyna(col, pos, floorHeight, floorPoly)) {
    result = true;
  }

  return result;
}

Collision::Collision(CollisionHeader* header, PlayerAge age) {
  this->age = age;
  this->vtxList = header->vertices;
  this->polyList = header->polys;
}

Collision::Collision(CollisionHeader* header, PlayerAge age, Vec3f min,
                     Vec3f max) {
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

void Collision::addPoly(int polyId) {
  CollisionPoly* poly = &this->polyList[polyId];

  if ((s16)poly->ny > (s16)(0.5f * SHT_MAX)) {
    this->floors.push_back(poly);
  } else if ((s16)poly->ny < (s16)(-0.8f * SHT_MAX)) {
    this->ceilings.push_back(poly);
  } else {
    this->walls.push_back(poly);
  }
}

void Collision::addDynapoly(CollisionHeader* header, Vec3f scale, Vec3s rot,
                            Vec3f pos) {
  Dyna dyna;

  SkinMatrix_SetTranslateRotateYXZScale(&dyna.mtx, scale.x, scale.y, scale.z,
                                        rot.x, rot.y, rot.z, pos.x, pos.y,
                                        pos.z);

  dyna.minY = 1.0e38f;
  dyna.maxY = -1.0e38f;

  for (int i = 0; i < header->numVertices; i++) {
    Vec3f vtx = Vec3f(header->vertices[i]);
    Vec3f vtxT;  // Vtx after mtx transform
    SkinMatrix_Vec3fMtxFMultXYZ(&dyna.mtx, &vtx, &vtxT);

    dyna.minY = std::min(dyna.minY, vtxT.y);
    dyna.maxY = std::max(dyna.maxY, vtxT.y);

    dyna.vertices.push_back(vtxT.toVec3s());
  }

  for (int i = 0; i < header->numPolys; i++) {
    CollisionPoly newPoly = header->polys[i];

    Vec3f polyVerts[3];
    CollisionPoly_GetVertices(&header->polys[i], dyna.vertices.data(),
                              polyVerts);

    Vec3f newNormal;
    Math3D_SurfaceNorm(&polyVerts[0], &polyVerts[1], &polyVerts[2], &newNormal);
    f32 newNormMagnitude = Math3D_Vec3fMagnitude(&newNormal);

    if (!IS_ZERO(newNormMagnitude)) {
      newNormal = newNormal * (1.0f / newNormMagnitude);
      newPoly.nx = (s16)(newNormal.x * SHT_MAX);
      newPoly.ny = (s16)(newNormal.y * SHT_MAX);
      newPoly.nz = (s16)(newNormal.z * SHT_MAX);
    }

    newPoly.dist = -DOTXYZ(newNormal, polyVerts[0]);
    if (newNormal.y > 0.5f) {
      dyna.floors.push_back(newPoly);
    } else if (newNormal.y < -0.8f) {
      dyna.ceilings.push_back(newPoly);
    } else {
      dyna.walls.push_back(newPoly);
    }
  }
  std::reverse(dyna.floors.begin(), dyna.floors.end());
  std::reverse(dyna.ceilings.begin(), dyna.ceilings.end());
  std::reverse(dyna.walls.begin(), dyna.walls.end());

  dyna.vtxList = header->vertices;
  this->dynas.push_back(dyna);
}

Vec3f Collision::runChecks(Vec3f prevPos, Vec3f intendedPos,
                           CollisionPoly** wallPoly, CollisionPoly** floorPoly,
                           f32* floorHeight) {
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
  if (BgCheck_RaycastDownImpl(this, checkPos, floorHeight, floorPoly)) {
    f32 floorHeightDiff = *floorHeight - intendedPos.y;
    if (floorHeightDiff >= 0.0f) {  // actor is on or below the ground
      intendedPos.y = *floorHeight;
    }
  }

  return intendedPos;
}

Vec3f Collision::runChecks(Vec3f prevPos, Vec3f intendedPos) {
  CollisionPoly* wallPoly;
  CollisionPoly* floorPoly;
  f32 floorHeight;
  return runChecks(prevPos, intendedPos, &wallPoly, &floorPoly, &floorHeight);
}

Vec3f Collision::findFloor(Vec3f pos) {
  f32 floorHeight;
  CollisionPoly* poly;
  BgCheck_RaycastDownImpl(this, pos, &floorHeight, &poly);
  return Vec3f(pos.x, floorHeight, pos.z);
}

Vec3f Collision::entityLineTest(Vec3f pos, Vec3f target, bool checkWalls,
                                bool checkFloors, CollisionPoly** outPoly) {
  *outPoly = NULL;
  BgCheck_CheckLineImpl(this, pos, target, checkWalls, checkFloors, true,
                        &target, outPoly);
  return target;
}

Vec3f Collision::cameraLineTest(Vec3f pos, Vec3f target,
                                CollisionPoly** outPoly) {
  *outPoly = NULL;
  BgCheck_CheckLineImpl(this, pos, target, true, true, true, &target, outPoly);
  return target;
}

f32 Collision::cameraFindFloor(Vec3f pos, CollisionPoly** outPoly) {
  *outPoly = NULL;
  f32 floorHeight;
  BgCheck_RaycastDownImpl(this, pos, &floorHeight, outPoly);
  return floorHeight;
}
