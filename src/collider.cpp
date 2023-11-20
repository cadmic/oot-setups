#include "collider.hpp"

s16 linkRadius = 12;

Vec3f calculatePush(Vec3s linkPos, Vec3s objectPos, s16 objectRadius,
                    f32 dispRatio) {
  f32 xDelta = linkPos.x - objectPos.x;
  f32 zDelta = linkPos.z - objectPos.z;

  f32 xzDist = sqrtf(SQ(xDelta) + SQ(zDelta));
  f32 overlap = linkRadius + objectRadius - xzDist;

  if (overlap <= 0) {
    return Vec3f();
  }

  if (xzDist != 0.0f) {
    xDelta *= overlap / xzDist;
    zDelta *= overlap / xzDist;
    return Vec3f(xDelta * dispRatio, 0, zDelta * dispRatio);
  } else {
    return Vec3f(-overlap * dispRatio, 0, 0);
  }
}

Vec3f bombPush(Vec3f linkPos, Vec3f bombPos) {
  return calculatePush(linkPos.toVec3s(), bombPos.toVec3s(), 6, 0.8f);
}

Vec3f immovablePush(Vec3f linkPos, Vec3f objectPos, s16 objectRadius) {
  return calculatePush(linkPos.toVec3s(), objectPos.toVec3s(), objectRadius,
                       1.0f);
}

bool colliderSphVsQuad(Sphere16* sph, Vec3f* quad) {
  TriNorm tri1;
  TriNorm tri2;
  Vec3f hitPos;

  Math3D_TriNorm(&tri1, &quad[2], &quad[3], &quad[1]);
  Math3D_TriNorm(&tri2, &quad[1], &quad[0], &quad[2]);
  return Math3D_TriVsSphIntersect(sph, &tri1, &hitPos) ||
         Math3D_TriVsSphIntersect(sph, &tri2, &hitPos);
}
