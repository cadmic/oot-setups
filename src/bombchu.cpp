#include "bombchu.hpp"

#include "sys_math.hpp"
#include "sys_math3d.hpp"
#include "sys_matrix.hpp"

Bombchu::Bombchu(Collision* col, Vec3f pos, u16 angle) {
  this->col = col;
  this->pos = pos;
  this->rot = {0, (s16)angle, 0};
  this->axisForwards = {Math_SinS(angle), 0.0f, Math_CosS(angle)};
  this->axisUp = {0.0f, 1.0f, 0.0f};
  this->axisLeft = {Math_SinS(angle + 0x4000), 0.0f, Math_CosS(angle + 0x4000)};

  CollisionPoly* poly;
  int dynaId;
  col->findFloor({pos.x, pos.y + 50.0f, pos.z}, &poly, &dynaId);
  if (poly != NULL) {
    this->updateFloorPoly(poly);
  } else {
    this->floorPoly = NULL;
  }
}

void Bombchu::updateFloorPoly(CollisionPoly* poly) {
  Vec3f normal = CollisionPoly_GetNormalF(poly);
  f32 normDotUp = DOTXYZ(normal, this->axisUp);
  f32 angle = Math_FAcosF(normDotUp);
  if (angle >= 0.001f) {
    Vec3f vec;
    Math3D_Vec3f_Cross(&this->axisUp, &normal, &vec);
    //! @bug this function expects a unit vector but `vec` is not normalized
    Matrix_RotateAxis(angle, &vec, MTXMODE_NEW);
    Matrix_MultVec3f(&this->axisLeft, &vec);
    this->axisLeft = vec;

    Math3D_Vec3f_Cross(&this->axisLeft, &normal, &this->axisForwards);

    f32 magnitude = Math3D_Vec3fMagnitude(&this->axisForwards);
    this->axisForwards = this->axisForwards * (1.0f / magnitude);

    this->axisUp = normal;

    // mf = (axisLeft | axisUp | axisForwards)
    MtxF mf;
    memset(&mf, 0, sizeof(mf));
    mf.xx = this->axisLeft.x;
    mf.yx = this->axisLeft.y;
    mf.zx = this->axisLeft.z;
    mf.xy = this->axisUp.x;
    mf.yy = this->axisUp.y;
    mf.zy = this->axisUp.z;
    mf.xz = this->axisForwards.x;
    mf.yz = this->axisForwards.y;
    mf.zz = this->axisForwards.z;

    Matrix_MtxFToYXZRotS(&mf, &this->rot, 0);

    // Hack?
    this->rot.x = -this->rot.x;
  }

  this->floorPoly = poly;
}

bool Bombchu::lineTest(Vec3f posA, Vec3f posB, Vec3f* outPos,
                       CollisionPoly** outPoly) {
  *outPos = this->col->entityLineTest(posA, posB, true, true, true, outPoly);
  return *outPoly != NULL;
}

bool Bombchu::move() {
  Vec3f posUp = this->pos + this->axisUp * 2.0f;
  Vec3f posDown = this->pos - this->axisUp * 4.0f;

  f32 speed = 0.0f;
  Vec3f upDownPos;
  CollisionPoly* upDownPoly;
  Vec3f sidePos;
  CollisionPoly* sidePoly;
  CollisionPoly* newFloorPoly = NULL;

  if (this->floorPoly == NULL) {
    // Emulate VC and explode immediately
    return false;
  }

  if (lineTest(posUp, posDown, &upDownPos, &upDownPoly)) {
    if (lineTest(posUp, posUp + axisForwards * 16.0f, &sidePos, &sidePoly)) {
      this->pos = sidePos;
      newFloorPoly = sidePoly;
    } else {
      this->pos = upDownPos;
      newFloorPoly = upDownPoly;
      speed = 8.0f;
    }
  } else if (lineTest(posDown, posDown - axisForwards * 48.0f, &sidePos,
                      &sidePoly)) {
    this->pos = sidePos;
    newFloorPoly = sidePoly;
  } else if (lineTest(posDown, posDown + axisLeft * 48.0f, &sidePos,
                      &sidePoly)) {
    this->pos = sidePos;
    newFloorPoly = sidePoly;
  } else if (lineTest(posDown, posDown - axisLeft * 48.0f, &sidePos,
                      &sidePoly)) {
    this->pos = sidePos;
    newFloorPoly = sidePoly;
  } else {
    return false;
  }

  if (newFloorPoly != this->floorPoly) {
    this->updateFloorPoly(newFloorPoly);
  }

  f32 speedXZ = Math_CosS(rot.x) * speed;
  Vec3f velocity = {Math_SinS(rot.y) * speedXZ, Math_SinS(rot.x) * speed,
                    Math_CosS(rot.y) * speedXZ};
  this->pos = this->pos + velocity * 1.5f;

  return true;
}
