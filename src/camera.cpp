#include "camera.hpp"

#include "camera_data.hpp"
#include "olib.hpp"
#include "sys_math.hpp"

#define ABS std::abs
#define CAM_DATA_SCALED(x) ((x)*0.01f)

const f32 xzOffsetUpdateRate = 0.05f;
const f32 yOffsetUpdateRate = 0.05f;

f32 Camera_InterpolateCurve(f32 a, f32 b) {
  f32 ret;
  f32 absB;
  f32 t = 0.4f;
  f32 t2;
  f32 t3;
  f32 t4;

  absB = fabsf(b);
  if (a < absB) {
    ret = 1.0f;
  } else {
    t2 = 1.0f - t;
    if ((a * t2) > absB) {
      t3 = SQ(b) * (1.0f - t);
      t4 = SQ(a * t2);
      ret = t3 / t4;
    } else {
      t3 = SQ(a - absB) * t;
      t4 = SQ(0.4f * a);
      ret = 1.0f - (t3 / t4);
    }
  }
  return ret;
}

f32 Camera_LERPCeilF(f32 target, f32 cur, f32 stepScale, f32 minDiff) {
  f32 diff = target - cur;
  f32 step;
  f32 ret;

  if (fabsf(diff) >= minDiff) {
    step = diff * stepScale;
    ret = cur + step;
  } else {
    ret = target;
  }

  return ret;
}

f32 Camera_LERPFloorF(f32 target, f32 cur, f32 stepScale, f32 minDiff) {
  f32 diff = target - cur;
  f32 step;
  f32 ret;

  if (fabsf(diff) >= minDiff) {
    step = diff * stepScale;
    ret = cur + step;
  } else {
    ret = cur;
  }

  return ret;
}

s16 Camera_LERPCeilS(s16 target, s16 cur, f32 stepScale, s16 minDiff) {
  s16 diff = target - cur;
  s16 step;
  s32 ret;

  if (ABS(diff) >= minDiff) {
    step = diff * stepScale + 0.5f;
    ret = cur + step;
  } else {
    ret = target;
  }

  return ret;
}

s16 Camera_LERPFloorS(s16 target, s16 cur, f32 stepScale, s16 minDiff) {
  s16 diff = target - cur;
  s16 step;
  s32 ret;

  if (ABS(diff) >= minDiff) {
    step = diff * stepScale + 0.5f;
    ret = cur + step;
  } else {
    ret = cur;
  }

  return ret;
}

void Camera_LERPCeilVec3f(Vec3f* target, Vec3f* cur, f32 yStepScale,
                          f32 xzStepScale, f32 minDiff) {
  cur->x = Camera_LERPCeilF(target->x, cur->x, xzStepScale, minDiff);
  cur->y = Camera_LERPCeilF(target->y, cur->y, yStepScale, minDiff);
  cur->z = Camera_LERPCeilF(target->z, cur->z, xzStepScale, minDiff);
}

Vec3f* Camera_AddVecGeoToVec3f(Vec3f* dest, Vec3f* a, VecGeo* geo) {
  Vec3f sum;
  Vec3f b;

  OLib_VecGeoToVec3f(&b, geo);

  sum.x = a->x + b.x;
  sum.y = a->y + b.y;
  sum.z = a->z + b.z;

  *dest = sum;

  return dest;
}

bool Camera_BGCheckInfo(Camera* camera, Collision* col, Vec3f from, Vec3f to,
                        Vec3f* result, Vec3f* normal) {
  VecGeo fromToOffset;
  OLib_Vec3fDiffToVecGeo(&fromToOffset, &from, &to);
  fromToOffset.r += 8.0f;

  Vec3f toPoint;
  Camera_AddVecGeoToVec3f(&toPoint, &from, &fromToOffset);

  Vec3f lineResult = col->cameraLineTest(from, toPoint, &camera->wallPoly);

  if (camera->wallPoly) {
    *normal = CollisionPoly_GetNormalF(camera->wallPoly);
    *result = lineResult + *normal;
    return true;
  } else {
    // TODO: check floors
    Vec3f fromToNorm;
    OLib_Vec3fDistNormalize(&fromToNorm, &from, &to);
    *normal = fromToNorm * -1.0f;
    *result = to + *normal;
    return false;
  }
}

f32 Camera_GetFloorYLayer(Camera* camera, Collision* col, Vec3f pos,
                          f32 playerGroundY) {
  f32 floorY = col->cameraFindFloor(pos, &camera->floorPoly);
  if (camera->floorPoly) {
    Vec3f normal = CollisionPoly_GetNormalF(camera->floorPoly);
    if (playerGroundY < floorY && normal.y <= 0.5f) {
      // player is below the floor and floor is considered steep
      return BGCHECK_Y_MIN;
    }
  }
  return floorY;
}

s16 Camera_GetPitchAdjFromFloorHeightDiffs(Camera* camera, Collision* col,
                                           s16 viewYaw) {
  // Assume standing on ground
  f32 playerGroundY = camera->playerPos.y;

  Vec3f viewForwards = {Math_SinS(viewYaw), 0.0f, Math_CosS(viewYaw)};

  f32 checkOffsetY = 1.2f * camera->playerHeight;
  f32 nearDist = camera->playerHeight;
  f32 farDist = 2.5f * camera->playerHeight;

  Vec3f playerPos = camera->playerPos;
  playerPos.y = playerGroundY + checkOffsetY;

  if (camera->frames % 2 == 0) {
    Vec3f testPos = playerPos + viewForwards * farDist;
    Camera_BGCheckInfo(camera, col, playerPos, testPos, &camera->pitchTestPos,
                       &camera->pitchTestNormal);
  } else {
    farDist = OLib_Vec3fDistXZ(&playerPos, &camera->pitchTestPos);
    camera->pitchTestPos =
        camera->pitchTestPos + camera->pitchTestNormal * 5.0f;

    if (nearDist > farDist) {
      nearDist = farDist;
      camera->floorYNear = camera->floorYFar = Camera_GetFloorYLayer(
          camera, col, camera->pitchTestPos, playerGroundY);
    } else {
      Vec3f nearPos = playerPos + viewForwards * nearDist;

      camera->floorYNear =
          Camera_GetFloorYLayer(camera, col, nearPos, playerGroundY);
      camera->floorYFar = Camera_GetFloorYLayer(
          camera, col, camera->pitchTestPos, playerGroundY);
    }

    if (camera->floorYNear == BGCHECK_Y_MIN) {
      camera->floorYNear = playerGroundY;
    }

    if (camera->floorYFar == BGCHECK_Y_MIN) {
      camera->floorYFar = camera->floorYNear;
    }
  }

  f32 floorYDiffNear = 0.8f * (camera->floorYNear - playerGroundY);
  f32 floorYDiffFar = 0.2f * (camera->floorYFar - playerGroundY);

  f32 pitchNear =
      CAM_DEG_TO_BINANG(RAD_TO_DEG(Math_FAtan2F(floorYDiffNear, nearDist)));
  f32 pitchFar =
      CAM_DEG_TO_BINANG(RAD_TO_DEG(Math_FAtan2F(floorYDiffFar, farDist)));

  return pitchNear + pitchFar;
}

f32 Camera_ClampLERPScale(Camera* camera, f32 maxLERPScale) {
  if (camera->atLERPStepScale < 0.12f) {
    return 0.12f;
  } else if (camera->atLERPStepScale >= maxLERPScale) {
    return maxLERPScale;
  } else {
    return camera->atLERPStepScale * 1.1f;
  }
}

void Camera_CalcAtDefault(Camera* camera, f32 yOffset) {
  Vec3f atOffsetTarget = {0, camera->playerHeight + yOffset, 0};
  Camera_LERPCeilVec3f(&atOffsetTarget, &camera->atOffset, yOffsetUpdateRate,
                       xzOffsetUpdateRate, 0.1f);
  Vec3f atTarget = camera->playerPos + camera->atOffset;
  Camera_LERPCeilVec3f(&atTarget, &camera->at, camera->atLERPStepScale,
                       camera->atLERPStepScale, 0.2f);
}

void Camera_ClampDist(Camera* camera, f32 dist, f32 minDist, f32 maxDist) {
  if (camera->rUpdateRateTimer != 0) {
    camera->rUpdateRateTimer--;
  }

  f32 distTarget;
  f32 rUpdateRateInvTarget;
  if (dist < minDist) {
    distTarget = minDist;
    rUpdateRateInvTarget = camera->rUpdateRateTimer != 0 ? 10.0f : 20.0f;
  } else if (dist > maxDist) {
    distTarget = maxDist;
    rUpdateRateInvTarget = camera->rUpdateRateTimer != 0 ? 10.0f : 20.0f;
  } else {
    distTarget = dist;
    rUpdateRateInvTarget = camera->rUpdateRateTimer != 0 ? 20.0f : 1.0f;
  }
  camera->rUpdateRateInv = Camera_LERPCeilF(rUpdateRateInvTarget,
                                            camera->rUpdateRateInv, 0.5f, 0.1f);
  camera->dist = Camera_LERPCeilF(distTarget, camera->dist,
                                  1.0f / camera->rUpdateRateInv, 0.2f);
}

s16 Camera_CalcDefaultPitch(Camera* camera, s16 pitch, s16 pitchTarget,
                            s16 slopePitchAdj) {
  f32 pad;
  f32 stepScale;
  f32 t;
  s16 phi_v0;
  s16 absCur;
  s16 target;

  absCur = ABS(pitch);
  phi_v0 = slopePitchAdj > 0 ? (s16)(Math_CosS(slopePitchAdj) * slopePitchAdj)
                             : slopePitchAdj;
  target = pitchTarget - phi_v0;

  if (ABS(target) < absCur) {
    stepScale = (1.0f / camera->pitchUpdateRateInv) * 3.0f;
  } else {
    t = absCur * (1.0f / 14500.0f);
    pad = Camera_InterpolateCurve(0.8f, 1.0f - t);
    stepScale = (1.0f / camera->pitchUpdateRateInv) * pad;
  }
  return Camera_LERPCeilS(target, pitch, stepScale, 0xA);
}

u16 Camera_CalcDefaultYaw(Camera* camera, u16 cur, u16 target, f32 maxYawUpdate,
                          f32 accel) {
  s16 angDelta = target - (cur - 0x7FFF);
  f32 speedT;
  if (camera->xzSpeed > 0.001f) {
    speedT = (s16)((u16)angDelta - 0x7FFF) * (1.0f / 32767.0f);
  } else {
    speedT = 0.3f;
  }

  f32 updSpeed = Camera_InterpolateCurve(maxYawUpdate, speedT);
  f32 velocity = updSpeed + (1.0f - updSpeed) * accel;
  if (velocity < 0.0f) {
    velocity = 0.0f;
  }

  f32 velFactor = Camera_InterpolateCurve(0.5f, camera->speedRatio);
  f32 yawUpdRate = 1.0f / camera->yawUpdateRateInv;
  return cur + (s16)(angDelta * velocity * velFactor * yawUpdRate);
}

Camera::Camera(PlayerAge age) {
  this->playerHeight = age == PLAYER_AGE_CHILD ? 44.0f : 68.0f;
}

u16 Camera::yaw() {
  VecGeo result;
  OLib_Vec3fDiffToVecGeo(&result, &this->eye, &this->at);
  return result.yaw;
}

void Camera::initParallel1(Vec3f pos, u16 angle, int setting) {
  CameraZParallelSettings* settings = &cameraZParallelSettings[setting];
  f32 yNormal = 1.0f - 0.1f - (-0.1f * (68.0f / this->playerHeight));
  f32 yOffset =
      CAM_DATA_SCALED(settings->yOffset) * this->playerHeight * yNormal;
  f32 dist = CAM_DATA_SCALED(settings->dist) * this->playerHeight * yNormal;

  this->setting = setting;
  this->mode = 1;
  this->frames = -1;

  this->playerPos = pos;
  this->atOffset = Vec3f(0, this->playerHeight + yOffset, 0);
  this->at = pos + this->atOffset;

  VecGeo target = {
      .r = dist,
      .pitch = 0,
      .yaw = (s16)(angle - 0x7fff),
  };

  Vec3f dir;
  OLib_VecGeoToVec3f(&dir, &target);
  this->eyeNext = this->at + dir;

  Vec3f norm;
  OLib_Vec3fDistNormalize(&norm, &this->at, &this->eyeNext);
  this->eye = this->eyeNext - norm;

  this->xzSpeed = 0;
  this->dist = dist;
  this->speedRatio = 0;

  // Not directly set, but we reach these values eventually if Link moves
  // around
  this->rUpdateRateInv = 20.0f;
  this->pitchUpdateRateInv = 2.0f;
  this->yawUpdateRateInv = 5.0f;
  this->atLERPStepScale = 0.5f;

  this->yawUpdateRateTarget = 0;
  this->rUpdateRateTimer = 10;
  this->slopePitchAdj = 0;

  this->floorYNear = pos.y;
  this->floorYFar = pos.y;

  this->wallPoly = NULL;
  this->floorPoly = NULL;
}

void Camera::updateNormal1(Collision* col, Vec3f pos, u16 angle, int setting) {
  CameraNormalSettings* settings = &cameraNormalSettings[setting];
  f32 yNormal = 1.0f - 0.1f - (-0.1f * (68.0f / this->playerHeight));
  f32 t = yNormal * (this->playerHeight * 0.01f);
  f32 yOffset = settings->yOffset * t;
  f32 distMin = settings->distMin * t;
  f32 distMax = settings->distMax * t;
  f32 pitchTarget = CAM_DEG_TO_BINANG(settings->pitchTarget);
  f32 yawUpdateRateTarget = settings->yawUpdateRateTarget;
  f32 pitchUpdateRateTarget = settings->pitchUpdateRateTarget;
  f32 maxYawUpdate = CAM_DATA_SCALED(settings->maxYawUpdate);
  f32 atLERPScaleMax = CAM_DATA_SCALED(settings->atLerpStepScale);

  this->frames++;

  Vec3f prevPos = this->playerPos;
  this->playerPos = pos;

  f32 xzSpeed = OLib_Vec3fDistXZ(&pos, &prevPos);

  if (this->mode != 0 || setting != this->setting) {
    this->setting = setting;
    this->mode = 0;

    this->slopePitchAdj = 0;
    this->rUpdateRateTimer = 10;
    // TODO: split this into this->xzSpeed and a replacement for rwData->unk_20
    this->xzSpeed = xzSpeed;
    this->yawUpdateRateTarget = yawUpdateRateTarget;
  }

  f32 prevXZSpeed = this->xzSpeed;
  this->xzSpeed = xzSpeed;
  this->speedRatio = OLib_ClampMaxDist(this->xzSpeed / 9.0f, 1.0f);

  VecGeo atEyeGeo;
  OLib_Vec3fDiffToVecGeo(&atEyeGeo, &this->at, &this->eye);

  VecGeo atEyeNextGeo;
  OLib_Vec3fDiffToVecGeo(&atEyeNextGeo, &this->at, &this->eyeNext);

  s16 slopePitchTarget =
      Camera_GetPitchAdjFromFloorHeightDiffs(this, col, atEyeGeo.yaw - 0x7FFF);
  f32 pitchAdjStep =
      ((1.0f / pitchUpdateRateTarget) * 0.5f) +
      ((1.0f / pitchUpdateRateTarget) * 0.5f) * (1.0f - this->speedRatio);
  this->slopePitchAdj =
      Camera_LERPCeilS(slopePitchTarget, this->slopePitchAdj, pitchAdjStep, 15);

  Camera_CalcAtDefault(this, yOffset);

  VecGeo eyeAdjustment;
  OLib_Vec3fDiffToVecGeo(&eyeAdjustment, &this->at, &this->eyeNext);

  Camera_ClampDist(this, eyeAdjustment.r, distMin, distMax);
  eyeAdjustment.r = this->dist;

  f32 accel = (xzSpeed - prevXZSpeed) * 0.333333f;
  //   if (accel > 1.0f) {
  //     accel = 1.0f;
  //   }
  // bug? previous if statement has no effect
  if (accel > -1.0f) {
    accel = -1.0f;
  }

  this->yawUpdateRateInv = Camera_LERPCeilF(
      this->yawUpdateRateTarget - 0.7f * this->yawUpdateRateTarget * accel,
      this->yawUpdateRateInv, this->speedRatio * 0.5f, 0.1f);
  this->pitchUpdateRateInv = Camera_LERPCeilF(16.0f, this->pitchUpdateRateInv,
                                              this->speedRatio * 0.2f, 0.1f);
  // bug? pitchUpdateRateInv is updated twice
  this->pitchUpdateRateInv = Camera_LERPCeilF(16.0f, this->pitchUpdateRateInv,
                                              this->speedRatio * 0.2f, 0.1f);

  eyeAdjustment.yaw =
      Camera_CalcDefaultYaw(this, atEyeNextGeo.yaw, angle, maxYawUpdate, accel);
  eyeAdjustment.pitch = Camera_CalcDefaultPitch(
      this, atEyeNextGeo.pitch, pitchTarget, this->slopePitchAdj);
  if (eyeAdjustment.pitch > 0x38A4) {
    eyeAdjustment.pitch = 0x38A4;
  }
  if (eyeAdjustment.pitch < -0x3C8C) {
    eyeAdjustment.pitch = -0x3C8C;
  }

  Camera_AddVecGeoToVec3f(&this->eyeNext, &this->at, &eyeAdjustment);

  Vec3f collisionPoint;
  Vec3f collisionNormal;
  // TODO: check both ways
  if (Camera_BGCheckInfo(this, col, this->at, this->eyeNext, &collisionPoint,
                         &collisionNormal)) {
    VecGeo geoNorm;
    OLib_Vec3fToVecGeo(&geoNorm, &collisionNormal);
    if (geoNorm.pitch >= 0x2EE1) {
      geoNorm.yaw = eyeAdjustment.yaw;
    }

    f32 collisionDist = OLib_Vec3fDist(&this->at, &collisionPoint);
    f32 collisionDistRatio =
        collisionDist > distMin ? 1.0f : collisionDist / distMin;
    this->yawUpdateRateTarget = collisionDistRatio * yawUpdateRateTarget;

    this->eye = collisionPoint + collisionNormal;
    if (collisionDist < 30.0f) {
      VecGeo offset;
      offset.yaw = eyeAdjustment.yaw;
      offset.pitch = Math_SinS(geoNorm.pitch + 0x3FFF) * 16380.0f;
      offset.r = (30.0f - collisionDist) * 1.2f;
      Camera_AddVecGeoToVec3f(&this->eye, &this->eye, &offset);
    }
  } else {
    this->yawUpdateRateTarget = yawUpdateRateTarget;
    this->eye = collisionPoint + collisionNormal;
  }

  this->atLERPStepScale = Camera_ClampLERPScale(this, atLERPScaleMax);
}
