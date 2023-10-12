#include "camera.hpp"

#include "camera_data.hpp"
#include "olib.hpp"
#include "sys_math.hpp"

#define ABS std::abs
#define CAM_DATA_SCALED(x) ((x)*0.01f)

// a - b without implementation-defined overflow behavior
s16 angleDiff(u16 a, u16 b) {
  u16 diff = a - b;
  if (diff >= 0x8000) {
    return diff - 0x10000;
  } else {
    return diff;
  }
}

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

bool Camera_BGCheckInfo(Camera* camera, Vec3f from, Vec3f to, Vec3f* result,
                        Vec3f* normal) {
  VecGeo fromToOffset;
  OLib_Vec3fDiffToVecGeo(&fromToOffset, &from, &to);
  fromToOffset.r += 8.0f;

  Vec3f toPoint;
  Camera_AddVecGeoToVec3f(&toPoint, &from, &fromToOffset);

  Vec3f lineResult =
      camera->col->cameraLineTest(from, toPoint, &camera->wallPoly);

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

f32 Camera_GetFloorYLayer(Camera* camera, Vec3f pos, f32 playerGroundY) {
  f32 floorY = camera->col->cameraFindFloor(pos, &camera->floorPoly);
  if (camera->floorPoly) {
    Vec3f normal = CollisionPoly_GetNormalF(camera->floorPoly);
    if (playerGroundY < floorY && normal.y <= 0.5f) {
      // player is below the floor and floor is considered steep
      return BGCHECK_Y_MIN;
    }
  }
  return floorY;
}

s16 Camera_GetPitchAdjFromFloorHeightDiffs(Camera* camera, s16 viewYaw) {
  // TODO: we assume player is standing on ground
  f32 playerGroundY = camera->playerPos.y;

  Vec3f viewForwards = {Math_SinS(viewYaw), 0.0f, Math_CosS(viewYaw)};

  f32 checkOffsetY = 1.2f * camera->playerHeight;
  f32 nearDist = camera->playerHeight;
  f32 farDist = 2.5f * camera->playerHeight;

  Vec3f playerPos = camera->playerPos;
  playerPos.y = playerGroundY + checkOffsetY;

  if (camera->frames % 2 == 0) {
    Vec3f testPos = playerPos + viewForwards * farDist;
    Camera_BGCheckInfo(camera, playerPos, testPos, &camera->pitchTestPos,
                       &camera->pitchTestNormal);
  } else {
    farDist = OLib_Vec3fDistXZ(&playerPos, &camera->pitchTestPos);
    camera->pitchTestPos =
        camera->pitchTestPos + camera->pitchTestNormal * 5.0f;

    if (nearDist > farDist) {
      nearDist = farDist;
      camera->floorYNear = camera->floorYFar =
          Camera_GetFloorYLayer(camera, camera->pitchTestPos, playerGroundY);
    } else {
      Vec3f nearPos = playerPos + viewForwards * nearDist;

      camera->floorYNear =
          Camera_GetFloorYLayer(camera, nearPos, playerGroundY);
      camera->floorYFar =
          Camera_GetFloorYLayer(camera, camera->pitchTestPos, playerGroundY);
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

void Camera_CalcAtForParallel(Camera* camera, f32 yOffset) {
  // TODO: we assume player is standing on ground. This ends up being identical
  // to Camera_CalcAtDefault
  Vec3f atOffsetTarget = {0, camera->playerHeight + yOffset, 0};
  Camera_LERPCeilVec3f(&atOffsetTarget, &camera->atOffset, yOffsetUpdateRate,
                       xzOffsetUpdateRate, 0.1f);
  Vec3f atTarget = camera->playerPos + camera->atOffset;
  Camera_LERPCeilVec3f(&atTarget, &camera->at, camera->atLERPStepScale,
                       camera->atLERPStepScale, 0.2f);
}

void Camera_ClampDist(Camera* camera, f32 dist, f32 minDist, f32 maxDist,
                      int timer) {
  f32 distTarget;
  f32 rUpdateRateInvTarget;
  if (dist < minDist) {
    distTarget = minDist;
    rUpdateRateInvTarget = timer != 0 ? 10.0f : 20.0f;
  } else if (dist > maxDist) {
    distTarget = maxDist;
    rUpdateRateInvTarget = timer != 0 ? 10.0f : 20.0f;
  } else {
    distTarget = dist;
    rUpdateRateInvTarget = timer != 0 ? 20.0f : 1.0f;
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
  s16 angDelta = angleDiff(target, cur - 0x7FFF);
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

Camera::Camera(Collision* col) {
  memset(this, 0, sizeof(Camera));
  this->col = col;
  this->playerHeight = col->age == PLAYER_AGE_CHILD ? 44.0f : 68.0f;
}

void Camera::initParallel(Vec3f pos, u16 angle, int setting) {
  CameraZParallelSettings* settings = &cameraZParallelSettings[setting];
  f32 yNormal = 1.0f - 0.1f - (-0.1f * (68.0f / this->playerHeight));
  f32 yOffset =
      CAM_DATA_SCALED(settings->yOffset) * this->playerHeight * yNormal;
  f32 dist = CAM_DATA_SCALED(settings->dist) * this->playerHeight * yNormal;

  this->setting = setting;
  this->mode = 1;
  this->modeChangeDisallowed = false;
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
  this->speedRatio = 0;

  // Not directly set, but we reach these values eventually if the player moves
  // around a bit
  this->dist = dist;
  this->rUpdateRateInv = 20.0f;
  this->pitchUpdateRateInv = 2.0f;
  this->yawUpdateRateInv = 5.0f;
  this->atLERPStepScale = 0.5f;

  this->parallelAnimTimer = 0;
  this->parallelYawTarget = angle - 0x7FFF;

  this->floorYNear = pos.y;
  this->floorYFar = pos.y;

  this->wallPoly = NULL;
  this->floorPoly = NULL;
}

void Camera_Normal1(Camera* camera, Vec3f pos, u16 angle, int setting) {
  CameraNormalSettings* settings = &cameraNormalSettings[setting];
  f32 yNormal = 1.0f - 0.1f - (-0.1f * (68.0f / camera->playerHeight));
  f32 t = yNormal * (camera->playerHeight * 0.01f);
  f32 yOffset = settings->yOffset * t;
  f32 distMin = settings->distMin * t;
  f32 distMax = settings->distMax * t;
  f32 pitchTarget = CAM_DEG_TO_BINANG(settings->pitchTarget);
  f32 yawUpdateRateTarget = settings->yawUpdateRateTarget;
  f32 pitchUpdateRateTarget = settings->pitchUpdateRateTarget;
  f32 maxYawUpdate = CAM_DATA_SCALED(settings->maxYawUpdate);
  f32 atLERPScaleMax = CAM_DATA_SCALED(settings->atLerpStepScale);

  if (camera->mode != 0 || setting != camera->setting) {
    camera->setting = setting;
    camera->mode = 0;

    camera->normalPrevXZSpeed = camera->xzSpeed;
    camera->normalRUpdateRateTimer = 10;
    camera->normalYawUpdateRateTarget = yawUpdateRateTarget;
    camera->normalSlopePitchAdj = 0;
  }

  if (camera->normalRUpdateRateTimer != 0) {
    camera->normalRUpdateRateTimer--;
  }

  VecGeo atEyeGeo;
  OLib_Vec3fDiffToVecGeo(&atEyeGeo, &camera->at, &camera->eye);

  VecGeo atEyeNextGeo;
  OLib_Vec3fDiffToVecGeo(&atEyeNextGeo, &camera->at, &camera->eyeNext);

  s16 slopePitchTarget =
      Camera_GetPitchAdjFromFloorHeightDiffs(camera, atEyeGeo.yaw - 0x7FFF);
  f32 pitchAdjStep =
      ((1.0f / pitchUpdateRateTarget) * 0.5f) +
      ((1.0f / pitchUpdateRateTarget) * 0.5f) * (1.0f - camera->speedRatio);
  camera->normalSlopePitchAdj = Camera_LERPCeilS(
      slopePitchTarget, camera->normalSlopePitchAdj, pitchAdjStep, 15);

  Camera_CalcAtDefault(camera, yOffset);

  VecGeo eyeAdjustment;
  OLib_Vec3fDiffToVecGeo(&eyeAdjustment, &camera->at, &camera->eyeNext);

  Camera_ClampDist(camera, eyeAdjustment.r, distMin, distMax,
                   camera->normalRUpdateRateTimer);
  eyeAdjustment.r = camera->dist;

  f32 accel = (camera->xzSpeed - camera->normalPrevXZSpeed) * 0.333333f;
  //   if (accel > 1.0f) {
  //     accel = 1.0f;
  //   }
  // bug? previous if statement has no effect
  if (accel > -1.0f) {
    accel = -1.0f;
  }

  camera->normalPrevXZSpeed = camera->xzSpeed;

  camera->yawUpdateRateInv = Camera_LERPCeilF(
      camera->normalYawUpdateRateTarget -
          0.7f * camera->normalYawUpdateRateTarget * accel,
      camera->yawUpdateRateInv, camera->speedRatio * 0.5f, 0.1f);
  camera->pitchUpdateRateInv = Camera_LERPCeilF(
      16.0f, camera->pitchUpdateRateInv, camera->speedRatio * 0.2f, 0.1f);
  // bug? pitchUpdateRateInv is updated twice
  camera->pitchUpdateRateInv = Camera_LERPCeilF(
      16.0f, camera->pitchUpdateRateInv, camera->speedRatio * 0.2f, 0.1f);

  eyeAdjustment.yaw = Camera_CalcDefaultYaw(camera, atEyeNextGeo.yaw, angle,
                                            maxYawUpdate, accel);
  eyeAdjustment.pitch = Camera_CalcDefaultPitch(
      camera, atEyeNextGeo.pitch, pitchTarget, camera->normalSlopePitchAdj);
  if (eyeAdjustment.pitch > 0x38A4) {
    eyeAdjustment.pitch = 0x38A4;
  }
  if (eyeAdjustment.pitch < -0x3C8C) {
    eyeAdjustment.pitch = -0x3C8C;
  }

  Camera_AddVecGeoToVec3f(&camera->eyeNext, &camera->at, &eyeAdjustment);

  Vec3f collisionPoint;
  Vec3f collisionNormal;
  // TODO: check both ways?
  if (Camera_BGCheckInfo(camera, camera->at, camera->eyeNext, &collisionPoint,
                         &collisionNormal)) {
    VecGeo geoNorm;
    OLib_Vec3fToVecGeo(&geoNorm, &collisionNormal);
    if (geoNorm.pitch >= 0x2EE1) {
      geoNorm.yaw = eyeAdjustment.yaw;
    }

    f32 collisionDist = OLib_Vec3fDist(&camera->at, &collisionPoint);
    f32 collisionDistRatio =
        collisionDist > distMin ? 1.0f : collisionDist / distMin;
    camera->normalYawUpdateRateTarget =
        collisionDistRatio * yawUpdateRateTarget;

    camera->eye = collisionPoint + collisionNormal;
    if (collisionDist < 30.0f) {
      VecGeo offset;
      offset.yaw = eyeAdjustment.yaw;
      offset.pitch = Math_SinS(geoNorm.pitch + 0x3FFF) * 16380.0f;
      offset.r = (30.0f - collisionDist) * 1.2f;
      Camera_AddVecGeoToVec3f(&camera->eye, &camera->eye, &offset);
    }
  } else {
    camera->normalYawUpdateRateTarget = yawUpdateRateTarget;
    camera->eye = collisionPoint + collisionNormal;
  }

  camera->atLERPStepScale = Camera_ClampLERPScale(camera, atLERPScaleMax);
}

void Camera_Parallel1(Camera* camera, Vec3f pos, u16 angle, int setting) {
  CameraZParallelSettings* settings = &cameraZParallelSettings[setting];
  f32 yNormal = 1.0f - 0.1f - (-0.1f * (68.0f / camera->playerHeight));
  f32 yOffset =
      CAM_DATA_SCALED(settings->yOffset) * camera->playerHeight * yNormal;
  f32 distTarget =
      CAM_DATA_SCALED(settings->dist) * camera->playerHeight * yNormal;
  f32 yawUpdateRateTarget = settings->yawUpdateRateTarget;
  f32 atLerpStepScale = CAM_DATA_SCALED(settings->atLerpStepScale);

  if (camera->mode != 1 || setting != camera->setting) {
    camera->setting = setting;
    camera->mode = 1;

    camera->parallelAnimTimer = 4;
  }

  if (camera->parallelAnimTimer != 0) {
    camera->parallelYawTarget = angle - 0x7FFF;
  }

  VecGeo atEyeGeo;
  OLib_Vec3fDiffToVecGeo(&atEyeGeo, &camera->at, &camera->eye);

  VecGeo atEyeNextGeo;
  OLib_Vec3fDiffToVecGeo(&atEyeNextGeo, &camera->at, &camera->eyeNext);

  camera->rUpdateRateInv = Camera_LERPCeilF(20.0f, camera->rUpdateRateInv,
                                            camera->speedRatio * 0.5f, 0.1f);
  camera->yawUpdateRateInv =
      Camera_LERPCeilF(yawUpdateRateTarget, camera->yawUpdateRateInv,
                       camera->speedRatio * 0.5f, 0.1f);
  camera->pitchUpdateRateInv = Camera_LERPCeilF(
      2.0f, camera->pitchUpdateRateInv, camera->speedRatio * 0.2f, 0.1f);

  Camera_CalcAtForParallel(camera, yOffset);

  VecGeo eyeAdjustment;
  if (camera->parallelAnimTimer != 0) {
    camera->modeChangeDisallowed = true;
    int animTimer = camera->parallelAnimTimer;
    int tangle = ((animTimer + 1) * animTimer) >> 1;
    eyeAdjustment.yaw =
        atEyeGeo.yaw +
        (angleDiff(camera->parallelYawTarget, atEyeGeo.yaw) / tangle) *
            animTimer;
    eyeAdjustment.pitch = atEyeGeo.pitch;
    eyeAdjustment.r = atEyeGeo.r;
    camera->parallelAnimTimer--;
  } else {
    camera->dist = Camera_LERPCeilF(distTarget, camera->dist,
                                    1.0f / camera->rUpdateRateInv, 2.0f);
    OLib_Vec3fDiffToVecGeo(&eyeAdjustment, &camera->at, &camera->eyeNext);
    eyeAdjustment.r = camera->dist;
    eyeAdjustment.yaw =
        Camera_LERPCeilS(camera->parallelYawTarget, atEyeNextGeo.yaw, 0.8f, 10);
    eyeAdjustment.pitch = Camera_LERPCeilS(
        0.0f, atEyeNextGeo.pitch, 1.0f / camera->pitchUpdateRateInv, 4);

    if (eyeAdjustment.pitch > 14500) {
      eyeAdjustment.pitch = 14500;
    }

    if (eyeAdjustment.pitch < -5460) {
      eyeAdjustment.pitch = -5460;
    }
  }

  Camera_AddVecGeoToVec3f(&camera->eyeNext, &camera->at, &eyeAdjustment);

  // TODO: if skyboxDisabled, call func_80043F94, which also checks if camera
  // view does not go through the floor poly that the player is on?
  Vec3f collisionPoint;
  Vec3f collisionNormal;
  Camera_BGCheckInfo(camera, camera->at, camera->eyeNext, &collisionPoint,
                     &collisionNormal);
  camera->eye = collisionPoint;

  camera->atLERPStepScale = Camera_ClampLERPScale(camera, atLerpStepScale);
}

void Camera::update(Vec3f pos, u16 angle, int setting, int mode) {
  this->frames++;

  Vec3f prevPos = this->playerPos;
  this->playerPos = pos;
  this->xzSpeed = OLib_Vec3fDistXZ(&pos, &prevPos);
  this->speedRatio = OLib_ClampMaxDist(this->xzSpeed / 9.0f, 1.0f);

  if (this->modeChangeDisallowed) {
    mode = this->mode;
  }
  this->modeChangeDisallowed = false;

  switch (mode) {
    case 0:
      Camera_Normal1(this, pos, angle, setting);
      break;
    case 1:
      Camera_Parallel1(this, pos, angle, setting);
      break;
  }
}

void Camera::updateNormal(Vec3f pos, u16 angle, int setting) {
  update(pos, angle, setting, 0);
}

void Camera::updateParallel(Vec3f pos, u16 angle, int setting) {
  update(pos, angle, setting, 1);
}

u16 Camera::yaw() {
  VecGeo result;
  OLib_Vec3fDiffToVecGeo(&result, &this->eye, &this->at);
  return result.yaw;
}
