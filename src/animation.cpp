#include "animation.hpp"

#include "actor.hpp"
#include "sys_math3d.hpp"
#include "sys_matrix.hpp"

Limb childLinkSkeleton[] = {
    {{0, 2376, 0}, 0x01, LIMB_DONE},
    {{-4, -104, 0}, 0x02, 0x09},
    {{607, 0, 0}, 0x03, LIMB_DONE},
    {{-172, 50, -190}, 0x04, 0x06},
    {{697, 0, 0}, 0x05, LIMB_DONE},
    {{825, 5, 11}, LIMB_DONE, LIMB_DONE},
    {{-170, 57, 192}, 0x07, LIMB_DONE},
    {{695, 0, 0}, 0x08, LIMB_DONE},
    {{817, 8, 4}, LIMB_DONE, LIMB_DONE},
    {{0, -103, -7}, 0x0A, LIMB_DONE},
    {{996, -201, -1}, 0x0B, 0x0C},
    {{-365, -670, 0}, LIMB_DONE, LIMB_DONE},
    {{0, 0, 0}, LIMB_DONE, 0x0D},
    {{696, -175, 466}, 0x0E, 0x10},
    {{581, 0, 0}, 0x0F, LIMB_DONE},
    {{514, 0, 0}, LIMB_DONE, LIMB_DONE},
    {{696, -175, -466}, 0x11, 0x13},
    {{577, 0, 0}, 0x12, LIMB_DONE},
    {{525, 0, 0}, LIMB_DONE, LIMB_DONE},
    {{657, -523, 367}, LIMB_DONE, 0x14},
    {{0, 0, 0}, LIMB_DONE, LIMB_DONE},
};

Limb adultLinkSkeleton[] = {
    {{-57, 3377, 0}, 0x01, LIMB_DONE},
    {{0, 0, 0}, 0x02, 0x09},
    {{945, 0, 0}, 0x03, LIMB_DONE},
    {{-399, 69, -249}, 0x04, 0x06},
    {{1306, 0, 0}, 0x05, LIMB_DONE},
    {{1256, 5, 11}, LIMB_DONE, LIMB_DONE},
    {{-396, 76, 264}, 0x07, LIMB_DONE},
    {{1304, 0, 0}, 0x08, LIMB_DONE},
    {{1257, 6, 3}, LIMB_DONE, LIMB_DONE},
    {{0, 21, -7}, 0x0A, LIMB_DONE},
    {{1392, -259, 0}, 0x0B, 0x0C},
    {{-298, -700, 0}, LIMB_DONE, LIMB_DONE},
    {{0, 0, 0}, LIMB_DONE, 0x0D},
    {{1039, -172, 680}, 0x0E, 0x10},
    {{919, 0, 0}, 0x0F, LIMB_DONE},
    {{754, 0, 0}, LIMB_DONE, LIMB_DONE},
    {{1039, -173, -680}, 0x11, 0x13},
    {{919, 0, 0}, 0x12, LIMB_DONE},
    {{754, 0, 0}, LIMB_DONE, LIMB_DONE},
    {{978, -692, 342}, LIMB_DONE, 0x14},
    {{0, 0, 0}, LIMB_DONE, LIMB_DONE},
};

bool nextAnimFrame(f32* curFrame, int endFrame, f32 updateRate) {
  if (*curFrame == endFrame) {
    return false;
  }

  *curFrame += updateRate;
  if (*curFrame >= endFrame) {
    *curFrame = endFrame;
  }

  return true;
}

void loadAnimFrame(AnimationHeader* animHeader, int limbCount, int frame, Vec3f* rootPos, Vec3s* jointTable) {
    JointIndex* jointIndices = &animHeader->jointIndices[0];
    u16* frameData = animHeader->frameData;
    u16* staticData = &frameData[0];
    u16* dynamicData = &frameData[frame];
    u16 staticIndexMax = animHeader->staticIndexMax;

    rootPos->x = (s16)(jointIndices->x >= staticIndexMax ? dynamicData[jointIndices->x] : staticData[jointIndices->x]);
    rootPos->y = (s16)(jointIndices->y >= staticIndexMax ? dynamicData[jointIndices->y] : staticData[jointIndices->y]);
    rootPos->z = (s16)(jointIndices->z >= staticIndexMax ? dynamicData[jointIndices->z] : staticData[jointIndices->z]);
    jointIndices++;

    for (int i = 0; i < limbCount; i++) {
        jointTable->x = (s16)(jointIndices->x >= staticIndexMax ? dynamicData[jointIndices->x] : staticData[jointIndices->x]);
        jointTable->y = (s16)(jointIndices->y >= staticIndexMax ? dynamicData[jointIndices->y] : staticData[jointIndices->y]);
        jointTable->z = (s16)(jointIndices->z >= staticIndexMax ? dynamicData[jointIndices->z] : staticData[jointIndices->z]);
        jointIndices++;
        jointTable++;
    }
}

void loadAnimFrame(u16* animData, int frame, AnimFrame* animFrame) {
  memcpy(animFrame, &animData[frame * sizeof(AnimFrame) / sizeof(u16)],
         sizeof(AnimFrame));
}

void loadUpperBodyAnimFrame(u16* animData, int frame, AnimFrame* animFrame) {
  memcpy(&animFrame->jointTable[9],
         &animData[frame * sizeof(AnimFrame) / sizeof(u16) +
                   9 * sizeof(Vec3s) / sizeof(u16)],
         sizeof(AnimFrame) - 9 * sizeof(Vec3s));
}

void applyLimb(Limb* skeleton, Vec3s* jointTable, u8 limbIndex,
               MtxF* outLimbMatrices) {
  Limb* limb = &skeleton[limbIndex];

  Matrix_Push();

  Vec3f pos = limb->jointPos;
  Vec3s rot = jointTable[limbIndex];
  Matrix_TranslateRotateZYX(&pos, &rot);

  Matrix_Get(&outLimbMatrices[limbIndex]);

  if (limb->child != LIMB_DONE) {
    applyLimb(skeleton, jointTable, limb->child, outLimbMatrices);
  }

  Matrix_Pop();

  if (limb->sibling != LIMB_DONE) {
    applyLimb(skeleton, jointTable, limb->sibling, outLimbMatrices);
  }
}

void applySkeleton(Limb* skeleton, Vec3s* jointTable, Vec3f pos, u16 angle,
                   Vec3f rootPos, MtxF* outLimbMatrices) {
  Matrix_Push();

  Vec3s rot = {0, (s16)angle, 0};
  Matrix_SetTranslateRotateYXZ(pos.x, pos.y, pos.z, &rot);
  Matrix_Scale(0.01f, 0.01f, 0.01f, MTXMODE_APPLY);

  Vec3s rootRot = jointTable[0];
  Matrix_TranslateRotateZYX(&rootPos, &rootRot);

  Limb* rootLimb = &skeleton[0];
  if (rootLimb->child != LIMB_DONE) {
    applyLimb(skeleton, jointTable, rootLimb->child, outLimbMatrices);
  }

  Matrix_Pop();
}

void applyAnimFrame(AnimFrame* animFrame, PlayerAge age, Vec3f pos, u16 angle,
                    MtxF* outLimbMatrices) {
  Limb* skeleton =
      age == PLAYER_AGE_CHILD ? childLinkSkeleton : adultLinkSkeleton;

  Vec3f rootPos = animFrame->rootPos;
  if (age == PLAYER_AGE_CHILD) {
    // 0.64f is from Player_OverrideLimbDrawGameplayCommon
    rootPos = rootPos * 0.64f;
  }

  applySkeleton(skeleton, animFrame->jointTable, pos, angle, rootPos, outLimbMatrices);
}

void applySkinLimb(Limb* skeleton, MtxF* outLimbMatrices, u8 parentIndex, u8 limbIndex) {
    Limb* limb = &skeleton[limbIndex];
    MtxF* mtx;

    if (parentIndex == LIMB_DONE) {
        SkinMatrix_GetClear(&mtx);
    } else {
        mtx = &outLimbMatrices[parentIndex];
    }

    MtxF sp28;
    SkinMatrix_MtxFMtxFMult(mtx, &outLimbMatrices[limbIndex], &sp28);
    SkinMatrix_MtxFCopy(&sp28, &outLimbMatrices[limbIndex]);

    if (limb->child != LIMB_DONE) {
        applySkinLimb(skeleton, outLimbMatrices, limbIndex, limb->child);
    }

    if (limb->sibling != LIMB_DONE) {
        applySkinLimb(skeleton, outLimbMatrices, parentIndex, limb->sibling);
    }
}

// See Skin_ApplyAnimTransformations
void applySkinSkeleton(Limb* skeleton, int limbCount, Vec3s* jointTable, Vec3f pos, u16 angle,
                       Vec3f rootPos, MtxF* outMatrix, MtxF* outLimbMatrices) {

  SkinMatrix_SetTranslateRotateZYX(&outLimbMatrices[0], jointTable[0].x, jointTable[0].y, jointTable[0].z, rootPos.x, rootPos.y, rootPos.z);

  for (int i = 1; i < limbCount; i++) {
      SkinMatrix_SetTranslateRotateZYX(&outLimbMatrices[i],
        jointTable[i].x, jointTable[i].y, jointTable[i].z, skeleton[i].jointPos.x, skeleton[i].jointPos.y, skeleton[i].jointPos.z);
  }

  SkinMatrix_SetTranslateRotateYXZScale(outMatrix, 0.01f, 0.01f, 0.01f, 0, angle, 0, pos.x, pos.y, pos.z);
  applySkinLimb(skeleton, outLimbMatrices, LIMB_DONE, 0);
}

Vec3f baseRootTranslation(PlayerAge age, u16 angle) {
  f32 ageScale = age == PLAYER_AGE_CHILD ? (11.0f / 17.0f) : 1.0f;
  Vec3s baseTranslation = (Vec3f(-57, 3377, 0) * ageScale).toVec3s();
  return rotate(baseTranslation, angle);
}

void updateRootTranslation(AnimFrame* animFrame, Vec3f* pos, u16 angle,
                           Vec3f* prevRootTranslation) {
  Vec3f root = rotate(animFrame->rootPos, angle);
  Vec3f diff = root - *prevRootTranslation;
  diff.y = 0.0f;

  animFrame->rootPos.x = -57;
  animFrame->rootPos.z = 0;
  *pos = *pos + diff * 0.01f;
  *prevRootTranslation = root;
}

Vec3f heldActorPosition(AnimFrame* animFrame, PlayerAge age, Vec3f pos,
                        u16 angle) {
  MtxF limbMatrices[PLAYER_LIMB_MAX];
  applyAnimFrame(animFrame, age, pos, angle, limbMatrices);

  Vec3f leftHandPos, rightHandPos;
  Vec3f sZeroVec = {0.0f, 0.0f, 0.0f};
  SkinMatrix_Vec3fMtxFMultXYZ(&limbMatrices[PLAYER_LIMB_L_HAND], &sZeroVec,
                              &leftHandPos);
  SkinMatrix_Vec3fMtxFMultXYZ(&limbMatrices[PLAYER_LIMB_R_HAND], &sZeroVec,
                              &rightHandPos);

  return (leftHandPos + rightHandPos) * 0.5f;
}

void getWeaponPosition(AnimFrame* animFrame, PlayerAge age, f32 weaponLength, Vec3f pos, u16 angle,
                       Vec3f* outBase, Vec3f* outTip) {
  MtxF limbMatrices[PLAYER_LIMB_MAX];
  applyAnimFrame(animFrame, age, pos, angle, limbMatrices);

  Vec3f baseOffset = {0.0f, 400.0f, 0.0f};
  Matrix_MultVec3fExt(&baseOffset, outBase, &limbMatrices[PLAYER_LIMB_L_HAND]);

  Vec3f tipOffset = {weaponLength, 400.0f, 0.0f};
  Matrix_MultVec3fExt(&tipOffset, outTip, &limbMatrices[PLAYER_LIMB_L_HAND]);
}

bool weaponRecoil(Collision* col, AnimFrame* animFrame, PlayerAge age, f32 weaponLength, Vec3f pos,
                  u16 angle) {
  Vec3f swordBase;
  Vec3f swordTip;
  getWeaponPosition(animFrame, age, weaponLength, pos, angle, &swordBase, &swordTip);

  f32 dist = Math_Vec3f_DistXYZ(&swordTip, &swordBase);
  Vec3f checkBase = swordTip + (swordBase - swordTip) * ((dist + 10.0f) / dist);

  CollisionPoly* outPoly;
  col->entityLineTest(checkBase, swordTip, true, false, false, &outPoly);
  if (outPoly) {
    return true;
  }

  return false;
}

Vec3f sShieldQuadVertices[] = {
    {-4500.0f, -3000.0f, -600.0f},
    {1500.0f, -3000.0f, -600.0f},
    {-4500.0f, 3000.0f, -600.0f},
    {1500.0f, 3000.0f, -600.0f},
};

void getShieldPosition(AnimFrame* animFrame, PlayerAge age, Vec3f pos,
                       u16 angle, Vec3f* outCorners) {
  MtxF limbMatrices[PLAYER_LIMB_MAX];
  applyAnimFrame(animFrame, age, pos, angle, limbMatrices);

  MtxF* mtx = &limbMatrices[PLAYER_LIMB_R_HAND];
  Matrix_MultVec3fExt(&sShieldQuadVertices[0], &outCorners[0], mtx);
  Matrix_MultVec3fExt(&sShieldQuadVertices[1], &outCorners[1], mtx);
  Matrix_MultVec3fExt(&sShieldQuadVertices[2], &outCorners[2], mtx);
  Matrix_MultVec3fExt(&sShieldQuadVertices[3], &outCorners[3], mtx);
}
