#include "animation.hpp"

#include "actor.hpp"
#include "sys_math3d.hpp"
#include "sys_matrix.hpp"

struct Limb {
  Vec3s jointPos;
  u8 child;
  u8 sibling;
};

#define LIMB_DONE 0xFF

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

struct AnimFrame {
  // Limb 0 is actually root limb position
  Vec3s jointTable[22];
  s16 face;
};

void applyLimb(Limb* skeleton, AnimFrame* animFrame, u8 limbIndex,
               MtxF* outLimbMatrices) {
  Limb* limb = &skeleton[limbIndex];

  Matrix_Push();

  Vec3f pos = limb->jointPos;
  Vec3s rot = animFrame->jointTable[limbIndex + 1];
  Matrix_TranslateRotateZYX(&pos, &rot);

  Matrix_Get(&outLimbMatrices[limbIndex + 1]);

  if (limb->child != LIMB_DONE) {
    applyLimb(skeleton, animFrame, limb->child, outLimbMatrices);
  }

  Matrix_Pop();

  if (limb->sibling != LIMB_DONE) {
    applyLimb(skeleton, animFrame, limb->sibling, outLimbMatrices);
  }
}

void extractAnimFrame(u16* animData, int frame, AnimFrame* animFrame) {
  memcpy(animFrame, &animData[frame * sizeof(AnimFrame) / sizeof(u16)],
         sizeof(AnimFrame));
}

Vec3s rootTranslation(u16* animData, int frame) {
  AnimFrame animFrame;
  extractAnimFrame(animData, frame, &animFrame);
  return animFrame.jointTable[0];
}

void applyAnimation(u16* animData, int frame, PlayerAge age, Vec3f pos,
                    u16 angle, Vec3f rootTranslation, MtxF* outLimbMatrices) {
  Limb* skeleton =
      age == PLAYER_AGE_CHILD ? childLinkSkeleton : adultLinkSkeleton;

  AnimFrame animFrame;
  extractAnimFrame(animData, frame, &animFrame);

  Matrix_Push();

  Vec3s linkRot = {0, (s16)angle, 0};
  Matrix_SetTranslateRotateYXZ(pos.x, pos.y, pos.z, &linkRot);
  Matrix_Scale(0.01f, 0.01f, 0.01f, MTXMODE_APPLY);

  Vec3f rootPos = rootTranslation;
  if (age == PLAYER_AGE_CHILD) {
    // 0.64f is from Player_OverrideLimbDrawGameplayCommon
    rootPos = rootPos * 0.64f;
  }
  Vec3s rootRot = animFrame.jointTable[1];
  Matrix_TranslateRotateZYX(&rootPos, &rootRot);

  Matrix_Get(&outLimbMatrices[PLAYER_LIMB_ROOT]);

  Limb* rootLimb = &skeleton[0];
  if (rootLimb->child != LIMB_DONE) {
    applyLimb(skeleton, &animFrame, rootLimb->child, outLimbMatrices);
  }

  Matrix_Pop();
}

void applyAnimation(u16* animData, int frame, PlayerAge age, Vec3f pos,
                    u16 angle, MtxF* outLimbMatrices) {
  applyAnimation(animData, frame, age, pos, angle,
                 rootTranslation(animData, frame), outLimbMatrices);
}

bool nextAnimationFrame(f32* curFrame, int endFrame, f32 updateRate) {
  if (*curFrame == endFrame) {
    return false;
  }

  *curFrame += updateRate;
  if (*curFrame >= endFrame) {
    *curFrame = endFrame;
  }

  return true;
}

Vec3f baseRootTranslation(u16 angle) { return rotate({-57, 3377, 0}, angle); }

void updateRootTranslation(u16* animData, int frame, PlayerAge age, Vec3f* pos,
                           u16 angle, Vec3f* prevRootTranslation) {
  f32 ageScale = age == PLAYER_AGE_CHILD ? (11.0f / 17.0f) : 1.0f;
  Vec3f root = rotate(rootTranslation(animData, frame), angle);
  Vec3f diff = root - *prevRootTranslation;
  diff.y = 0.0f;
  *pos = *pos + diff * ageScale * 0.01f;
  *prevRootTranslation = root;
}

Vec3f heldActorPosition(u16* animData, int frame, PlayerAge age, Vec3f pos,
                        u16 angle) {
  MtxF limbMatrices[PLAYER_LIMB_MAX];
  applyAnimation(animData, frame, age, pos, angle, limbMatrices);

  Vec3f leftHandPos, rightHandPos;
  Vec3f sZeroVec = {0.0f, 0.0f, 0.0f};
  SkinMatrix_Vec3fMtxFMultXYZ(&limbMatrices[PLAYER_LIMB_L_HAND], &sZeroVec,
                              &leftHandPos);
  SkinMatrix_Vec3fMtxFMultXYZ(&limbMatrices[PLAYER_LIMB_R_HAND], &sZeroVec,
                              &rightHandPos);

  return (leftHandPos + rightHandPos) * 0.5f;
}

void getSwordPosition(u16* animData, int frame, PlayerAge age, Vec3f pos,
                      u16 angle, Vec3f* outBase, Vec3f* outTip) {
  f32 swordLength = age == PLAYER_AGE_CHILD ? 3000.0f : 4000.0f;

  Vec3f root = rootTranslation(animData, frame);
  Vec3f swordRoot = Vec3f(-57, root.y, 0);

  MtxF limbMatrices[PLAYER_LIMB_MAX];
  applyAnimation(animData, frame, age, pos, angle, swordRoot, limbMatrices);

  Vec3f baseOffset = {0.0f, 400.0f, 0.0f};
  Matrix_MultVec3fExt(&baseOffset, outBase, &limbMatrices[PLAYER_LIMB_L_HAND]);

  Vec3f tipOffset = {swordLength, 400.0f, 0.0f};
  Matrix_MultVec3fExt(&tipOffset, outTip, &limbMatrices[PLAYER_LIMB_L_HAND]);
}

bool swordRecoil(Collision* col, u16* animData, int frame, PlayerAge age,
                 Vec3f pos, u16 angle) {
  Vec3f swordBase;
  Vec3f swordTip;

  getSwordPosition(animData, frame, age, pos, angle, &swordBase, &swordTip);

  f32 dist = Math_Vec3f_DistXYZ(&swordTip, &swordBase);
  Vec3f checkBase = swordTip + (swordBase - swordTip) * ((dist + 10.0f) / dist);

  CollisionPoly* outPoly;
  col->entityLineTest(checkBase, swordTip, true, false, &outPoly);
  if (outPoly) {
    return true;
  }

  return false;
}
