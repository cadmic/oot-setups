#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "camera.hpp"
#include "collision.hpp"
#include "collision_data.hpp"
#include "global.hpp"
#include "search.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"
#include "sys_matrix.hpp"

#include <vector>

u16 targetCameraAngles[0x10000];

void initCameraAngles(Collision* col) {
  for (int i = 0; i < 0x10000; i++) {
    Camera camera(col);
    camera.initParallel(Vec3f(0, 0, 0), i, 4);
    targetCameraAngles[i] = camera.yaw();
  }
}

Vec3f move(Collision* col, Vec3f pos, u16 angle, f32 speed) {
  return col->runChecks(pos, translate(pos, angle, speed, -5.0f));
}

bool startWeirdshot(Collision* col, Vec3f pos, u16 angle, Vec3f bombPos, int controlStickDir, int rollBombTimer, int unshieldFrame, int untargetFrame, Vec3f*outPos, int* outWeirdshotFrame, bool debug) {
  int frame = 1;
  Vec3f bombPosTrunc = bombPos.toVec3s();

  u16 camAngle = targetCameraAngles[angle];

  // Dry roll
  f32 speed = 0.0f;
  for (; frame < 11; frame++) {
    int bombTimer = rollBombTimer - frame;
    if (debug) {
      printf("roll: f=%d bombTimer=%d x=%.9g (%08x) z=%.9g (%08x) speed=%.0f\n",
             frame, bombTimer, pos.x, floatToInt(pos.x), pos.z,
             floatToInt(pos.z), speed);
    }

    if (bombTimer < 0) {
      f32 radius = (-bombTimer) * 8.0f;
      Vec3f posTrunc = pos.toVec3s();
      if (Math_Vec3f_DistXZ(&posTrunc, &bombPosTrunc) < radius + 12.0f) {
        break;
      }
    }

    pos = move(col, pos, angle, speed);

    Math_StepToF(&speed, 3.0f, 2.0f);
  }

  if (frame < 6) {
    if (debug) {
      printf("bomb hit too early\n");
    }
    return false;
  }

  int bombTimer = rollBombTimer - frame;
  if (bombTimer <= -7) {
    if (debug) {
      printf("bomb hit too late\n");
    }
    return false;
  }

  pos = move(col, pos, angle, 3.0f);
  u16 damageAngle = Math_Vec3f_Yaw(&pos, &bombPos);
  frame++;

  if (debug) {
    printf(
        "hit: f=%d x=%.9g (%08x) z=%.9g (%08x) damageAngle=%04x\n",
        frame, pos.x, floatToInt(pos.x), pos.z, floatToInt(pos.z), damageAngle);
  }
  pos = move(col, pos, damageAngle, 15.0f);
  frame++;

  if (debug) {
    printf("damage: f=%d x=%.9g (%08x) z=%.9g (%08x)\n", frame, pos.x,
           floatToInt(pos.x), pos.z, floatToInt(pos.z));
  }
  pos = move(col, pos, damageAngle, 7.0f);
  frame++;

  u16 yaw = angle;
  speed = -10.0f;

  bool backwalk = false;

  // Player_Action_808414F8 (backwalk)
  for (int i = 0; i < unshieldFrame + 1; i++, frame++) {
    if (debug) {
      printf("%s: f=%d x=%.9g (%08x) z=%.9g (%08x) speed=%.1f yaw=%04x\n", backwalk ? "backwalk" : "sidewalk", frame, pos.x,
        floatToInt(pos.x), pos.z, floatToInt(pos.z), speed, yaw);
    }

    pos = move(col, pos, yaw, speed);

    int bombTimer = rollBombTimer - frame;
    if (bombTimer > -10) {
      speed = -18.0f;
      yaw = angle;
    }

    if (backwalk) {
      // backwalk
      u16 yawTarget = camAngle + 0x8000;
      speed += 1.5f;
      Math_ScaledStepToS(&yaw, yawTarget, (s16)(yawTarget - yaw) * 0.1f);
    } else if ((controlStickDir == 0x8000 && i == 0) || (controlStickDir != 0x8000 && i == unshieldFrame)) {
      // sidewalk -> backwalk transition
      yaw = camAngle + 0x8000;
      backwalk = true;
    } else {
      // sidewalk
      u16 yawTarget = camAngle + controlStickDir;

      s16 temp2 = yawTarget - yaw;
      s32 temp3 = std::abs(temp2);

      if (temp3 <= 0x4000) {
        speed += 2.0f;
        Math_ScaledStepToS(&yaw, yawTarget, temp3 * 0.1f);
      }
    }
  }

  if (debug) {
    printf("slide: f=%d x=%.9g (%08x) z=%.9g (%08x) speed=%.1f yaw=%04x\n", frame, pos.x,
           floatToInt(pos.x), pos.z, floatToInt(pos.z), speed, yaw);
  }
  pos = move(col, pos, yaw, speed);
  frame++;

  // Player_Action_80840DE4 (slow sidewalk)
  f32 animFrame = 0.0f;
  f32 animSpeed = 1.0f;
  for (int i = 0; i < untargetFrame + 1; i++, frame++) {
    if (debug) {
      printf("untarget: f=%d x=%.9g (%08x) z=%.9g (%08x) speed=%.1f yaw=%04x animFrame=%.2f animSpeed=%.2f\n", frame, pos.x,
             floatToInt(pos.x), pos.z, floatToInt(pos.z), speed, yaw, animFrame, animSpeed);
    }

    int bombTimer = rollBombTimer - frame;
    if (bombTimer >= -10) {
      if (debug) {
        printf("bomb hit while not shielded\n");
      }
      return false;
    }

    pos = move(col, pos, yaw, speed);

    if ((s16)(yaw - angle) >= 0) {
      if (debug) {
        printf("wrong camera angle\n");
      }
      return false;
    }

    animSpeed = speed * (130.0f / 100.0f);
    animFrame += animSpeed * 1.5f;
    animFrame += 24.0f;

    if (animFrame >= 0.0f) {
      if (debug) {
        printf("no weirdshot\n");
      }
      return false;
    }

    speed += 1.5f;

    s16 temp2 = angle - yaw;
    s32 temp3 = std::abs(temp2);

    if (temp3 <= 0x4000) {
       Math_ScaledStepToS(&yaw, angle, temp3 * 0.1f);
    }
  }

  if (debug) {
    printf("final: f=%d x=%.9g (%08x) z=%.9g (%08x) animFrame=%.2f animSpeed=%.2f\n", frame, pos.x,
           floatToInt(pos.x), pos.z, floatToInt(pos.z), animFrame, animSpeed);
  }

  if (animFrame <= -24.0f) {
    if (debug) {
      printf("sideways weirdshot\n");
    }
    return false;
  }

  *outPos = pos;
  *outWeirdshotFrame = animFrame;
  return true;
}

void validateWeirdshotStarts(Collision* col, u16 controlStickDir, bool debug) {
  // sideroll setup
  // Vec3f linkPos = {intToFloat(0xc228373a), 0.0f, intToFloat(0xc285e4a4)};
  // Vec3f bombPos = {intToFloat(0xc10913b4), 0.0f, intToFloat(0x4075a010)};

  // bonk setup
  Vec3f linkPos = {intToFloat(0x3d2354af), 0.0f, intToFloat(0x4333946a)};
  Vec3f bombPos = {intToFloat(0xc10913b4), 0.0f, intToFloat(0x437c5680)};

  u16 angle = 0x0;

  Vec3f pos;
  int weirdshotFrame;

  for (int i = 5; i >= 0; i--) {
    for (int j = 0; j <= 4; j++) {
      for (int k = 0; k <= 7; k++) {
        if (debug) {
          printf("testing: rollBombTimer=%d unshieldFrame=%d untargetFrame=%d\n", i, j, k);
        }
        if (startWeirdshot(col, linkPos, angle, bombPos, controlStickDir, i, j, k, &pos, &weirdshotFrame, debug)) {
          printf("weirdshot start: controlStickDir=%04x rollBombTimer=%d unshieldFrame=%d untargetFrame=%d x=%.9g (%08x) z=%.9g (%08x) weirdshotFrame=%d\n",
            controlStickDir, i, j, k, pos.x, floatToInt(pos.x), pos.z, floatToInt(pos.z), weirdshotFrame);
        }
      }
    }
  }
}

const int ANIM_FRAME_SIZE = 3 * 22 + 1;
const int LOWER_BODY_FRAME_SIZE = 3 * 10;

// Data for animations before gPlayerAnim_link_bow_side_walk_Data
extern u16 weirdshotAnimationData[];
extern size_t weirdshotAnimationDataSize;

// Upper body animation data. Fortunately the right arm position doesn't change
// so we can use the first frame only.
extern u16 upperBodyAnimationData[];

void debugMatrix() {
  MtxF mtx;
  Matrix_Get(&mtx);

  printf(
      "matrix entries:\n"
      "  %08x %08x %08x %08x\n"
      "  %08x %08x %08x %08x\n"
      "  %08x %08x %08x %08x\n"
      "  %08x %08x %08x %08x\n",
      floatToInt(mtx.xx), floatToInt(mtx.yx), floatToInt(mtx.zx),
      floatToInt(mtx.wx), floatToInt(mtx.xy), floatToInt(mtx.yy),
      floatToInt(mtx.zy), floatToInt(mtx.wy), floatToInt(mtx.xz),
      floatToInt(mtx.yz), floatToInt(mtx.zz), floatToInt(mtx.wz),
      floatToInt(mtx.xw), floatToInt(mtx.yw), floatToInt(mtx.zw),
      floatToInt(mtx.ww));
}

void applyLimb(AnimFrame* animFrame, Vec3s jointPos, PlayerLimb limb,
               bool debug) {
  if (debug) {
    printf("applying limb %d\n", limb);
  }

  Vec3f pos = jointPos;
  Vec3s rot = animFrame->jointTable[limb];
  Matrix_TranslateRotateZYX(&pos, &rot);

  if (debug) {
    debugMatrix();
  }
}

bool testWeirdshot(Vec3f pos, u16 angle, u16 xrot, u16 zrot, int weirdshotFrame, bool debug) {
  Vec3s linkRot = {0, (s16)angle, 0};

  u16 animData[ANIM_FRAME_SIZE];

  // lower body
  memcpy(&animData[0],
         &weirdshotAnimationData[weirdshotAnimationDataSize +
                                 weirdshotFrame * ANIM_FRAME_SIZE],
         ANIM_FRAME_SIZE * sizeof(u16));

  // upper body
  memcpy(&animData[LOWER_BODY_FRAME_SIZE],
         &upperBodyAnimationData[LOWER_BODY_FRAME_SIZE],
         (ANIM_FRAME_SIZE - LOWER_BODY_FRAME_SIZE) * sizeof(u16));

  AnimFrame animFrame;
  memcpy(&animFrame, &animData[0], sizeof(animFrame));

  if (debug) {
    printf("joint table:\n");
    printf("  root: x=%04x y=%04x z=%04x\n", (u16)animFrame.rootPos.x, (u16)animFrame.rootPos.y,
           (u16)animFrame.rootPos.z);
    for (int i = 0; i < 21; i++) {
      printf("  limb %d: x=%04x y=%04x z=%04x\n", i,
             (u16)animFrame.jointTable[i].x, (u16)animFrame.jointTable[i].y,
             (u16)animFrame.jointTable[i].z);
    }
  }

  Matrix_SetTranslateRotateYXZ(pos.x, pos.y, pos.z, &linkRot);
  Matrix_Scale(0.01f, 0.01f, 0.01f, MTXMODE_APPLY);

  if (debug) {
    debugMatrix();
  }

  // Root limb
  if (debug) {
    printf("applying root limb\n");
  }

  Vec3f rootPos = animFrame.rootPos;
  Vec3s rootRot = animFrame.jointTable[PLAYER_LIMB_ROOT];
  Matrix_TranslateRotateZYX(&rootPos, &rootRot);

  if (debug) {
    debugMatrix();
  }

  // apply aim rotation
  if (xrot != 0) {
    Matrix_RotateX(BINANG_TO_RAD(xrot), MTXMODE_APPLY);
  }
  if (zrot != 0) {
    Matrix_RotateZ(BINANG_TO_RAD(zrot), MTXMODE_APPLY);
  }
  applyLimb(&animFrame, {0, 21, -7}, PLAYER_LIMB_UPPER, debug);

  applyLimb(&animFrame, {1039, -173, -680}, PLAYER_LIMB_R_SHOULDER, debug);
  applyLimb(&animFrame, {919, 0, 0}, PLAYER_LIMB_R_FOREARM, debug);
  applyLimb(&animFrame, {754, 0, 0}, PLAYER_LIMB_R_HAND, debug);

  Vec3f hookshotOffset = {100.0f, 1640.0f, 0.0f};
  MtxF hookshotMatrix;
  Vec3f hookshotStartPos;
  Vec3s hookshotRot;

  Matrix_MultVec3f(&hookshotOffset, &hookshotStartPos);
  Matrix_RotateZYX(0, -0x4000, -0x4000, MTXMODE_APPLY);
  Matrix_Get(&hookshotMatrix);
  Matrix_MtxFToYXZRotS(&hookshotMatrix, &hookshotRot, 0);

  // shoot hookshot
  f32 speed = 20.0f;
  f32 xzSpeed = Math_CosS(hookshotRot.x) * speed;
  Vec3f velocity = {
      Math_SinS(hookshotRot.y) * xzSpeed,
      -Math_SinS(hookshotRot.x) * speed,
      Math_CosS(hookshotRot.y) * xzSpeed,
  };

  if (debug) {
    printf("hookshotStartPos: x=%.7f (%08x) y=%.7f (%08x) z=%.7f (%08x)\n",
           hookshotStartPos.x, floatToInt(hookshotStartPos.x),
           hookshotStartPos.y, floatToInt(hookshotStartPos.y),
           hookshotStartPos.z, floatToInt(hookshotStartPos.z));
    printf("hookshotRot: x=%04x y=%04x z=%04x\n", (u16)hookshotRot.x,
           (u16)hookshotRot.y, (u16)hookshotRot.z);
    printf("hookshotVel: x=%.7f (%08x) y=%.7f (%08x) z=%.7f (%08x)\n", velocity.x,
           floatToInt(velocity.x), velocity.y, floatToInt(velocity.y),
           velocity.z, floatToInt(velocity.z));
  }

  Vec3f hookshotPos = hookshotStartPos;

  // int hookshotLength = 13; // hookshot
  int hookshotLength = 26; // longshot

  for (int i = 0; i < hookshotLength - 1; i++) {
    Vec3f tipOffset = {0.0f, 0.0f, 900.0f};
    Vec3f tipPos;
    Matrix_SetTranslateRotateYXZ(hookshotPos.x, hookshotPos.y, hookshotPos.z,
                                &hookshotRot);
    Matrix_Scale(0.01f, 0.01f, 0.01f, MTXMODE_APPLY);
    Matrix_MultVec3f(&tipOffset, &tipPos);

    if (debug) {
        printf("tip pos (visual): x=%.7f (%08x) y=%.7f (%08x) z=%.7f (%08x)\n",
            tipPos.x, floatToInt(tipPos.x), tipPos.y,
            floatToInt(tipPos.y), tipPos.z, floatToInt(tipPos.z));
    }

    Vec3f hookshotPrevPos = hookshotPos;
    hookshotPos = hookshotPos + velocity * 1.5f;

    if (debug) {
        printf("hookshot pos: x=%.7f (%08x) y=%.7f (%08x) z=%.7f (%08x)\n",
            hookshotPos.x, floatToInt(hookshotPos.x), hookshotPos.y,
            floatToInt(hookshotPos.y), hookshotPos.z, floatToInt(hookshotPos.z));
    }

    Vec3f prevFrameDiff = hookshotPos - hookshotPrevPos;
    tipPos = tipPos + prevFrameDiff;
    hookshotRot.x = Math_Atan2S(xzSpeed, -velocity.y);

    if (debug) {
        printf("tip pos (collision): x=%.7f (%08x) y=%.7f (%08x) z=%.7f (%08x)\n",
            tipPos.x, floatToInt(tipPos.x), tipPos.y,
            floatToInt(tipPos.y), tipPos.z, floatToInt(tipPos.z));
    }

    if (tipPos.z == 1016.0f) {
        if (tipPos.x < -11.0f || tipPos.x > 11.0f) {
          if (debug) {
            printf("too wide\n");
          }
          return false;
        } else {
          if (debug) {
              printf("hit!\n");
          }
          return true;
        }
    } else if (tipPos.z < 1016.0f) {
        if (debug) {
            printf("overshot\n");
        }
        return false;
    }
  }

  if (debug) {
    printf("undershot\n");
  }
  return false;
}

void searchWeirdshots() {
  std::vector<int> weirdshotFrames = {-1, -2, -3, -5, -6, -7, -8, -11, -13, -15, -16, -19};
  int found = 0;
  u16 amin = 0x7300;
  u16 amax = 0x8d00;

  for (u16 angle = amin; angle <= amax; angle += 0x10) {
    fprintf(stderr, "testing angle=%04x found=%i ...\r", angle, found);
    for (int zi = floatToInt(1318.0f); zi <= floatToInt(1830.0f); zi++) {
      for (int frame : weirdshotFrames) {
        if (testWeirdshot({0.0f, 240.0f, intToFloat(zi)}, angle, 0x0, 0x4b0, frame, false)) {
          printf("angle=%04x z=%.9f zi=%08x frame=%d\n", angle, intToFloat(zi), zi, frame);
          found++;
        }
      }
    }
    fflush(stdout);
  }
}

Vec3f dropBomb(Vec3f pos, u16 angle, bool instant) {
  AnimFrame animFrame;
  loadAnimFrame(gPlayerAnim_link_normal_free2bom_Data, instant ? 7 : 19, &animFrame);
  Vec3f pullPos = heldActorPosition(&animFrame, PLAYER_AGE_ADULT, pos, angle);
  pullPos.y = pos.y;
  return pullPos;
}

std::vector<std::vector<Action>> weirdshotSetups = {
  {BACKFLIP, ROLL},
  {ROLL, BACKFLIP},
  {BACKFLIP_SIDEROLL, ROLL},
  {ROLL, BACKFLIP_SIDEROLL},
  // {ROLL, ROLL, BACKFLIP},
  // {ROLL, BACKFLIP, ROLL},
};

// yrot diff, xrot, zrot
std::vector<std::tuple<u16, u16, u16>> weirdshotAims = {
  {0, 0, 0x4b0},
  {0, 0, 0x258},
  {0, 0, 0},
  // {0, 0x379, 0},
  // {0, 0xfc89, 0},
  // {0x377, 0, 0},
  // {0xfc87, 0, 0},
};

void findSetups(Collision* col, int argc, char* argv[]) {
  SearchParams params = {
      .col = col,
      .minBounds = {-82, 240, 1318},
      .maxBounds = {82, 344, 1830},
      .starts =
          {
              // right corner
              {{82, 240, 1318}, 0x8000},
              {{82, 240, 1318}, 0x4000},
              // left corner
              {{-82, 240, 1318}, 0x8000},
              {{-82, 240, 1318}, 0xc000},
              // yolo along wall
              // {{0, 240, 1318}, 0x8000},
              // {{21, 240, 1318}, 0x8000},
              // {{-21, 240, 1318}, 0x8000},
              // {{42, 240, 1318}, 0x8000},
              // {{-42, 240, 1318}, 0x8000},
              // {{63, 240, 1318}, 0x8000},
              // {{-63, 240, 1318}, 0x8000},
          },
      .maxCost = 85,
      .angleMin = 0x7700,
      .angleMax = 0x8900,
      .actions =
          {
              TARGET_WALL,
              ROLL,
              LONG_ROLL,
              SIDEHOP_LEFT,
              SIDEHOP_LEFT_SIDEROLL,
              SIDEHOP_LEFT_SIDEROLL_UNTARGET,
              SIDEHOP_RIGHT,
              SIDEHOP_RIGHT_SIDEROLL,
              SIDEHOP_RIGHT_SIDEROLL_UNTARGET,
              BACKFLIP,
              BACKFLIP_SIDEROLL,
              BACKFLIP_SIDEROLL_UNTARGET,
              ROTATE_ESS_LEFT,
              ROTATE_ESS_RIGHT,
              ESS_TURN_UP,
              // SHIELD_TURN_LEFT,
              // SHIELD_TURN_RIGHT,
              // SHIELD_TURN_DOWN,
              ESS_TURN_LEFT,
              ESS_TURN_RIGHT,
              ESS_TURN_DOWN,
          },
  };

  auto output = [&](Vec3f initialPos, u16 initialAngle,
                    const PosAngleSetup& setup, const std::vector<Action>& path,
                    int cost) {
    bool found = false;

    u16 angle = setup.angle;

    for (const auto& spacingSetupActions : weirdshotSetups) {
      PosAngleSetup spacingSetup = setup;
      spacingSetup.performActions(spacingSetupActions);

      for (bool instant : {false, true}) {
        Vec3f bombPos = dropBomb(setup.pos, angle, instant);

        for (u16 controlStickDir : {0x4000, 0x8000, 0xc000}) {
          for (int rollBombTimer = 2; rollBombTimer >= 0; rollBombTimer--) {
            for (int unshieldFrame = 0; unshieldFrame <= 2; unshieldFrame++) {
              for (int untargetFrame = 0; untargetFrame <= 2; untargetFrame++) {
                Vec3f weirdshotPos;
                int weirdshotFrame;
                if (startWeirdshot(col, spacingSetup.pos, angle, bombPos, controlStickDir, rollBombTimer, unshieldFrame, untargetFrame, &weirdshotPos, &weirdshotFrame, false)) {
                  if (weirdshotPos.z == 1318.0f) {
                    continue;
                  }

                  for (const auto& [yrotDiff, xrot, zrot] : weirdshotAims) {
                    if (testWeirdshot(weirdshotPos, angle + yrotDiff, xrot, zrot, weirdshotFrame, false)) {
                      printf(
                        "cost=%d startAngle=%04x startx=%.9g startz=%.9g angle=%04x "
                        "startpos x=%.9g (%08x) z=%.9g (%08x) "
                        "bombpos x=%.9g (%08x) z=%.9g (%08x) "
                        "spacing x=%.9g (%08x) z=%.9g (%08x) actions=%s "
                        "weirdshot x=%.9g (%08x) z=%.9g (%08x) frame=%d "
                        "instant=%d controlStickDir=%04x rollBombTimer=%d unshieldFrame=%d untargetFrame=%d "
                        "yrotdiff=%04x xrot=%04x zrot=%04x "
                        "actions=%s\n",
                        cost, initialAngle, initialPos.x, initialPos.z, angle,
                        setup.pos.x, floatToInt(setup.pos.x), setup.pos.z, floatToInt(setup.pos.z),
                        bombPos.x, floatToInt(bombPos.x), bombPos.z, floatToInt(bombPos.z),
                        spacingSetup.pos.x, floatToInt(spacingSetup.pos.x), spacingSetup.pos.z, floatToInt(spacingSetup.pos.z),
                        actionNames(spacingSetupActions).c_str(),
                        weirdshotPos.x, floatToInt(weirdshotPos.x), weirdshotPos.z, floatToInt(weirdshotPos.z), weirdshotFrame,
                        instant, controlStickDir, rollBombTimer, unshieldFrame, untargetFrame,
                        yrotDiff, xrot, zrot,
                        actionNames(path).c_str());
                      fflush(stdout);
                      found = true;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }

    return found;
  };

  if (argc > 1) {
    int shard = atoi(argv[1]);
    searchSetupsShard(params, 1, shard, output);
  } else {
    searchSetups(params, output);
  }
}

int main(int argc, char* argv[]) {
  Collision col(&HAKAdanCH_sceneCollisionHeader_00A558, PLAYER_AGE_ADULT, {-100, 240, 1300}, {100, 280, 1300});

  initCameraAngles(&col);

  // searchWeirdshots();
  // validateWeirdshotStarts(&col, 0x8000, false);

  // Vec3f pos;
  // int weirdshotFrame;
  // startWeirdshot(&col, {intToFloat(0x41f385ba), 240.0f, intToFloat(0x44ae9d81)}, 0x8051, {intToFloat(0x421a268e), 240.0f, intToFloat(0x44a6d0a5)}, 0x8000, 2, 1, 1, &pos, &weirdshotFrame, true);
  // testWeirdshot(pos, 0x8051, 0x0, 0x4b0, weirdshotFrame, true);

  findSetups(&col, argc, argv);

  return 0;
}

// clang-format off
u16 weirdshotAnimationData[] = {
    // gPlayerAnim_link_bow_side_runR_Data
    0xFD57, 0x0C75, 0xFD93, 0x0000, 0x0000, 0x0000, 0x95A0, 0x02DA, 
    0xC000, 0x0000, 0x0000, 0x0000, 0x0244, 0x0D14, 0xE896, 0x0000, 
    0x0000, 0x2E92, 0x08FA, 0x0919, 0xAA50, 0xFC76, 0xF14F, 0xE452, 
    0x0000, 0x0000, 0x2291, 0xF851, 0xF9F1, 0xB1DE, 0x61C2, 0xFE1D, 
    0x35CD, 0xF95B, 0x1FEF, 0x34D3, 0x0000, 0x0000, 0x4DBE, 0x0000, 
    0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 
    0xF8A1, 0x8BC0, 0xD4DA, 0xD92F, 0x1D54, 0x3848, 0x0000, 0x0000, 
    0x0000, 0xFB23, 0x0547, 0xC9CE, 0xC006, 0x6A68, 0xFD32, 0x0000, 
    0x0000, 0x0000, 0x0000, 0xFD54, 0x0C77, 0xFD9E, 0x0000, 0x0000, 
    0x0000, 0x94AD, 0x0356, 0xBF9C, 0x0000, 0x0000, 0x0000, 0x0259, 
    0x0CB4, 0xE74A, 0x0000, 0x0000, 0x2EEA, 0x0809, 0x08E4, 0xAAB8, 
    0xFE17, 0xF127, 0xDF5D, 0x0000, 0x0000, 0x2B98, 0xF8F7, 0xF9E2, 
    0xB095, 0x61B8, 0xFE53, 0x357E, 0xF916, 0x212E, 0x34CB, 0x0000, 
    0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 
    0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD95F, 0x1EAA, 
    0x371E, 0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 0xC04E, 
    0x6A68, 0xFDC0, 0x0000, 0x0000, 0x0000, 0x0000, 0xFD53, 0x0C8C, 
    0xFDA3, 0x0000, 0x0000, 0x0000, 0x93F0, 0x03B8, 0xBF4B, 0x0000, 
    0x0000, 0x0000, 0x02C7, 0x0BD5, 0xE5AA, 0x0000, 0x0000, 0x2E30, 
    0x0652, 0x0865, 0xABEB, 0xFFCD, 0xF110, 0xDABC, 0x0000, 0x0000, 
    0x34F5, 0xF9DB, 0xF9BC, 0xAEBB, 0x61AF, 0xFE7D, 0x3540, 0xF8EF, 
    0x21B7, 0x34B9, 0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 
    0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 
    0xD4DA, 0xD984, 0x1FB1, 0x3639, 0x0000, 0x0000, 0x0000, 0xFB23, 
    0x0547, 0xC9CE, 0xC0B7, 0x6A6B, 0xFE8E, 0x0000, 0x0000, 0x0000, 
    0x0000, 0xFD51, 0x0CAA, 0xFDA5, 0x0000, 0x0000, 0x0000, 0x9366, 
    0x0400, 0xBF0D, 0x0000, 0x0000, 0x0000, 0x0364, 0x0A9D, 0xE3F4, 
    0x0000, 0x0000, 0x2C91, 0x047C, 0x07C5, 0xADB1, 0x00D9, 0xF153, 
    0xD839, 0x0000, 0x0000, 0x3C47, 0xFAE5, 0xF986, 0xAC76, 0x61A8, 
    0xFE9C, 0x3513, 0xF8E4, 0x21B6, 0x34A5, 0x0000, 0x0000, 0x4DBE, 
    0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 
    0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD99F, 0x2070, 0x3592, 0x0000, 
    0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 0xC133, 0x6A70, 0xFF83, 
    0x0000, 0x0000, 0x0000, 0x0000, 0xFD50, 0x0CC4, 0xFDA5, 0x0000, 
    0x0000, 0x0000, 0x930A, 0x0432, 0xBEE3, 0x0000, 0x0000, 0x0000, 
    0x040C, 0x0932, 0xE254, 0x0000, 0x0000, 0x2A5B, 0x032F, 0x072C, 
    0xAFD6, 0x007B, 0xF22A, 0xD940, 0x0000, 0x0000, 0x3FC3, 0xFC00, 
    0xF947, 0xA9EC, 0x61A4, 0xFEB0, 0x34F5, 0xF8E4, 0x2157, 0x3493, 
    0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 
    0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD9B1, 
    0x20EF, 0x3524, 0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 
    0xC1B6, 0x6A78, 0x0088, 0x0000, 0x0000, 0x0000, 0x0000, 0xFD50, 
    0x0CCF, 0xFDA6, 0x0000, 0x0000, 0x0000, 0x92D5, 0x044D, 0xBECA, 
    0x0000, 0x0000, 0x0000, 0x049D, 0x07BC, 0xE0C0, 0x0000, 0x0000, 
    0x2834, 0x0288, 0x067C, 0xB323, 0xFE75, 0xF3EA, 0xDE7B, 0x0000, 
    0x0000, 0x3F06, 0xFD16, 0xF905, 0xA741, 0x61A1, 0xFEBC, 0x34E4, 
    0xF8F8, 0x20C9, 0x3489, 0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 
    0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 
    0x8BC0, 0xD4DA, 0xD9BA, 0x2134, 0x34E7, 0x0000, 0x0000, 0x0000, 
    0xFB23, 0x0547, 0xC9CE, 0xC235, 0x6A82, 0x0184, 0x0000, 0x0000, 
    0x0000, 0x0000, 0xFD4F, 0x0CC6, 0xFDA6, 0x0000, 0x0000, 0x0000, 
    0x92C4, 0x0457, 0xBEC2, 0x0000, 0x0000, 0x0000, 0x0547, 0x05B8, 
    0xDF1F, 0x0000, 0x0000, 0x24E0, 0x0233, 0x05AE, 0xB76D, 0xFBC0, 
    0xF679, 0xE64F, 0x0000, 0x0000, 0x3AF5, 0xFE10, 0xF8C7, 0xA49B, 
    0x61A1, 0xFEBF, 0x34DF, 0xF917, 0x2036, 0x348C, 0x0000, 0x0000, 
    0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 
    0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD9BD, 0x2148, 0x34D6, 
    0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 0xC2A3, 0x6A8D, 
    0x0260, 0x0000, 0x0000, 0x0000, 0x0000, 0xFD4F, 0x0CAE, 0xFDA6, 
    0x0000, 0x0000, 0x0000, 0x92D1, 0x0450, 0xBEC8, 0x0000, 0x0000, 
    0x0000, 0x05F3, 0x030D, 0xDE5A, 0x0000, 0x0000, 0x1E9D, 0x022D, 
    0x050A, 0xBB0D, 0xF963, 0xF95F, 0xEEBA, 0x0000, 0x0000, 0x33EA, 
    0xFEDA, 0xF893, 0xA21E, 0x61A1, 0xFEBC, 0x34E4, 0xF93D, 0x1FCD, 
    0x34A3, 0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 
    0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 
    0xD9BA, 0x2130, 0x34EB, 0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 
    0xC9CE, 0xC2F5, 0x6A97, 0x0304, 0x0000, 0x0000, 0x0000, 0x0000, 
    0xFD4E, 0x0C90, 0xFDA6, 0x0000, 0x0000, 0x0000, 0x92F8, 0x043B, 
    0xBEDA, 0x0000, 0x0000, 0x0000, 0x063A, 0x009D, 0xDE80, 0x0000, 
    0x0000, 0x17CA, 0x0272, 0x04DA, 0xBC5D, 0xF7F1, 0xFBE8, 0xF5E9, 
    0x0000, 0x0000, 0x2BD5, 0xFF5C, 0xF86F, 0x9FF1, 0x61A3, 0xFEB3, 
    0x34F1, 0xF965, 0x1FBA, 0x34D3, 0x0000, 0x0000, 0x4DBE, 0x0000, 
    0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 
    0xF8A1, 0x8BC0, 0xD4DA, 0xD9B1, 0x20F4, 0x351F, 0x0000, 0x0000, 
    0x0000, 0xFB23, 0x0547, 0xC9CE, 0xC320, 0x6A9D, 0x0359, 0x0000, 
    0x0000, 0x0000, 0x0000, 0xFD4E, 0x0C75, 0xFDA5, 0x0000, 0x0000, 
    0x0000, 0x9332, 0x041C, 0xBEF5, 0x0000, 0x0000, 0x0000, 0x0643, 
    0xFF9C, 0xDC98, 0x0000, 0x0000, 0x1859, 0x02F7, 0x0515, 0xBB8A, 
    0xF76E, 0xFD33, 0xF9B4, 0x0000, 0x0000, 0x26AE, 0xFF80, 0xF863, 
    0x9E38, 0x61A6, 0xFEA6, 0x3504, 0xF9C6, 0x2019, 0x356D, 0x0000, 
    0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 
    0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD9A5, 0x209C, 
    0x356C, 0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 0xC317, 
    0x6A9B, 0x0346, 0x0000, 0x0000, 0x0000, 0x0000, 0xFD4D, 0x0C58, 
    0xFDA3, 0x0000, 0x0000, 0x0000, 0x937C, 0x03F5, 0xBF17, 0x0000, 
    0x0000, 0x0000, 0x0645, 0x0057, 0xD6D7, 0x0000, 0x0000, 0x23BD, 
    0x03A8, 0x0578, 0xB9FB, 0xF78A, 0xFCE7, 0xF92C, 0x0000, 0x0000, 
    0x26FA, 0xFD92, 0xF8C4, 0x9DB0, 0x61AA, 0xFE95, 0x351D, 0xFA67, 
    0x20B7, 0x366D, 0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 
    0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 
    0xD4DA, 0xD996, 0x202F, 0x35CB, 0x0000, 0x0000, 0x0000, 0xFB23, 
    0x0547, 0xC9CE, 0xC249, 0x6A84, 0x01AB, 0x0000, 0x0000, 0x0000, 
    0x0000, 0xFD4D, 0x0C3A, 0xFDA2, 0x0000, 0x0000, 0x0000, 0x93D1, 
    0x03C8, 0xBF3D, 0x0000, 0x0000, 0x0000, 0x05EF, 0x01C8, 0xD0E9, 
    0x0000, 0x0000, 0x3122, 0x0477, 0x05F9, 0xB7E3, 0xF80A, 0xFBA8, 
    0xF635, 0x0000, 0x0000, 0x29F3, 0xF9E9, 0xF981, 0x9EA1, 0x61AE, 
    0xFE82, 0x3539, 0xFAFF, 0x214C, 0x375D, 0x0000, 0x0000, 0x4DBE, 
    0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 
    0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD984, 0x1FB3, 0x3637, 0x0000, 
    0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 0xC0AA, 0x6A6A, 0xFE71, 
    0x0000, 0x0000, 0x0000, 0x0000, 0xFD4D, 0x0C2C, 0xFDA0, 0x0000, 
    0x0000, 0x0000, 0x942B, 0x0399, 0xBF64, 0x0000, 0x0000, 0x0000, 
    0x050D, 0x039A, 0xCCEF, 0x0000, 0x0000, 0x3C88, 0x0557, 0x068B, 
    0xB571, 0xF8DE, 0xF9EF, 0xF23F, 0x0000, 0x0000, 0x2CE0, 0xF780, 
    0xF9FF, 0xA080, 0x61B2, 0xFE6E, 0x3556, 0xFB42, 0x218E, 0x37C8, 
    0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 
    0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD972, 
    0x1F31, 0x36A8, 0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 
    0xBEF9, 0x6A6D, 0xFB18, 0x0000, 0x0000, 0x0000, 0x0000, 0xFD4C, 
    0x0C31, 0xFD9E, 0x0000, 0x0000, 0x0000, 0x9485, 0x036B, 0xBF8B, 
    0x0000, 0x0000, 0x0000, 0x03A2, 0x059C, 0xCBA5, 0x0000, 0x0000, 
    0x4487, 0x063A, 0x0725, 0xB2D8, 0xF9DF, 0xF818, 0xEE47, 0x0000, 
    0x0000, 0x2E5B, 0xF701, 0xFA1D, 0xA340, 0x61B6, 0xFE5A, 0x3574, 
    0xFB1E, 0x216B, 0x3790, 0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 
    0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 
    0x8BC0, 0xD4DA, 0xD960, 0x1EAF, 0x371A, 0x0000, 0x0000, 0x0000, 
    0xFB23, 0x0547, 0xC9CE, 0xBDFA, 0x6A7E, 0xF923, 0x0000, 0x0000, 
    0x0000, 0x0000, 0xFD4D, 0x0C3F, 0xFD9B, 0x0000, 0x0000, 0x0000, 
    0x94DB, 0x033F, 0xBFAF, 0x0000, 0x0000, 0x0000, 0x021E, 0x0783, 
    0xCD13, 0x0000, 0x0000, 0x4842, 0x0713, 0x07BC, 0xB04A, 0xFAD8, 
    0xF65E, 0xEADE, 0x0000, 0x0000, 0x2E4B, 0xF6FD, 0xFA23, 0xA6CE, 
    0x61BA, 0xFE47, 0x358F, 0xFAC6, 0x2115, 0x3704, 0x0000, 0x0000, 
    0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 
    0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD94E, 0x1E34, 0x3784, 
    0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 0xBE13, 0x6A7C, 
    0xF955, 0x0000, 0x0000, 0x0000, 0x0000, 0xFD4D, 0x0C50, 0xFD98, 
    0x0000, 0x0000, 0x0000, 0x9527, 0x0318, 0xBFCF, 0x0000, 0x0000, 
    0x0000, 0x00F9, 0x098E, 0xD2CD, 0x0000, 0x0000, 0x45FA, 0x07D4, 
    0x0844, 0xADF7, 0xFB87, 0xF4FB, 0xE893, 0x0000, 0x0000, 0x2CAA, 
    0xF748, 0xFA1B, 0xAA9C, 0x61BD, 0xFE37, 0x35A8, 0xFA53, 0x20A4, 
    0x364E, 0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 
    0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 
    0xD93F, 0x1DC8, 0x37E2, 0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 
    0xC9CE, 0xBECB, 0x6A70, 0xFAC2, 0x0000, 0x0000, 0x0000, 0x0000, 
    0xFD4D, 0x0C62, 0xFD95, 0x0000, 0x0000, 0x0000, 0x9565, 0x02F8, 
    0xBFE8, 0x0000, 0x0000, 0x0000, 0x00C4, 0x0B84, 0xDBCB, 0x0000, 
    0x0000, 0x3E09, 0x086E, 0x08B3, 0xAC11, 0xFC24, 0xF370, 0xE65E, 
    0x0000, 0x0000, 0x299C, 0xF7B6, 0xFA0A, 0xAE18, 0x61C0, 0xFE29, 
    0x35BC, 0xF9E1, 0x2033, 0x3597, 0x0000, 0x0000, 0x4DBE, 0x0000, 
    0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 
    0xF8A1, 0x8BC0, 0xD4DA, 0xD933, 0x1D72, 0x382D, 0x0000, 0x0000, 
    0x0000, 0xFB23, 0x0547, 0xC9CE, 0xBF9C, 0x6A69, 0xFC60, 0x0000, 
    0x0000, 0x0000, 0x0000, 0xFD4D, 0x0C70, 0xFD93, 0x0000, 0x0000, 
    0x0000, 0x958F, 0x02E3, 0xBFF9, 0x0000, 0x0000, 0x0000, 0x019D, 
    0x0CAF, 0xE462, 0x0000, 0x0000, 0x3400, 0x08D5, 0x08FE, 0xAAC9, 
    0xFC7A, 0xF1EA, 0xE4AD, 0x0000, 0x0000, 0x251B, 0xF81D, 0xF9FA, 
    0xB0B2, 0x61C1, 0xFE20, 0x35C9, 0xF988, 0x1FDD, 0x350B, 0x0000, 
    0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 0x453C, 0xF957, 0x2A97, 
    0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 0xD4DA, 0xD92B, 0x1D3A, 
    0x385F, 0x0000, 0x0000, 0x0000, 0xFB23, 0x0547, 0xC9CE, 0xC000, 
    0x6A68, 0xFD26, 0x0000, 0x0000, 0x0000, 0x0000, 0xFD4D, 0x0C75, 
    0xFD93, 0x0000, 0x0000, 0x0000, 0x95A0, 0x02DA, 0xC000, 0x0000, 
    0x0000, 0x0000, 0x024A, 0x0D01, 0xE869, 0x0000, 0x0000, 0x2EA2, 
    0x08FA, 0x0919, 0xAA50, 0xFC74, 0xF144, 0xE44B, 0x0000, 0x0000, 
    0x2254, 0xF850, 0xF9F2, 0xB1DA, 0x61C2, 0xFE1D, 0x35CD, 0xF965, 
    0x1FBA, 0x34D3, 0x0000, 0x0000, 0x4DBE, 0x0000, 0x0000, 0x0000, 
    0x453C, 0xF957, 0x2A97, 0x0000, 0x0000, 0xAAD8, 0xF8A1, 0x8BC0, 
    0xD4DA, 0xD928, 0x1D25, 0x3871, 0x0000, 0x0000, 0x0000, 0xFB23, 
    0x0547, 0xC9CE, 0xC000, 0x6A68, 0xFD26, 0x0000, 0x0000, 0x0000, 
    0x0000,
    // link_animetion_possiblePadding_0989B4 (+ 1 more for alignment)
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
};
// clang-format on

size_t weirdshotAnimationDataSize = ARRAY_COUNT(weirdshotAnimationData);

// clang-format off
u16 upperBodyAnimationData[] = {
    0xFFC7, 0x0DC3, 0xFD90, 0x0000, 0x0000, 0x0000, 0x95A0, 0x02DA, 
    0xC000, 0x0000, 0x0000, 0x0000, 0x0988, 0x0CA7, 0xFC20, 0x0000, 
    0x0000, 0x0EB4, 0x0BCF, 0x05BE, 0xB78E, 0xF7AE, 0xF903, 0xF8D8, 
    0x0000, 0x0000, 0x0E2C, 0xF913, 0xFA86, 0xBAE7, 0x61C2, 0xFE1D, 
    0x3AB1, 0x03B0, 0x21BD, 0x3D44, 0x0000, 0x0000, 0x4DBE, 0x0000, 
    0x0000, 0x0000, 0xFF64, 0xEB9A, 0x8D97, 0x0000, 0x0000, 0xEC09, 
    0xFFDC, 0xFE09, 0xC0D5, 0xDC8E, 0x21A9, 0x3D4F, 0x0000, 0x0000, 
    0x0000, 0x00C0, 0x26BD, 0xC092, 0xC000, 0x6A68, 0xFD26, 0x0000, 
    0x0000, 0x0000, 0x0000,
};
// clang-format on
