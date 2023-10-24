#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "camera.hpp"
#include "collision_data.hpp"
#include "global.hpp"
#include "pos_angle_setup.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

// TODO: move somewhere common (what should the y coordinate be though?)
Vec3f dropBomb(Vec3f pos, u16 angle, bool instant, bool swordInHand) {
  MtxF limbMatrices[PLAYER_LIMB_MAX];
  applyAnimation(swordInHand ? gPlayerAnim_link_normal_normal2bom_Data
                             : gPlayerAnim_link_normal_free2bom_Data,
                 instant ? 7 : 19, PLAYER_AGE_ADULT, pos, angle, limbMatrices);

  Vec3f leftHandPos, rightHandPos;
  Vec3f sZeroVec = {0.0f, 0.0f, 0.0f};
  SkinMatrix_Vec3fMtxFMultXYZ(&limbMatrices[PLAYER_LIMB_L_HAND], &sZeroVec,
                              &leftHandPos);
  SkinMatrix_Vec3fMtxFMultXYZ(&limbMatrices[PLAYER_LIMB_R_HAND], &sZeroVec,
                              &rightHandPos);

  Vec3f pullPos = (leftHandPos + rightHandPos) * 0.5f;
  pullPos.y = pos.y;
  return pullPos;
}

// TODO: move somewhere common
Vec3f bombPushDisplacement(Vec3f linkPos, Vec3s bombPosTrunc) {
  Vec3s linkPosTrunc = linkPos.toVec3s();

  f32 xDelta = linkPosTrunc.x - bombPosTrunc.x;
  f32 zDelta = linkPosTrunc.z - bombPosTrunc.z;

  f32 xzDist = sqrtf(SQ(xDelta) + SQ(zDelta));
  f32 overlap = 18 - xzDist;

  if (overlap <= 0) {
    return Vec3f();
  }

  f32 dispRatio = 0.8f;

  if (xzDist != 0.0f) {
    xDelta *= overlap / xzDist;
    zDelta *= overlap / xzDist;
    return Vec3f(xDelta * dispRatio, 0, zDelta * dispRatio);
  } else {
    return Vec3f(-overlap * dispRatio, 0, 0);
  }
}

bool simulateDoorOpen(Vec3f pos, u16 angle, Vec3s bombPos, bool debug) {
  // ensure can open door
  // door pos: (119, -3010)
  // < 20 units in x direction and < 50 units in z direction
  if (!(pos.x > 99.0f && pos.x < 139.0f && pos.z < -2960.0f)) {
    return false;
  }

  // hess speed + bomb push
  Vec3f displacement = bombPushDisplacement(pos, bombPos) * 2;
  pos = translate(pos, angle, -18.0f, 0.0f, displacement);

  if (debug) {
    printf("bomb push: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  // walk through door
  pos = translate(pos, 0x8000, 0.1f, 0.0f);
  pos = translate(pos, 0x8bb8, 2.1f, 0.0f);
  pos = translate(pos, 0x8000, 4.1f, 0.0f);
  for (int i = 0; i < 6; i++) {
    pos = translate(pos, 0x7faf, 5.0f, 0.0f);
  }
  for (int i = 0; i < 6; i++) {
    pos = translate(pos, 0x7fa4, 5.0f, 0.0f);
  }
  pos = translate(pos, 0x7f9a, 5.0f, 0.0f);
  pos = translate(pos, 0x7f9a, 3.5f, 0.0f);
  pos = translate(pos, 0x7f9a, 2.0f, 0.0f);
  pos = translate(pos, 0x7f9a, 0.5f, 0.0f);

  if (debug) {
    printf("final position: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", pos.x,
           floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z,
           floatToInt(pos.z));
  }

  // ensure inside door frame
  return ((pos.x < 85.0f || pos.x > 153.0f) && pos.z > -3026.0f);
}

Vec3s pushBombPos(bool rightSide) {
  return rightSide ? Vec3s(138, 467, -2961) : Vec3s(99, 467, -2961);
}

u16 essUpAngle(Collision* col, Vec3f pos, u16 angle) {
  Camera camera(col);
  int setting = 4;
  camera.initParallel(pos, angle, setting);
  camera.updateNormal(pos, angle, setting);
  return camera.yaw();
}

void findLeftSideDoorOpens() {
  int tested = 0;
  int found = 0;
  Vec3s bombPos = pushBombPos(false);
  for (u16 angle = 0x4000; angle <= 0x8000; angle += 0x10) {
    for (f32 x = 99.0f; x <= 100.0f; x += 0.01f) {
      for (f32 z = -2961.0f; z <= -2960.0f; z += 0.01f) {
        if (tested % 1000000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x ... \r", tested, found,
                  angle);
        }
        tested++;

        Vec3f pos = {x, 467, z};
        if (simulateDoorOpen(pos, angle, bombPos, false)) {
          found++;
          printf("angle=%04x x=%.9g z=%.9g\n", angle, x, z);
        }
      }
    }
  }
}

void findRightSideDoorOpens() {
  int tested = 0;
  int found = 0;
  Vec3s bombPos = pushBombPos(true);
  for (u16 angle = 0x8000; angle <= 0xc000; angle += 0x10) {
    for (f32 x = 138.0f; x <= 139.0f; x += 0.01f) {
      for (f32 z = -2961.0f; z <= -2960.0f; z += 0.01f) {
        if (tested % 1000000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x ... \r", tested, found,
                  angle);
        }
        tested++;

        Vec3f pos = {x, 467, z};
        if (simulateDoorOpen(pos, angle, bombPos, false)) {
          found++;
          printf("angle=%04x x=%.9g z=%.9g\n", angle, x, z);
        }
      }
    }
  }
}

Vec3f move(Vec3f pos, u16 angle, f32 speed) {
  pos = translate(pos, angle, speed, 0.0f);
  if (pos.x < 53.0f) {
    pos.x = 53.0f;
  } else if (pos.x > 185.0f) {
    pos.x = 185.0f;
  }
  return pos;
}

void startHess(Camera* camera, Vec3f pos, u16 angle, Vec3f bombPos,
               int rollBombTimer, int untargetFrame, u16 damageEssAngle,
               Vec3f* outPos, u16* outAngle, bool debug) {
  int setting = 4;
  camera->initParallel(pos, angle, setting);

  int frame = 1;
  int bombTimer = rollBombTimer - 1;

  // Dry roll
  f32 speed = 0.0f;
  for (; frame < 11; frame++) {
    if (debug) {
      printf("roll: f=%d bombTimer=%d x=%.9g (%08x) z=%.9g (%08x) speed=%.0f\n",
             frame, bombTimer, pos.x, floatToInt(pos.x), pos.z,
             floatToInt(pos.z), speed);
    }

    if (bombTimer < 0) {
      f32 radius = (-bombTimer) * 8.0f;
      if (Math_Vec3f_DistXZ(&pos, &bombPos) < radius + 12.0f) {
        break;
      }
    }

    pos = move(pos, angle, speed);
    if (frame < untargetFrame) {
      camera->updateParallel(pos, angle, setting);
    } else {
      camera->updateNormal(pos, angle, setting);
    }

    Math_StepToF(&speed, 3.0f, 2.0f);
    bombTimer--;
  }

  pos = move(pos, angle, 3.0f);
  u16 facingAngle = Math_Vec3f_Yaw(&pos, &bombPos);
  camera->updateNormal(pos, facingAngle, setting);
  u16 damageAngle = camera->yaw();
  frame++;

  if (debug) {
    printf(
        "hit: f=%d x=%.9g (%08x) z=%.9g (%08x) "
        "facingAngle=%04x damageAngle=%04x\n",
        frame, pos.x, floatToInt(pos.x), pos.z, floatToInt(pos.z), facingAngle,
        damageAngle);
  }

  pos = move(pos, facingAngle, 15.0f);
  camera->updateNormal(pos, facingAngle, setting);
  frame++;

  if (debug) {
    printf("damage: f=%d x=%.9g (%08x) z=%.9g (%08x)\n", frame, pos.x,
           floatToInt(pos.x), pos.z, floatToInt(pos.z));
  }

  pos = move(pos, damageAngle + damageEssAngle, 7.0f);

  *outPos = pos;
  *outAngle = facingAngle;
}

struct HessStart {
  Camera camera;

  Vec3f startPos;
  u16 startAngle;
  Vec3f hessPos;
  u16 hessAngle;

  bool rightSide;
  bool swordInHand;
  bool hessInstant;
  bool pushInstant;
  bool essUp;
  int rollFrame;
  int untargetFrame;
  u16 damageEssAngle;

  HessStart(Collision* col) : camera(col) {}
};

struct HessResult {
  Vec3f startPos;
  u16 startAngle;

  bool rightSide;
  bool swordInHand;
  bool hessInstant;
  bool pushInstant;
  bool essUp;
  int rollFrame;
  int rollFrameWindow;
  int untargetFrame;
  u16 damageEssAngle;

  int shieldFrame;
  std::vector<int> targets;
  std::vector<int> essDirs;
  bool goodBuffering;
};

void printHessResult(const HessResult& result) {
  printf(
      "angle=%04x x=%.9g z=%.9g x_raw=%08x z_raw=%08x "
      "rightSide=%d swordInHand=%d hessInstant=%d "
      "pushInstant=%d essUp=%d rollFrame=%d rollFrameWindow=%d "
      "damageEssAngle=%04x shieldFrame=%d "
      "hessFrames=%d goodBuffering=%d targets=",
      result.startAngle, result.startPos.x, result.startPos.z,
      floatToInt(result.startPos.x), floatToInt(result.startPos.z),
      result.rightSide, result.swordInHand, result.hessInstant,
      result.pushInstant, result.essUp, result.rollFrame,
      result.rollFrameWindow, result.damageEssAngle, result.shieldFrame,
      (int)result.targets.size(), result.goodBuffering);
  for (int i = 0; i < result.targets.size(); i++) {
    printf("%d,", result.targets[i]);
  }
  printf(" essDirs=");
  for (int i = 0; i < result.essDirs.size(); i++) {
    printf("%d,", result.essDirs[i]);
  }
  printf(" untargetFrame=%d\n", result.untargetFrame);
  fflush(stdout);
}

bool testHessPath(const HessStart& start, int shieldFrame,
                  const std::vector<int>& targets,
                  const std::vector<int>& essDirs, int cost, bool debug) {
  int k = targets.size();

  // Re-simulate path to determine camera angle for turn

  Vec3f pos = start.hessPos;
  u16 angle = start.hessAngle;

  f32 speed = 7.0f;
  u16 movementAngle = angle + essDirs[0] * ESS;
  u16 facingAngle = targets[0] ? angle : angle + essDirs[0] * ESS;

  Camera camera = start.camera;
  int setting = 4;
  camera.update(pos, facingAngle, setting, targets[0]);

  for (int i = 1; i < k; i++) {
    if (debug) {
      printf(
          "i=%d x=%.9g z=%.9g speed=%.0f facingAngle=%04x movementAngle=%04x "
          "cameraAngle=%04x\n",
          i, pos.x, pos.z, speed, facingAngle, movementAngle, camera.yaw());
    }

    pos = move(pos, movementAngle, speed);

    movementAngle = facingAngle + essDirs[i] * ESS;
    if (!targets[i]) {
      facingAngle = movementAngle;
    }

    if (i == shieldFrame + 2) {
      speed = -18.0f;
    }

    camera.update(pos, facingAngle, setting, targets[i]);
  }

  if (debug) {
    printf(
        "x=%.9g z=%.9g speed=%.0f facingAngle=%04x movementAngle=%04x "
        "cameraAngle=%04x\n",
        pos.x, pos.z, speed, facingAngle, movementAngle, camera.yaw());
  }

  u16 cameraAngle = camera.yaw();
  u16 turnAngle = start.rightSide ? cameraAngle - 0x4000 : cameraAngle + 0x4000;

  pos = move(pos, movementAngle, speed);
  return simulateDoorOpen(pos, turnAngle, pushBombPos(start.rightSide), debug);
}

void searchHessPaths(const HessStart& start, int shieldFrame,
                     std::vector<int>* targets, std::vector<int>* essDirs,
                     int cost, Vec3f pos, u16 angle, f32 speed, int depth,
                     std::vector<HessResult>* results) {
  int k = targets->size();

  Vec3s posTrunc = pos.toVec3s();
  if (posTrunc.z == -2960 && ((!start.rightSide && posTrunc.x == 99) ||
                              (start.rightSide && posTrunc.x == 138))) {
    if (testHessPath(start, shieldFrame, *targets, *essDirs, cost, false)) {
      int rollFrameWindow = 1;
      if (start.hessInstant) {
        if (start.rollFrame == 3 && shieldFrame == 0) {
          rollFrameWindow = 2;
        }
      } else {
        if (start.rollFrame == 4 || start.rollFrame == 2) {
          rollFrameWindow = 2;
        }
      }

      bool buffers[10];
      for (int i = 0; i < k; i++) {
        if (i == 0 || i == shieldFrame) {
          buffers[i] = true;
        } else if ((*targets)[i] != (*targets)[i - 1] ||
                   (*essDirs)[i] != (*essDirs)[i - 1]) {
          buffers[i] = true;
        } else {
          buffers[i] = false;
        }
      }

      int skips = 0;
      for (int i = 1; i < k; i++) {
        if (buffers[i - 1] && !buffers[i]) {
          skips++;
        }
      }

      bool goodBuffering = (skips == 1);

      results->emplace_back();
      HessResult& result = results->back();
      result.startPos = start.startPos;
      result.startAngle = start.startAngle;
      result.rightSide = start.rightSide;
      result.swordInHand = start.swordInHand;
      result.hessInstant = start.hessInstant;
      result.pushInstant = start.pushInstant;
      result.essUp = start.essUp;
      result.rollFrame = start.rollFrame;
      result.rollFrameWindow = rollFrameWindow;
      result.untargetFrame = start.untargetFrame;
      result.damageEssAngle = start.damageEssAngle;
      result.shieldFrame = shieldFrame;
      result.targets = *targets;
      result.essDirs = *essDirs;
      result.goodBuffering = goodBuffering;
    }
    return;
  }

  if (depth == 0) {
    return;
  }

  for (int target = 0; target <= 1; target++) {
    for (int essDir = -1; essDir <= 1; essDir += 2) {
      if (k == shieldFrame && !target) {
        continue;
      }

      f32 newSpeed = (k == shieldFrame + 2) ? -18.0f : speed;

      Vec3f newPos = move(pos, angle + essDir * ESS, newSpeed);
      u16 newAngle = target ? angle : angle + essDir * ESS;

      int newCost = cost;
      if (k == shieldFrame) {
        newCost++;
      } else if (k == 0) {
        if (target || (essDir == 1 && start.damageEssAngle != 0x4000) ||
            (essDir == -1 && start.damageEssAngle != 0xc000)) {
          newCost++;
        }
      } else if (target != targets->back() || essDir != essDirs->back()) {
        newCost++;
      }

      if (newCost > 3) {
        continue;
      }

      targets->push_back(target);
      essDirs->push_back(essDir);
      searchHessPaths(start, shieldFrame, targets, essDirs, newCost, newPos,
                      newAngle, newSpeed, depth - 1, results);
      targets->pop_back();
      essDirs->pop_back();
    }
  }
}

// instant, roll frame, max untarget frame, max shield frame
std::vector<std::tuple<bool, int, int, int>> hessFrameData = {
    // TODO: 3rd frame hess is possible on this frame but has a tendency to
    // target the wall
    {false, 5, 8, 1},
    // {false, 4, 8, 1},  // redundant, previous has 2f window
    {false, 3, 7, 1},
    {false, 2, 6, 0},
    // {false, 1, 6, 0},  // redundant, previous has 2f window
    //
    // {true, 5, 9, 1},  // hard to see bomb flashes
    // {true, 4, 8, 1},  // hard to see bomb flashes
    {true, 3, 7, 1},
    // {true, 2, 7, 0},  // redundant, previous has 2f window
    {true, 1, 6, 0},
};

void findHessPaths(Collision* col, Vec3f startPos, u16 startAngle,
                   bool rightSide, std::vector<int>* targets,
                   std::vector<int>* essDirs,
                   std::vector<HessResult>* results) {
  Vec3s targetBombPos = pushBombPos(rightSide);

  Vec3f bombDropPos = startPos;

  // backflip
  for (int i = 0; i < 11; i++) {
    bombDropPos = translate(bombDropPos, startAngle + 0x8000, 6.0f, 0.0f);
  }
  bombDropPos = translate(bombDropPos, startAngle + 0x8000, 5.0f, 0.0f);

  for (bool pushInstant : {false, true}) {
    Vec3s bombPos =
        dropBomb(bombDropPos, startAngle, pushInstant, false).toVec3s();
    if (!(bombPos.x == targetBombPos.x && bombPos.z == targetBombPos.z)) {
      continue;
    }

    for (bool essUp : {false, true}) {
      Vec3f pos = bombDropPos;
      u16 angle = essUp ? essUpAngle(col, pos, startAngle) : startAngle;

      // roll
      pos = translate(pos, angle, 2.0f, 0.0f);
      for (int i = 0; i < 10; i++) {
        pos = translate(pos, angle, 3.0f, 0.0f);
      }

      for (const auto& frameData : hessFrameData) {
        bool hessInstant = std::get<0>(frameData);
        int rollFrame = std::get<1>(frameData);
        // int maxUntargetFrame = std::get<2>(frameData);
        int maxUntargetFrame = 1;  // we can determine frame windows later
        int maxShieldFrame = std::get<3>(frameData);

        for (bool swordInHand : {false, true}) {
          if (swordInHand && !hessInstant) {
            continue;
          }

          Vec3f hessBombPos =
              dropBomb(startPos, startAngle, hessInstant, swordInHand);

          for (int untargetFrame = 1; untargetFrame <= maxUntargetFrame;
               untargetFrame++) {
            for (int damageEssAngle = 0x0000; damageEssAngle < 0x10000;
                 damageEssAngle += 0x4000) {
              if (!hessInstant && damageEssAngle == 0x0000) {
                continue;
              }

              HessStart start(col);
              start.startPos = startPos;
              start.startAngle = startAngle;
              start.rightSide = rightSide;
              start.swordInHand = swordInHand;
              start.hessInstant = hessInstant;
              start.pushInstant = pushInstant;
              start.essUp = essUp;
              start.rollFrame = rollFrame;
              start.untargetFrame = untargetFrame;
              start.damageEssAngle = damageEssAngle;

              startHess(&start.camera, pos, angle, hessBombPos, rollFrame,
                        untargetFrame, damageEssAngle, &start.hessPos,
                        &start.hessAngle, false);

              for (int shieldFrame = 0; shieldFrame <= maxShieldFrame;
                   shieldFrame++) {
                targets->clear();
                essDirs->clear();
                searchHessPaths(start, shieldFrame, targets, essDirs, 0,
                                start.hessPos, start.hessAngle, 7.0f, 7,
                                results);
              }
            }
          }
        }
      }
    }
  }
}

void printHessPaths(Collision* col) {
  std::vector<int> targets;
  targets.reserve(20);

  std::vector<int> essDirs;
  essDirs.reserve(20);

  std::vector<HessResult> results;
  results.reserve(200);

  int tested = 0;
  int found = 0;
  f32 step = 0.05f;
  for (u16 angle = 0x2300; angle < 0x2500; angle += 0x10) {
    for (f32 x = 185.0f; x >= 180.0f; x -= step) {
      for (f32 z = -2900.0f; z <= -2880.0f; z += step) {
        if (tested % 1000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x x=%.2f z=%.2f ... \r",
                  tested, found, angle, x, z);
        }
        tested++;

        results.clear();
        findHessPaths(col, {x, 467, z}, angle, false, &targets, &essDirs,
                      &results);
        if (!results.empty()) {
          found++;
          for (const auto& result : results) {
            printHessResult(result);
          }
        }
      }
    }
  }

  for (u16 angle = 0xd600; angle < 0xe200; angle += 0x10) {
    for (f32 x = 53.0f; x <= 83.0f; x += step) {
      for (f32 z = -2900.0f; z <= -2880.0f; z += step) {
        if (tested % 1000 == 0) {
          fprintf(stderr, "tested=%d found=%d angle=%04x x=%.2f z=%.2f ... \r",
                  tested, found, angle, x, z);
        }
        tested++;

        results.clear();
        findHessPaths(col, {x, 467, z}, angle, true, &targets, &essDirs,
                      &results);
        if (!results.empty()) {
          found++;
          for (const auto& result : results) {
            printHessResult(result);
          }
        }
      }
    }
  }
}

std::vector<Action> addlActions = {
    // HORIZONTAL_SLASH,
    HORIZONTAL_SLASH_SHIELD,
    // DIAGONAL_SLASH,
    DIAGONAL_SLASH_SHIELD,
    // VERTICAL_SLASH,
    VERTICAL_SLASH_SHIELD,
    // FORWARD_STAB,
    FORWARD_STAB_SHIELD,
    JUMPSLASH_SHIELD,
    CROUCH_STAB,
    // ROLL,
    SIDEHOP_LEFT,
    SIDEHOP_RIGHT,
    BACKFLIP,
    TURN_ESS_UP,
    TURN_ESS_LEFT,
    TURN_ESS_RIGHT,
    TURN_LEFT,
    TURN_DOWN,
    TURN_RIGHT,
};

u64 tested = 0;
int close = 0;
int found = 0;

void searchSetups(Collision* col, Vec3f initialPos, u16 initialAngle,
                  std::vector<int>* targets, std::vector<int>* essDirs,
                  std::vector<HessResult>* results,
                  std::vector<Action>* actions, Vec3f pos, u16 angle, int cost,
                  int depth) {
  int k = actions->size();

  if (depth == 0) {
    if (tested % 10000 == 0) {
      fprintf(stderr,
              "tested=%llu close=%d found=%d k=%d x=%.0f z=%.0f angle=%04x "
              "cost=%d ",
              tested, close, found, k, initialPos.x, initialPos.z, initialAngle,
              cost);
      for (int i = 0; i < 3; i++) {
        fprintf(stderr, "%s ", actionName((*actions)[i]));
      }
      fprintf(stderr, "... \r");
    }
    tested++;

    if (0xd6c0 <= angle && angle <= 0xe100 && pos.x < 75 && pos.z > -2900 &&
        pos.z < -2880) {
      close++;
      results->clear();
      findHessPaths(col, pos, angle, true, targets, essDirs, results);
      if (!results->empty()) {
        found++;

        for (const auto& result : *results) {
          printf(
              "cost=%d initialAngle=%04x initialx=%.0f initialz=%.0f "
              "actions=",
              cost, initialAngle, initialPos.x, initialPos.z);
          for (int i = 0; i < actions->size(); i++) {
            printf("%s,", actionName((*actions)[i]));
          }
          printf(" ");
          printHessResult(result);
        }
      }
    }

    return;
  }

  for (const Action action : addlActions) {
    if (k > 0) {
      if ((action == TURN_ESS_LEFT && actions->back() == TURN_ESS_RIGHT) ||
          (action == TURN_ESS_RIGHT && actions->back() == TURN_ESS_LEFT)) {
        continue;
      }

      // against wall
      if ((pos.x == 185 && 0x2000 <= angle && angle <= 0x6000) ||
          (pos.z <= -2972 && 0x6000 <= angle && angle <= 0xa000) ||
          (pos.x == 53 && 0xa000 <= angle && angle <= 0xe000)) {
        switch (actions->back()) {
          case TURN_ESS_UP:
          case TURN_ESS_LEFT:
          case TURN_ESS_RIGHT:
          case TURN_LEFT:
          case TURN_DOWN:
          case TURN_RIGHT:
          case SIDEHOP_LEFT_SIDEROLL_RETARGET:
          case SIDEHOP_RIGHT_SIDEROLL_RETARGET:
            // previous action was untargeted; only allow actions that don't
            // require targeting
            switch (action) {
              case TURN_ESS_LEFT:
              case TURN_ESS_RIGHT:
              case HORIZONTAL_SLASH:
              case HORIZONTAL_SLASH_SHIELD:
              case CROUCH_STAB:
                break;
              default:
                continue;
            }
            break;
          default:
            break;
        }
      }
    }

    int newCost = cost;
    if (k > 0 &&
        ((action == TURN_ESS_RIGHT && actions->back() == TURN_ESS_RIGHT) ||
         (action == TURN_ESS_LEFT && actions->back() == TURN_ESS_LEFT))) {
      newCost += 1;
    } else {
      newCost += actionCost(action);
    }

    if (newCost > 75) {
      continue;
    }

    PosAngleSetup setup(col, pos, angle);
    switch (action) {
      // TODO: put in PosAngleSetup (need to detect camera setting from floor
      // poly)
      case TURN_ESS_UP:
        setup.angle = essUpAngle(col, pos, angle);
        break;
      case TURN_LEFT:
        setup.angle = essUpAngle(col, pos, angle) + 0x4000;
        break;
      case TURN_DOWN:
        setup.angle = essUpAngle(col, pos, angle) + 0x8000;
        break;
      case TURN_RIGHT:
        setup.angle = essUpAngle(col, pos, angle) + 0xc000;
        break;
      default:
        if (!setup.performAction(action)) {
          continue;
        }
        break;
    }
    if (setup.pos == pos && setup.angle == angle) {
      continue;
    }

    // Check if there's no hope of reaching required angle with ESS turns and
    // cardinal turns (with leeway for angle range and ess up shenanigans)
    int angleDiff = (setup.angle - 0xd900) % 0x4000;
    if (angleDiff < 0) {
      angleDiff += 0x4000;
    }
    int maxTurn = (depth - 1) * ESS + 0x800;
    if (angleDiff > maxTurn && angleDiff < 0x4000 - maxTurn) {
      continue;
    }

    actions->push_back(action);
    searchSetups(col, initialPos, initialAngle, targets, essDirs, results,
                 actions, setup.pos, setup.angle, newCost, depth - 1);
    actions->pop_back();
  }
}

void findPosAngleSetups(Collision* col) {
  std::vector<int> targets;
  targets.reserve(20);

  std::vector<int> essDirs;
  essDirs.reserve(20);

  std::vector<HessResult> results;
  results.reserve(20);

  std::vector<Action> actions;
  actions.reserve(20);

  std::vector<std::pair<Vec3f, u16>> initialPositions = {
      {{53, 467, -2972}, 0xc000},
      {{53, 467, -2972}, 0x8000},
      {{185, 467, -2972}, 0x8000},
      {{185, 467, -2972}, 0x4000},
  };

  for (int depth = 9; depth <= 10; depth++) {
    for (const auto& initialPosition : initialPositions) {
      Vec3f initialPos = initialPosition.first;
      u16 initialAngle = initialPosition.second;
      searchSetups(col, initialPos, initialAngle, &targets, &essDirs, &results,
                   &actions, initialPos, initialAngle, 0, depth);
    }
  }
}

int main(int argc, char* argv[]) {
  Collision col(&Bmori1_sceneCollisionHeader_014054, PLAYER_AGE_ADULT);
  std::vector<u32> polys = {
      // floor
      0x80373820,
      0x80373830,
      // left side wall
      0x80373870,
      // left side of door
      0x80373880,
      0x80373890,
      // door
      0x80373720,
      0x80373810,
      // right side of door
      0x803738b0,
      0x803738c0,
      // right side wall
      0x80373940,
      0x80373950,
  };
  for (u32 address : polys) {
    col.addPoly((address - 0x8036f840) / 0x10);
  }

  // findLeftSideDoorOpens();
  // findRightSideDoorOpens();

  // printHessPaths(&col);

  findPosAngleSetups(&col);

  return 0;
}
