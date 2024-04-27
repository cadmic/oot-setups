#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "collider.hpp"
#include "collision_data.hpp"
#include "search.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"

#include <vector>

Vec3f dropBomb(Vec3f pos, u16 angle, bool instant, bool swordInHand) {
  AnimFrame animFrame;
  loadAnimFrame(swordInHand ? gPlayerAnim_link_normal_normal2bom_Data
                            : gPlayerAnim_link_normal_free2bom_Data,
                instant ? 7 : 19, &animFrame);
  Vec3f pullPos = heldActorPosition(&animFrame, PLAYER_AGE_CHILD, pos, angle);
  pullPos.y = pos.y;
  return pullPos;
}

void findBomb1Setups(int argc, char* argv[]) {
  Collision col(&spot07_sceneCollisionHeader_003824, PLAYER_AGE_CHILD, {580, 350, -130}, {680, 450, -30});

  SearchParams params = {
      .col = &col,
      .minBounds = {580, 50, -130},
      .maxBounds = {680, 450, -30},
      .starts =
          {
              {{intToFloat(0x441d759b), 402, intToFloat(0xc2ad3379)}, 0x05be},
          },
      .maxCost = 65,
      .angleMin = 0x05be,
      .angleMax = 0x05be,
      .xMin = -10000,
      .xMax = 10000,
      .zMin = -10000,
      .zMax = 10000,
      .actions =
          {
              // TARGET_WALL,
              ROLL,
              // LONG_ROLL,
              // SHIELD_SCOOT,
              SIDEHOP_LEFT,
              SIDEHOP_LEFT_SIDEROLL,
              // SIDEHOP_LEFT_SIDEROLL_UNTARGET,
              SIDEHOP_RIGHT,
              SIDEHOP_RIGHT_SIDEROLL,
              // SIDEHOP_RIGHT_SIDEROLL_UNTARGET,
              BACKFLIP,
              BACKFLIP_SIDEROLL,
              // BACKFLIP_SIDEROLL_UNTARGET,
              HORIZONTAL_SLASH,
              HORIZONTAL_SLASH_SHIELD,
              DIAGONAL_SLASH,
              DIAGONAL_SLASH_SHIELD,
              VERTICAL_SLASH,
              VERTICAL_SLASH_SHIELD,
              FORWARD_STAB,
              FORWARD_STAB_SHIELD,
              JUMPSLASH,
              JUMPSLASH_SHIELD,
              LONG_JUMPSLASH_SHIELD,
              CROUCH_STAB,
              ROTATE_ESS_LEFT,
              ROTATE_ESS_RIGHT,
              // ESS_TURN_UP,
              // SHIELD_TURN_LEFT,
              // SHIELD_TURN_RIGHT,
              // SHIELD_TURN_DOWN,
          },
  };

  auto output = [&](Vec3f initialPos, u16 initialAngle,
                    const PosAngleSetup& setup, const std::vector<Action>& path,
                    int cost) {
    bool found = false;
    
    bool instant = true;
    for (bool swordInHand : {false, true}) {
      Vec3f bombPos = dropBomb(setup.pos, setup.angle, instant, swordInHand);
      if ((floatToInt(bombPos.z) & 0xFFFF) == 2) {
        found = true;
        printf(
            "cost=%d startAngle=%04x startx=%.9g startz=%.9g angle=%04x x=%.9g "
            "(%08x) z=%.9g (%08x) instant=%d swordInHand=%d actions=%s\n",
            cost, initialAngle, initialPos.x, initialPos.z, setup.angle,
            setup.pos.x, floatToInt(setup.pos.x), setup.pos.z,
            floatToInt(setup.pos.z), instant, swordInHand, actionNames(path).c_str());
        fflush(stdout);
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

BgCamInfo gDTWebFloorColCamDataList[] = {
    { 0x0000, 0, NULL },
};

SurfaceType gDTWebFloorColSurfaceType[] = {
    {0x0000C000, 0x000007C8},
};

CollisionPoly gDTWebFloorColPolygons[] = {
    {0x0000, 0x0000, 0x0001, 0x0002, 0x08AB, 0x7DF8, 0x14F8, 0x00C5},
    {0x0000, 0x0000, 0x0002, 0x0003, 0x08B7, 0x7DF6, 0x14FE, 0x00C5},
    {0x0000, 0x0004, 0x0005, 0x0006, 0xEB02, 0x7DF7, 0x08AE, 0x00C5},
    {0x0000, 0x0004, 0x0006, 0x0007, 0xEB0E, 0x7DF9, 0x08B2, 0x00C5},
    {0x0000, 0x0008, 0x0009, 0x0005, 0xEB08, 0x7DF8, 0xF755, 0x00C5},
    {0x0000, 0x0008, 0x0005, 0x0004, 0xEB02, 0x7DF6, 0xF749, 0x00C5},
    {0x0000, 0x0001, 0x000A, 0x000B, 0x14FE, 0x7DF7, 0x08AE, 0x00C5},
    {0x0000, 0x0001, 0x000B, 0x0002, 0x14F2, 0x7DF9, 0x08B2, 0x00C5},
    {0x0000, 0x000A, 0x000C, 0x000D, 0x14F8, 0x7DF8, 0xF755, 0x00C5},
    {0x0000, 0x000A, 0x000D, 0x000B, 0x14FE, 0x7DF6, 0xF749, 0x00C5},
    {0x0000, 0x000C, 0x000E, 0x000F, 0x08AE, 0x7DF7, 0xEB02, 0x00C5},
    {0x0000, 0x000C, 0x000F, 0x000D, 0x08B2, 0x7DF9, 0xEB0E, 0x00C5},
    {0x0000, 0x000E, 0x0009, 0x0008, 0xF755, 0x7DF8, 0xEB08, 0x00C5},
    {0x0000, 0x000E, 0x0008, 0x000F, 0xF749, 0x7DF6, 0xEB02, 0x00C5},
    {0x0000, 0x0003, 0x0002, 0x0007, 0x0000, 0x7FFF, 0x0000, 0x0064},
    {0x0000, 0x0002, 0x000B, 0x000D, 0x0000, 0x7FFF, 0x0000, 0x0064},
    {0x0000, 0x000D, 0x000F, 0x0008, 0x0000, 0x7FFF, 0x0000, 0x0064},
    {0x0000, 0x0008, 0x0004, 0x0007, 0x0000, 0x7FFF, 0x0000, 0x0064},
    {0x0000, 0x0002, 0x000D, 0x0008, 0x0000, 0x7FFF, 0x0000, 0x0064},
    {0x0000, 0x0002, 0x0008, 0x0007, 0x0000, 0x7FFF, 0x0000, 0x0064},
    {0x0000, 0x0000, 0x0003, 0x0007, 0xF749, 0x7DF6, 0x14FE, 0x00C5},
    {0x0000, 0x0000, 0x0007, 0x0006, 0xF755, 0x7DF8, 0x14F8, 0x00C5},
};

Vec3s gDTWebFloorColVertices[] = {
    {      0,      0,  -1200 },
    {   -849,      0,   -849 },
    {   -424,   -100,   -424 },
    {      0,   -100,   -600 },
    {    600,   -100,      0 },
    {   1200,      0,      0 },
    {    849,      0,   -849 },
    {    424,   -100,   -424 },
    {    424,   -100,    424 },
    {    849,      0,    849 },
    {  -1200,      0,      0 },
    {   -600,   -100,      0 },
    {   -849,      0,    849 },
    {   -424,   -100,    424 },
    {      0,      0,   1200 },
    {      0,   -100,    600 },
};

CollisionHeader gDTWebFloorCol = {
    { -1200, -100, -1200 },
    { 1200, 0, 1200 },
    ARRAY_COUNT(gDTWebFloorColVertices), gDTWebFloorColVertices,
    ARRAY_COUNT(gDTWebFloorColPolygons), gDTWebFloorColPolygons,
    gDTWebFloorColSurfaceType,
    gDTWebFloorColCamDataList,
    0, NULL
};

void updateFloorWebCollision(s16 newY) {
    CollisionHeader* colHeader = &gDTWebFloorCol;
    colHeader->vertices[14].y = newY;
    colHeader->vertices[12].y = newY;
    colHeader->vertices[10].y = newY;
    colHeader->vertices[9].y = newY;
    colHeader->vertices[6].y = newY;
    colHeader->vertices[5].y = newY;
    colHeader->vertices[1].y = newY;
    colHeader->vertices[0].y = newY;
}

Vec3f simulateBombDrop(Collision* col, f32 x, f32 z, u16 angle, int bombTimer, bool sidewalk, bool debug) {
  updateFloorWebCollision(0);
  col->updateDynapoly(0, &gDTWebFloorCol, {0.1f, 0.1f, 0.1f}, {0, 0x4000, 0}, {0, 0, 0});

  // col.printPolys();

  u16 movementAngle = angle + 0x8000;
  Vec3f pos = col->findFloor({x, 10, z});
  Vec3f bombPos;
  f32 prevXZSpeed = 0.0f;
  f32 xzSpeed = 0.0f;
  s16 fallDistance = 0;
  bool onWeb = false;
  s16 webTimer = 0;
  f32 webAmplitude = 0.0f;
  f32 webY = 0.0f;
  f32 legFrame = 0.0f;
  f32 upperBodyFrame = (65 - bombTimer) * 1.5;
  while (upperBodyFrame >= 55.0f) {
    upperBodyFrame -= 55.0f;
  }

  for (; bombTimer > 5; bombTimer--) {
    if (debug) {
      printf(
        "bombTimer=%d x=%.9g y=%.9g z=%.9g movementAngle=%04x "
        "bombx=%.9g bomby=%.9g bombz=%.9g "
        "xzSpeed=%.2f fallDistance=%d legFrame=%.2f upperBodyFrame=%.2f "
        "webTimer=%d webAmplitude=%.9g webY=%.9g\n",
        bombTimer, pos.x, pos.y, pos.z, movementAngle,
        bombPos.x, bombPos.y, bombPos.z,
        xzSpeed, fallDistance, legFrame, upperBodyFrame,
        webTimer, webAmplitude, webY);
    }

    if (onWeb) {
      f32 sqrtFallDistance = sqrtf(std::max((f32)fallDistance, 0.0f));
      f32 unk = sqrtFallDistance + sqrtFallDistance;
      if (webAmplitude < unk) {
        if (unk > 2.0f) {
          webAmplitude = unk;
          webTimer = 14;
        }
      }
      if (prevXZSpeed != 0.0f) {
        if (webAmplitude < 0.1f) {
          webTimer = 14;
        }
        webAmplitude = std::max(webAmplitude, 2.0f);
      }
    }

    if (webTimer != 0) {
        webTimer--;
    }
    if (webTimer == 0) {
        webTimer = 14;
    }
    webY = sinf((f32)webTimer * (M_PI / 7)) * webAmplitude;
    Math_ApproachZeroF(&webAmplitude, 1.0f, 0.8f);
    updateFloorWebCollision(-webY * 10.0f);
    col->updateDynapoly(0, &gDTWebFloorCol, {0.1f, 0.1f, 0.1f}, {0, 0x4000, 0}, {0, webY, 0});

    f32 prevY = pos.y;

    CollisionPoly* wallPoly;
    CollisionPoly* floorPoly;
    int dynaId;
    f32 floorHeight;
    pos = col->runChecks(pos, translate(pos, movementAngle, xzSpeed, -5.0f), &wallPoly, &floorPoly, &dynaId,
                    &floorHeight);
    if (dynaId == 0) {
      onWeb = true;
    }

    // func_80841138
    f32 legIncrement;
    if (xzSpeed == 0.0f) {
      legIncrement = 0.5f;
    } else if (xzSpeed - 3.7f < 0.0f) {
      legIncrement = 0.5f + 0.4f * xzSpeed;
    } else if ((xzSpeed - 3.7f) * 0.8f < 1.0f) {
      legIncrement = 0.5f + 0.4f * xzSpeed;
    } else {
      legIncrement = 1.2f + 0.4f * (xzSpeed - 3.7f);
    }

    legFrame += legIncrement * 1.5f;
    if (legFrame < 0.0f) {
        legFrame += 29.0f;
    } else if (legFrame >= 29.0f) {
        legFrame -= 29.0f;
    }

    upperBodyFrame += 1.5f;
    if (upperBodyFrame >= 55.0f) {
      upperBodyFrame = 0.0f;
    }

    fallDistance = (s16)prevY - (s16)pos.y;
    prevXZSpeed = xzSpeed;
    xzSpeed = std::min(xzSpeed + 1.5f, 8.25f);

    AnimFrame animFrame;
    loadAnimFrame(gPlayerAnim_link_normal_back_run_Data, legFrame * (16.0f / 29.0f), &animFrame);
    loadUpperBodyAnimFrame(gPlayerAnim_link_normal_carryB_wait_Data, upperBodyFrame, &animFrame);
    bombPos = heldActorPosition(&animFrame, PLAYER_AGE_CHILD, pos, angle);
  }

  f32 ySpeed = 0.0f;
  for (int i = 0; i < 4; i++) {
    if (debug) {
      printf(
        "i=%d bombx=%.9g bomby=%.9g bombz=%.9g\n",
        i, bombPos.x, bombPos.y, bombPos.z);
    }

    ySpeed -= 1.2f;
    bombPos.y += ySpeed * 1.5f;
  }

  // Explosion is off 9.8f units
  bombPos.y += 9.8f;

  if (debug) {
    printf(
        "explosion: x=%.9g y=%.9g z=%.9g\n",
        bombPos.x, bombPos.y, bombPos.z);
  }
  return bombPos;
}

struct BombDrop {
  f32 deltax;
  f32 deltaz;
  int bombTimer;
};

std::vector<BombDrop> findBombDrops(Collision* col, bool debug) {
  std::vector<BombDrop> result;
  for (int bombTimer = 6; bombTimer < 60; bombTimer++) {
    for (int z = 0; z < 640; z++) {
      int x = 0;
      Vec3f explosionPos = simulateBombDrop(col, x, z, 0x0009, bombTimer, false, false);
      Vec3s explosionPosTrunc = explosionPos.toVec3s();
      if (explosionPosTrunc.y == 0x13 && explosionPosTrunc.z == 0) {
        if (debug) {
          printf("bombTimer=%d x=%d z=%d bombx=%.8g bomby=%.9g bombz=%.9g deltax=%.9g deltaz=%.9g\n",
            bombTimer, x, z, explosionPos.x, explosionPos.y, explosionPos.z,
            explosionPos.x - x, explosionPos.z - z);
        }
        result.push_back({explosionPos.x - x, explosionPos.z - z, bombTimer});
        break;
      }
    }
  }
  return result;
}

void findBomb2Setups(int argc, char* argv[]) {
  Collision col(&ydan_sceneCollisionHeader_00B618, PLAYER_AGE_CHILD, {-150, 0, 0}, {150, 0, 500});
  col.addDynapoly(&gDTWebFloorCol, {0.1f, 0.1f, 0.1f}, {0, 0x4000, 0}, {0, 0, 0});

  std::vector<BombDrop> bombDrops = findBombDrops(&col, false);

  SearchParams params = {
      .col = &col,
      .minBounds = {-150, 0, 0},
      .maxBounds = {150, 100, 640},
      .starts =
          {
              // neutral
              {{-4, 0, 603}, 0x8000},
              // up
              {{-4, 0, 594}, 0x8000},
              // down
              {{-4, 0, 610.875f}, 0x8000},
              // left
              {{intToFloat(0xC0D3D32B), 0, intToFloat(0x44149C3E)}, 0x8000},
              // right
              {{intToFloat(0xBFB10A06), 0, intToFloat(0x44149C32)}, 0x8000},
          },
      .maxCost = 80,
      .angleMin = 0x0000,
      .angleMax = 0xffff,
      .xMin = -0.92f,
      .xMax = 1.09f,
      .zMin = 139.84f,
      .zMax = 141.86f,
      .actions =
          {
              // TARGET_WALL,
              ROLL,
              LONG_ROLL,
              // SHIELD_SCOOT,
              SIDEHOP_LEFT,
              SIDEHOP_LEFT_SIDEROLL,
              SIDEHOP_LEFT_SIDEROLL_UNTARGET,
              SIDEHOP_RIGHT,
              SIDEHOP_RIGHT_SIDEROLL,
              SIDEHOP_RIGHT_SIDEROLL_UNTARGET,
              BACKFLIP,
              BACKFLIP_SIDEROLL,
              BACKFLIP_SIDEROLL_UNTARGET,
              // HORIZONTAL_SLASH,
              HORIZONTAL_SLASH_SHIELD,
              // DIAGONAL_SLASH,
              DIAGONAL_SLASH_SHIELD,
              // VERTICAL_SLASH,
              VERTICAL_SLASH_SHIELD,
              // FORWARD_STAB,
              FORWARD_STAB_SHIELD,
              // JUMPSLASH,
              JUMPSLASH_SHIELD,
              LONG_JUMPSLASH_SHIELD,
              // CROUCH_STAB,
              ROTATE_ESS_LEFT,
              ROTATE_ESS_RIGHT,
              // ESS_TURN_UP,
              SHIELD_TURN_LEFT,
              SHIELD_TURN_RIGHT,
              SHIELD_TURN_DOWN,
          },
  };

  auto output = [&](Vec3f initialPos, u16 initialAngle,
                    const PosAngleSetup& setup, const std::vector<Action>& path,
                    int cost) {
    bool found = false;

    for (const BombDrop& bombDrop : bombDrops) {
      if ((s16)(setup.pos.x + bombDrop.deltax) == 0 && (s16)(setup.pos.z + bombDrop.deltaz) == 0) {
        found = true;
        printf(
            "cost=%d startAngle=%04x startx=%.9g startz=%.9g angle=%04x x=%.9g "
            "(%08x) z=%.9g (%08x) bombTimer=%d actions=%s\n",
            cost, initialAngle, initialPos.x, initialPos.z, setup.angle,
            setup.pos.x, floatToInt(setup.pos.x), setup.pos.z,
            floatToInt(setup.pos.z), bombDrop.bombTimer, actionNames(path).c_str());
        fflush(stdout);
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
    findBomb1Setups(argc, argv);

    // Collision col(&ydan_sceneCollisionHeader_00B618, PLAYER_AGE_CHILD, {-150, 0, 0}, {150, 0, 500});
    // col.addDynapoly(&gDTWebFloorCol, {0.1f, 0.1f, 0.1f}, {0, 0x4000, 0}, {0, 0, 0});
    // simulateBombDrop(&col, 0, 215, 0x0009, 26, false, true);
    // simulateBombDrop(&col, 0, 66, 0x0009, 14, false, true);
    // findBombDrops(&col, true);

    // findBomb2Setups(argc, argv);
    return 0;
}
