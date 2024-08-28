#include "actor.hpp"
#include "animation.hpp"
#include "animation_data.hpp"
#include "collider.hpp"
#include "collision.hpp"
#include "collision_data.hpp"
#include "pos_angle_setup.hpp"
#include "search.hpp"
#include "sys_math.hpp"

BgCamInfo gDTWebWallColCamDataList[] = {
    { 0x0000, 0, NULL },
};

SurfaceType gDTWebWallColSurfaceType[] = {
    {0x0000C000, 0x000007C0},
};

CollisionPoly gDTWebWallColPolygons[] = {
    {0x0000, 0x8000, 0x0001, 0x0002, 0x0000, 0x0000, 0x7FFF, 0x0000},
    {0x0000, 0x8003, 0x0004, 0x0000, 0x0000, 0x0000, 0x7FFF, 0x0000},
    {0x0000, 0x8004, 0x0001, 0x0000, 0x0000, 0x0000, 0x7FFF, 0x0000},
    {0x0000, 0x8003, 0x0000, 0x0002, 0x0000, 0x0000, 0x7FFF, 0x0000},
};

Vec3s gDTWebWallColVertices[] = {
    {      0,   1444,      0 },
    {   1400,      0,      0 },
    {   1400,   2888,      0 },
    {  -1400,   2888,      0 },
    {  -1400,      0,      0 },
};

CollisionHeader gDTWebWallCol = {
    { -1400, 0, 0 },
    { 1400, 2888, 0 },
    ARRAY_COUNT(gDTWebWallColVertices), gDTWebWallColVertices,
    ARRAY_COUNT(gDTWebWallColPolygons), gDTWebWallColPolygons,
    gDTWebWallColSurfaceType,
    gDTWebWallColCamDataList,
    0, NULL
};

Vec3f instantDropBomb(Vec3f pos, u16 angle) {
  AnimFrame animFrame;
  loadAnimFrame(gPlayerAnim_link_normal_free2bom_Data, 7, &animFrame);
  Vec3f pullPos = heldActorPosition(&animFrame, PLAYER_AGE_ADULT, pos, angle);
  pullPos.y = pos.y;
  return pullPos;
}

bool simulateClip(Collision* col, Vec3f pos, u16 angle, Vec3f hoverBombPos, Vec3f damageBombPos, bool fakeGroundClip, bool debug) {
    f32 xzSpeed = 6.0f;
    f32 ySpeed = 5.8f;
    u16 movementAngle = angle + 0x8000;
    CollisionPoly* wallPoly;
    CollisionPoly* floorPoly;
    int dynaId;
    f32 floorHeight;

    for (int i = 0; i < 5; i++) {
        if (debug) {
            printf("i=%d x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) angle=%04x xzSpeed=%.0f ySpeed=%.1f\n", i,
                pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z, floatToInt(pos.z), movementAngle, xzSpeed, ySpeed);
        }

        ySpeed -= 1.0f;
        pos = col->runChecks(pos, translate(pos, movementAngle, xzSpeed, ySpeed));
    }

    if (debug) {
        printf("hover: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) ySpeed=%.1f\n",
            pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z, floatToInt(pos.z), ySpeed);
    }

    AnimFrame animFrame;
    Vec3f shieldCorners[4];

    loadAnimFrame(gPlayerAnim_link_fighter_backturn_jump_Data, 5, &animFrame);
    loadUpperBodyAnimFrame(gPlayerAnim_link_anchor_waitR2defense_Data, 2, &animFrame); // left stance
    getShieldPosition(&animFrame, PLAYER_AGE_ADULT, pos, angle, shieldCorners);

    if (debug) {
        for (int j = 0; j < 4; j++) {
            Vec3f v = shieldCorners[j];
            printf("shieldCorners[%d]: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n", j,
                v.x, floatToInt(v.x), v.y, floatToInt(v.y), v.z, floatToInt(v.z));
        }
    }

    Vec3s hoverBombPosTrunc = hoverBombPos.toVec3s();
    Vec3s damageBombPosTrunc = damageBombPos.toVec3s();
    Vec3s posTrunc = pos.toVec3s();

    int hoverBombTimer = 6;
    int explosionRadius = (10 - hoverBombTimer) * 8;
    Vec3s center = hoverBombPosTrunc;
    center.y += 10;  // explosion is offset 9.8 units

    Sphere16 explosion = {center, (s16)explosionRadius};
    if (!colliderSphVsQuad(&explosion, shieldCorners)) {
        if (debug) {
            printf("hover bomb did not hit shield\n");
        }
        return false;
    }

    f32 hoverXZ = sqrtf(SQ(hoverBombPosTrunc.x - damageBombPosTrunc.x) + SQ(hoverBombPosTrunc.z - damageBombPosTrunc.z));
    int hoverDelay = ceilf((hoverXZ - 6.0f) / 8.0f);

    f32 damageXZ = sqrtf(SQ(damageBombPosTrunc.x - posTrunc.x) + SQ(damageBombPosTrunc.z - posTrunc.z));
    int damageDelay = ceilf((damageXZ - 12.0f) / 8.0f);

    int delay = hoverDelay + damageDelay;

    if (debug) {
        printf("hoverXZ=%.9g hoverDelay=%d damageXZ=%.9g damageDelay=%d delay=%d\n",
            hoverXZ, hoverDelay, damageXZ, damageDelay, delay);
    }

    if (!((hoverDelay == 8 && damageDelay == 9) || (hoverDelay == 9 && damageDelay == 8))) {
        if (debug) {
            printf("wrong bomb timing\n");
        }
        return false;
    }

    u16 damageAngle = Math_Vec3f_Yaw(&pos, &damageBombPos) + 0x8000;

    Vec3f earlyFramePos = col->runChecks(pos, translate(pos, angle, 0.0f, -13.2f), &wallPoly, &floorPoly, &dynaId, &floorHeight);
    if (debug) {
        printf("early: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
            earlyFramePos.x, floatToInt(earlyFramePos.x), earlyFramePos.y, floatToInt(earlyFramePos.y), earlyFramePos.z, floatToInt(earlyFramePos.z));
    }
    if (earlyFramePos.y <= floorHeight) {
        if (debug) {
            printf("too low\n");
        }
        return false;
    }

    pos = col->runChecks(pos, translate(pos, angle, 0.0f, -14.2f));
    if (debug) {
        printf("landing: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) damageAngle=%04x\n",
            pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z, floatToInt(pos.z), damageAngle);
    }

    if (pos.y == -760) {
        if (debug) {
            printf("wrong landing height\n");
        }
        return false;
    }

    if (fakeGroundClip) {
        pos.y -= 0.0001f;
    }

    pos = col->runChecks(pos, translate(pos, damageAngle, 15, -15.2f));
    if (debug) {
        printf("damage: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
            pos.x, floatToInt(pos.x), pos.y, floatToInt(pos.y), pos.z, floatToInt(pos.z));
    }

    if (pos.x + pos.z > -2475) {
        if (debug) {
            printf("no clip\n");
        }
        return false;
    }

    return true;
}

void searchClipPositions(Collision* col) {
    PosAngleRange range = {
        .angleMin = 0x2000,
        .angleMax = 0x2080,
        .xMin = -2174,
        .xMax = -2170,
        .xStep = 0.01f,
        .zMin = -294,
        .zMax = -290,
        .zStep = 0.01f,
    };

    searchPosAngleRange(range, [&](u16 angle, f32 x, f32 z) {
        Vec3f pos = col->findFloor({x, -700, z});
        if (pos.y == BGCHECK_Y_MIN) {
            return false;
        }
        pos = col->runChecks(pos, translate(pos, angle, 0.0f, -5.0f));
        if (pos.x != x || pos.z != z) {
            return false;
        }

        return simulateClip(col, pos, angle, {-2191, -760, -266}, {-2121, -760, -247}, false, false);
    });
}

bool simulateGroundClipSetup(Collision* col, Vec3f pos, u16 angle, bool fakeGroundClip, bool debug) {
    PosAngleSetup setup(col, pos, angle);

    bool pushActivated = false;
    Vec3f hoverBombPos = instantDropBomb(setup.pos, setup.angle);
    if (!setup.performAction(SIDEHOP_RIGHT)) {
        return false;
    }

    if (Math_Vec3f_DistXZ(&pos, &hoverBombPos) >= 20.0f) {
        setup.addCollider(hoverBombPos.toVec3s(), 6, 0.8f);
        pushActivated = true;
    }

    if (!setup.performAction(SIDEHOP_LEFT)) {
        return false;
    }
    Vec3f damageBombPos = instantDropBomb(setup.pos, setup.angle);

    if (!pushActivated) {
        setup.addCollider(hoverBombPos.toVec3s(), 6, 0.8f);
    }

    if (!setup.performAction(SIDEHOP_RIGHT)) {
        return false;
    }
    if (!setup.performAction(ESS_TURN_LEFT)) {
        return false;
    }

    if (debug) {
        printf("hoverBombPos: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
            hoverBombPos.x, floatToInt(hoverBombPos.x), hoverBombPos.y, floatToInt(hoverBombPos.y), hoverBombPos.z, floatToInt(hoverBombPos.z));
        printf("damageBombPos: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x)\n",
            damageBombPos.x, floatToInt(damageBombPos.x), damageBombPos.y, floatToInt(damageBombPos.y), damageBombPos.z, floatToInt(damageBombPos.z));
        printf("final pos: x=%.9g (%08x) y=%.9g (%08x) z=%.9g (%08x) angle=%04x\n",
            setup.pos.x, floatToInt(setup.pos.x), setup.pos.y, floatToInt(setup.pos.y), setup.pos.z, floatToInt(setup.pos.z), setup.angle);
    }

    return simulateClip(col, setup.pos, setup.angle, hoverBombPos, damageBombPos, fakeGroundClip, debug);
}

void searchGroundClipPositions(Collision* col) {
    // Right side
    PosAngleRange range = {
        .angleMin = 0xe000,
        .angleMax = 0xe100,
        .xMin = -2167,
        .xMax = -2163,
        .xStep = 0.01f,
        .zMin = -264,
        .zMax = -262,
        .zStep = 0.01f,
    };
    // Left side
    // PosAngleRange range = {
    //     .angleMin = 0xdf00,
    //     .angleMax = 0xe200,
    //     .xMin = -2142,
    //     .xMax = -2122,
    //     .xStep = 0.1f,
    //     .zMin = -202,
    //     .zMax = -182,
    //     .zStep = 0.1f,
    // };
    searchPosAngleRange(range, [&](u16 angle, f32 x, f32 z) {
        Vec3f pos = col->findFloor({x, -700, z});
        if (pos.y == BGCHECK_Y_MIN) {
            return false;
        }

        pos = col->runChecks(pos, translate(pos, angle, 0.0f, -5.0f));
        if (pos.x != x || pos.z != z) {
            return false;
        }

        if (!simulateGroundClipSetup(col, pos, angle, false, false)) {
            return false;
        }

        printf("angle=%04x x=%.9g (%08x) z=%.9g (%08x)\n", angle, x, floatToInt(x), z, floatToInt(z));
        fflush(stdout);
        return true;
    });
}

void findSetups(Collision* col, int argc, char* argv[]) {
    SearchParams params = {
        .col = col,
        .minBounds = {-10000, -760, -10000},
        .maxBounds = {10000, 10000, 10000},
        .starts =
            {
                // roll into right corner (target left wall)
                {{intToFloat(0xc503f666), -760, intToFloat(0xc3a31437)}, 0x9f8a},
                // {{intToFloat(0xc503f067), -760, intToFloat(0xc3a34318)}, 0x9882},
                // {{intToFloat(0xc503e952), -760, intToFloat(0xc3a37a76)}, 0x917a},
                // {{intToFloat(0xc503e181), -760, intToFloat(0xc3a3b796)}, 0x8a72},
                // {{intToFloat(0xc503d907), -760, intToFloat(0xc3a3f9db)}, 0x836a},
                // {{intToFloat(0xc503d063), -760, intToFloat(0xc3a43d68)}, 0x7c62},
                // roll into right corner (target right wall)
                {{intToFloat(0xc503ce1a), -760, intToFloat(0xc3a44f44)}, 0x7a95},
                // {{intToFloat(0xc503d6ca), -760, intToFloat(0xc3a40b5b)}, 0x819d},
                // {{intToFloat(0xc503df60), -760, intToFloat(0xc3a3c83e)}, 0x88a5},
                // {{intToFloat(0xc503e75e), -760, intToFloat(0xc3a3c83e)}, 0x8fad},
                // {{intToFloat(0xc503eeae), -760, intToFloat(0xc3a35095)}, 0x96b6},
                // {{intToFloat(0xc503f4f5), -760, intToFloat(0xc3a31f7c)}, 0x9dbd},
                // target web, sidehop into right corner
                // {{intToFloat(0xc507c580), intToFloat(0xc43d5ca7), intToFloat(0xc392251c)}, 0xa033},
                // target web, sidehop sideroll into right corner
                {{intToFloat(0xc507c57f), intToFloat(0xc43d5ca5), intToFloat(0xc392251c)}, 0xa033},
                // target web, sidehop sideroll into left corner
                {{intToFloat(0xc50d347e), intToFloat(0xc43d5f75), intToFloat(0xc34c7303)}, 0xa033},
                // roll into left corner
                {{intToFloat(0xc50f9000), -760, intToFloat(0xc3089454)}, 0xc000},
            },
        .maxCost = 120,
        .angleMin = 0xe040,
        .angleMax = 0xe0cf,
        .xMin = -2165.98f,
        .xMax = -2164.16f,
        .zMin = -263.83f,
        .zMax = -262.77f,
        .actions =
            {
                TARGET_WALL,
                ROLL,
                BACKFLIP,
                BACKFLIP_SIDEROLL,
                BACKFLIP_SIDEROLL_UNTARGET,
                SIDEHOP_LEFT,
                SIDEHOP_LEFT_SIDEROLL,
                SIDEHOP_LEFT_SIDEROLL_UNTARGET,
                SIDEHOP_RIGHT,
                SIDEHOP_RIGHT_SIDEROLL,
                SIDEHOP_RIGHT_SIDEROLL_UNTARGET,
                ROTATE_ESS_LEFT,
                ROTATE_ESS_RIGHT,
                ESS_TURN_UP,
                ESS_TURN_LEFT,
                ESS_TURN_RIGHT,
                ESS_TURN_DOWN,
            },
    };

    auto filter = [&](Vec3f initialPos, u16 initialAngle,
                        const PosAngleSetup& setup, const std::vector<Action>& path,
                        int cost) {
        if (setup.pos.x > -2035.0f && setup.pos.z < -280.0f) {
            return false;  // Too close to bushes
        }

        return true;
    };

    auto output = [&](Vec3f initialPos, u16 initialAngle,
                        const PosAngleSetup& setup, const std::vector<Action>& path,
                        int cost) {
        if (simulateGroundClipSetup(col, setup.pos, setup.angle, false, false)) {
            printf(
                "found: cost=%d startAngle=%04x startx=%.9g (%08x) startz=%.9g (%08x) angle=%04x x=%.9g "
                "(%08x) z=%.9g (%08x) actions=%s\n",
                cost, initialAngle, initialPos.x, floatToInt(initialPos.x), initialPos.z, floatToInt(initialPos.z), setup.angle,
                setup.pos.x, floatToInt(setup.pos.x), setup.pos.z, floatToInt(setup.pos.z), actionNames(path).c_str());
            fflush(stdout);
            return true;
        } else {
            return false;
        }
    };

    if (argc > 1) {
        int shard = atoi(argv[1]);
        searchSetupsShard(params, 2, shard, filter, output);
    } else {
        searchSetups(params, filter, output);
    }
}

int main(int argc, char* argv[]) {
    Collision col(&ydan_sceneCollisionHeader_00B618, PLAYER_AGE_ADULT, {-2315, -760, -348}, {-2115, -740, -148});
    col.addDynapoly(&gDTWebWallCol, {0.1f, 0.1f, 0.1f}, {0x0000, 0x2000, 0x0000}, {-2229, -760, -262});
    // col.printPolys();

    // simulateClip(&col, {intToFloat(0xc507c57f), intToFloat(0xc43d5ca5), intToFloat(0xc392251b)}, 0x2039,
    //     {-2191, -760, -266}, {-2121, -760, -247}, false, true);
    // searchClipPositions(&col);

    // simulateGroundClipSetup(&col, {intToFloat(0xc50753ed), -760, intToFloat(0xc383d3b6)}, 0xe050, false, true);
    // searchGroundClipPositions(&col);

    findSetups(&col, argc, argv);

    return 0;
}
