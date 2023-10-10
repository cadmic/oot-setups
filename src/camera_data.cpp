#include "camera_data.hpp"

// TODO
#define CAM_INTERFACE_FIELD(letterboxFlag, hudVisibilityMode, funcFlags) 0

#define CAM_FUNCDATA_NORM1(yOffset, distMin, distMax, pitchTarget,             \
                           yawUpdateRateTarget, pitchUpdateRateTarget,         \
                           maxYawUpdate, fov, atLerpStepScale, interfaceField) \
  {                                                                            \
    yOffset, distMin, distMax, pitchTarget, yawUpdateRateTarget,               \
        pitchUpdateRateTarget, maxYawUpdate, fov, atLerpStepScale,             \
        interfaceField                                                         \
  }

#define CAM_FUNCDATA_PARA1(yOffset, dist, pitchTarget, yawTarget,          \
                           yawUpdateRateTarget, xzUpdateRateTarget, fov,   \
                           atLerpStepScale, interfaceField, groundYOffset, \
                           groundAtLerpStepScale)                          \
  {                                                                        \
    yOffset, dist, pitchTarget, yawTarget, yawUpdateRateTarget,            \
        xzUpdateRateTarget, fov, atLerpStepScale, interfaceField,          \
        groundYOffset, groundAtLerpStepScale                               \
  }

// TODO: import more camera data
CameraNormalSettings cameraNormalSettings[] = {
    {},  // CAM_SET_NONE
    CAM_FUNCDATA_NORM1(
        -20, 200, 300, 10, 12, 10, 35, 60, 60,
        CAM_INTERFACE_FIELD(
            CAM_LETTERBOX_NONE, CAM_HUD_VISIBILITY_ALL,
            NORMAL1_FLAG_1 | NORMAL1_FLAG_0)),  // CAM_SET_NORMAL0
    CAM_FUNCDATA_NORM1(
        0, 200, 400, 10, 12, 20, 40, 60, 60,
        CAM_INTERFACE_FIELD(
            CAM_LETTERBOX_NONE, CAM_HUD_VISIBILITY_ALL,
            NORMAL1_FLAG_1 | NORMAL1_FLAG_0)),  // CAM_SET_NORMAL1
    CAM_FUNCDATA_NORM1(
        -10, 150, 250, 5, 10, 5, 30, 60, 60,
        CAM_INTERFACE_FIELD(
            CAM_LETTERBOX_NONE, CAM_HUD_VISIBILITY_ALL,
            NORMAL1_FLAG_1 | NORMAL1_FLAG_0)),  // CAM_SET_DUNGEON0
    CAM_FUNCDATA_NORM1(
        -40, 150, 150, 0, 10, 5, 30, 60, 60,
        CAM_INTERFACE_FIELD(
            CAM_LETTERBOX_NONE, CAM_HUD_VISIBILITY_ALL,
            NORMAL1_FLAG_1 | NORMAL1_FLAG_0)),  // CAM_SET_DUNGEON1
    {},                                         // CAM_SET_NORMAL3
    {},                                         // CAM_SET_HORSE
    {},                                         // CAM_SET_BOSS_GOHMA
    {},                                         // CAM_SET_BOSS_DODONGO
    {},                                         // CAM_SET_BOSS_BARINADE
    {},                                         // CAM_SET_BOSS_PHANTOM_GANON
    {},                                         // CAM_SET_BOSS_VOLVAGIA
    {},                                         // CAM_SET_BOSS_BONGO
    {},                                         // CAM_SET_BOSS_MORPHA
    {},  // CAM_SET_BOSS_TWINROVA_PLATFORM
    {},  // CAM_SET_BOSS_TWINROVA_FLOOR
    {},  // CAM_SET_BOSS_GANONDORF
    {},  // CAM_SET_BOSS_GANON
    {},  // CAM_SET_TOWER_CLIMB
    {},  // CAM_SET_TOWER_UNUSED
    {},  // CAM_SET_MARKET_BALCONY
    {},  // CAM_SET_CHU_BOWLING
    {},  // CAM_SET_PIVOT_CRAWLSPACE
    {},  // CAM_SET_PIVOT_SHOP_BROWSING
    {},  // CAM_SET_PIVOT_IN_FRONT
    {},  // CAM_SET_PREREND_FIXED
    {},  // CAM_SET_PREREND_PIVOT
    {},  // CAM_SET_PREREND_SIDE_SCROLL
    {},  // CAM_SET_DOOR0
    {},  // CAM_SET_DOORC
    {},  // CAM_SET_CRAWLSPACE
    {},  // CAM_SET_START0
    {},  // CAM_SET_START1
    {},  // CAM_SET_FREE0
    {},  // CAM_SET_FREE2
    {},  // CAM_SET_PIVOT_CORNER
    {},  // CAM_SET_PIVOT_WATER_SURFACE
    {},  // CAM_SET_CS_0
    {},  // CAM_SET_CS_TWISTED_HALLWAY
    {},  // CAM_SET_FOREST_BIRDS_EYE
    {},  // CAM_SET_SLOW_CHEST_CS
    {},  // CAM_SET_ITEM_UNUSED
    {},  // CAM_SET_CS_3
    {},  // CAM_SET_CS_ATTENTION
    {},  // CAM_SET_BEAN_GENERIC
    {},  // CAM_SET_BEAN_LOST_WOODS
    {},  // CAM_SET_SCENE_UNUSED
    {},  // CAM_SET_SCENE_TRANSITION
    {},  // CAM_SET_ELEVATOR_PLATFORM
    {},  // CAM_SET_FIRE_STAIRCASE
    {},  // CAM_SET_FOREST_UNUSED
    {},  // CAM_SET_FOREST_DEFEAT_POE
    {},  // CAM_SET_BIG_OCTO
    {},  // CAM_SET_MEADOW_BIRDS_EYE
    {},  // CAM_SET_MEADOW_UNUSED
    {},  // CAM_SET_FIRE_BIRDS_EYE
    {},  // CAM_SET_TURN_AROUND
    {},  // CAM_SET_PIVOT_VERTICAL
    {},  // CAM_SET_NORMAL2
    {},  // CAM_SET_FISHING
    {},  // CAM_SET_CS_C
    {},  // CAM_SET_JABU_TENTACLE
    CAM_FUNCDATA_NORM1(
        -20, 350, 350, 20, 15, 5, 30, 60, 60,
        CAM_INTERFACE_FIELD(
            CAM_LETTERBOX_NONE, CAM_HUD_VISIBILITY_ALL,
            NORMAL1_FLAG_1 | NORMAL1_FLAG_0)),  // CAM_SET_DUNGEON2
    {},                                         // CAM_SET_DIRECTED_YAW
    {},                                         // CAM_SET_PIVOT_FROM_SIDE
    {},                                         // CAM_SET_NORMAL4
    {},                                         // CAM_SET_MAX
};

CameraZParallelSettings cameraZParallelSettings[] = {
    {},  // CAM_SET_NONE
    CAM_FUNCDATA_PARA1(
        -20, 250, 0, 0, 5, 5, 45, 50,
        CAM_INTERFACE_FIELD(CAM_LETTERBOX_MEDIUM, CAM_HUD_VISIBILITY_ALL,
                            PARALLEL1_FLAG_3 | PARALLEL1_FLAG_1),
        -40,
        20),  // CAM_SET_NORMAL0
    CAM_FUNCDATA_PARA1(
        0, 250, 0, 0, 5, 5, 45, 50,
        CAM_INTERFACE_FIELD(CAM_LETTERBOX_MEDIUM, CAM_HUD_VISIBILITY_ALL,
                            PARALLEL1_FLAG_1),
        -40, 20),  // CAM_SET_NORMAL1
    CAM_FUNCDATA_PARA1(
        -20, 150, 0, 0, 5, 5, 45, 50,
        CAM_INTERFACE_FIELD(CAM_LETTERBOX_MEDIUM, CAM_HUD_VISIBILITY_ALL,
                            PARALLEL1_FLAG_3 | PARALLEL1_FLAG_1),
        -40,
        20),  // CAM_SET_DUNGEON0
    CAM_FUNCDATA_PARA1(
        -20, 150, 0, 0, 5, 5, 45, 50,
        CAM_INTERFACE_FIELD(CAM_LETTERBOX_MEDIUM, CAM_HUD_VISIBILITY_ALL,
                            PARALLEL1_FLAG_3 | PARALLEL1_FLAG_1),
        -40,
        20),  // CAM_SET_DUNGEON1
    {},       // CAM_SET_NORMAL3
    {},       // CAM_SET_HORSE
    {},       // CAM_SET_BOSS_GOHMA
    {},       // CAM_SET_BOSS_DODONGO
    {},       // CAM_SET_BOSS_BARINADE
    {},       // CAM_SET_BOSS_PHANTOM_GANON
    {},       // CAM_SET_BOSS_VOLVAGIA
    {},       // CAM_SET_BOSS_BONGO
    {},       // CAM_SET_BOSS_MORPHA
    {},       // CAM_SET_BOSS_TWINROVA_PLATFORM
    {},       // CAM_SET_BOSS_TWINROVA_FLOOR
    {},       // CAM_SET_BOSS_GANONDORF
    {},       // CAM_SET_BOSS_GANON
    {},       // CAM_SET_TOWER_CLIMB
    {},       // CAM_SET_TOWER_UNUSED
    {},       // CAM_SET_MARKET_BALCONY
    {},       // CAM_SET_CHU_BOWLING
    {},       // CAM_SET_PIVOT_CRAWLSPACE
    {},       // CAM_SET_PIVOT_SHOP_BROWSING
    {},       // CAM_SET_PIVOT_IN_FRONT
    {},       // CAM_SET_PREREND_FIXED
    {},       // CAM_SET_PREREND_PIVOT
    {},       // CAM_SET_PREREND_SIDE_SCROLL
    {},       // CAM_SET_DOOR0
    {},       // CAM_SET_DOORC
    {},       // CAM_SET_CRAWLSPACE
    {},       // CAM_SET_START0
    {},       // CAM_SET_START1
    {},       // CAM_SET_FREE0
    {},       // CAM_SET_FREE2
    {},       // CAM_SET_PIVOT_CORNER
    {},       // CAM_SET_PIVOT_WATER_SURFACE
    {},       // CAM_SET_CS_0
    {},       // CAM_SET_CS_TWISTED_HALLWAY
    {},       // CAM_SET_FOREST_BIRDS_EYE
    {},       // CAM_SET_SLOW_CHEST_CS
    {},       // CAM_SET_ITEM_UNUSED
    {},       // CAM_SET_CS_3
    {},       // CAM_SET_CS_ATTENTION
    {},       // CAM_SET_BEAN_GENERIC
    {},       // CAM_SET_BEAN_LOST_WOODS
    {},       // CAM_SET_SCENE_UNUSED
    {},       // CAM_SET_SCENE_TRANSITION
    {},       // CAM_SET_ELEVATOR_PLATFORM
    {},       // CAM_SET_FIRE_STAIRCASE
    {},       // CAM_SET_FOREST_UNUSED
    {},       // CAM_SET_FOREST_DEFEAT_POE
    {},       // CAM_SET_BIG_OCTO
    {},       // CAM_SET_MEADOW_BIRDS_EYE
    {},       // CAM_SET_MEADOW_UNUSED
    {},       // CAM_SET_FIRE_BIRDS_EYE
    {},       // CAM_SET_TURN_AROUND
    {},       // CAM_SET_PIVOT_VERTICAL
    {},       // CAM_SET_NORMAL2
    {},       // CAM_SET_FISHING
    {},       // CAM_SET_CS_C
    {},       // CAM_SET_JABU_TENTACLE
    CAM_FUNCDATA_PARA1(
        -20, 200, 0, 0, 5, 5, 45, 50,
        CAM_INTERFACE_FIELD(CAM_LETTERBOX_MEDIUM, CAM_HUD_VISIBILITY_ALL,
                            PARALLEL1_FLAG_3 | PARALLEL1_FLAG_1),
        -40,
        20),  // CAM_SET_DUNGEON2
    {},       // CAM_SET_DIRECTED_YAW
    {},       // CAM_SET_PIVOT_FROM_SIDE
    {},       // CAM_SET_NORMAL4
    {},       // CAM_SET_MAX
};
