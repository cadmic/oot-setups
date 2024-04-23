#include "global.hpp"
#include "sys_math.hpp"
#include "sys_math3d.hpp"
#include "sys_matrix.hpp"

#define DOTXYZ(vec1, vec2) ((vec1).x * (vec2).x + (vec1).y * (vec2).y + (vec1).z * (vec2).z)

void EnBomChu_CrossProduct(Vec3f* a, Vec3f* b, Vec3f* dest) {
    dest->x = (a->y * b->z) - (a->z * b->y);
    dest->y = (a->z * b->x) - (a->x * b->z);
    dest->z = (a->x * b->y) - (a->y * b->x);
}

bool testOobChu(u16 facingAngle) {
    u16 arg = facingAngle + 0x8000;

    s32 t6 = arg;
    s32 t7 = arg >> 4;
    s32 t8 = t7;

    if (arg & 0x800) {
        s32 result = sins(arg);
        t6 = result << 16;
    }

    Vec3f axisForwards;
    Vec3f axisUp;
    Vec3f axisLeft;

    // rot.y = 0 -> +z (forwards in model space)
    axisForwards.x = Math_SinS(facingAngle);
    axisForwards.y = 0.0f;
    axisForwards.z = Math_CosS(facingAngle);

    // +y (up in model space)
    axisUp.x = 0.0f;
    axisUp.y = 1.0f;
    axisUp.z = 0.0f;

    // rot.y = 0 -> +x (left in model space)
    axisLeft.x = Math_SinS(facingAngle + 0x4000);
    axisLeft.y = 0;
    axisLeft.z = Math_CosS(facingAngle + 0x4000);

    Vec3f normal;
    normal.x = (f32)t6 * (1.0f / SHT_MAX);
    normal.y = (f32)t7 * (1.0f / SHT_MAX);
    normal.z = (f32)t8 * (1.0f / SHT_MAX);

    f32 normDotUp = DOTXYZ(normal, axisUp);

    if (!(fabsf(normDotUp) >= 1.0f)) {
        f32 angle = Math_FAcosF(normDotUp);

        if (!(angle < 0.001f)) {
            Vec3f vec;

            EnBomChu_CrossProduct(&axisUp, &normal, &vec);
            //! @bug this function expects a unit vector but `vec` is not normalized
            Matrix_RotateAxis(angle, &vec, MTXMODE_NEW);

            Matrix_MultVec3f(&axisLeft, &vec);
            axisLeft = vec;

            EnBomChu_CrossProduct(&axisLeft, &normal, &axisForwards);

            f32 magnitude = Math3D_Vec3fMagnitude(&axisForwards);

            if (magnitude < 0.001f) {
                return true;
            }
        }
    }

    return false;
}

int main(int argc, char* argv[]) {
    for (int i = 0; i < 0x10000; i++) {
        if (testOobChu(i)) {
            printf("angle=%04x\n", i);
        }
    }

    return 0;
}
