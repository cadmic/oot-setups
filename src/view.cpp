#include "global.hpp"
#include "sys_matrix.hpp"
#include "skin_matrix.hpp"
#include "view.hpp"

#define GU_PI 3.1415926

#define FTOFIX32(x) (s32)((x) * (f32)0x00010000)

void guMtxF2L(f32 mf[4][4], Mtx* m) {
    s32 i, j;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            s32 e = FTOFIX32(mf[i][j]);
            m->intPart[i][j] = e >> 16;
            m->fracPart[i][j] = e & 0xFFFF;
        }
    }
}

void guMtxIdentF(f32 mf[4][4]) {
    s32 i, j;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            if (i == j) {
                mf[i][j] = 1.0;
            } else {
                mf[i][j] = 0.0;
            }
        }
    }
}

void guPerspectiveF(f32 mf[4][4], u16* perspNorm, f32 fovy, f32 aspect, f32 near, f32 far, f32 scale) {
    f32 yscale;
    s32 row;
    s32 col;

    guMtxIdentF(mf);

    fovy *= GU_PI / 180.0;
    yscale = cosf(fovy / 2) / sinf(fovy / 2);
    mf[0][0] = yscale / aspect;
    mf[1][1] = yscale;
    mf[2][2] = (near + far) / (near - far);
    mf[2][3] = -1;
    mf[3][2] = 2 * near * far / (near - far);
    mf[3][3] = 0.0f;

    for (row = 0; row < 4; row++) {
        for (col = 0; col < 4; col++) {
            mf[row][col] *= scale;
        }
    }

    if (perspNorm != NULL) {
        if (near + far <= 2.0) {
            *perspNorm = 65535;
        } else {
            *perspNorm = (f64)(1 << 17) / (near + far);
            if (*perspNorm <= 0) {
                *perspNorm = 1;
            }
        }
    }
}

void guPerspective(Mtx* m, u16* perspNorm, f32 fovy, f32 aspect, f32 near, f32 far, f32 scale) {
    f32 mf[4][4];

    guPerspectiveF(mf, perspNorm, fovy, aspect, near, far, scale);

    guMtxF2L(mf, m);
}

void guLookAtF(f32 mf[4][4], f32 xEye, f32 yEye, f32 zEye, f32 xAt, f32 yAt, f32 zAt, f32 xUp, f32 yUp, f32 zUp) {
    f32 length;
    f32 xLook;
    f32 yLook;
    f32 zLook;
    f32 xRight;
    f32 yRight;
    f32 zRight;

    guMtxIdentF(mf);

    xLook = xAt - xEye;
    yLook = yAt - yEye;
    zLook = zAt - zEye;
    length = -1.0 / sqrtf(SQ(xLook) + SQ(yLook) + SQ(zLook));
    xLook *= length;
    yLook *= length;
    zLook *= length;

    xRight = yUp * zLook - zUp * yLook;
    yRight = zUp * xLook - xUp * zLook;
    zRight = xUp * yLook - yUp * xLook;
    length = 1.0 / sqrtf(SQ(xRight) + SQ(yRight) + SQ(zRight));
    xRight *= length;
    yRight *= length;
    zRight *= length;

    xUp = yLook * zRight - zLook * yRight;
    yUp = zLook * xRight - xLook * zRight;
    zUp = xLook * yRight - yLook * xRight;
    length = 1.0 / sqrtf(SQ(xUp) + SQ(yUp) + SQ(zUp));
    xUp *= length;
    yUp *= length;
    zUp *= length;

    mf[0][0] = xRight;
    mf[1][0] = yRight;
    mf[2][0] = zRight;
    mf[3][0] = -(xEye * xRight + yEye * yRight + zEye * zRight);

    mf[0][1] = xUp;
    mf[1][1] = yUp;
    mf[2][1] = zUp;
    mf[3][1] = -(xEye * xUp + yEye * yUp + zEye * zUp);

    mf[0][2] = xLook;
    mf[1][2] = yLook;
    mf[2][2] = zLook;
    mf[3][2] = -(xEye * xLook + yEye * yLook + zEye * zLook);

    mf[0][3] = 0;
    mf[1][3] = 0;
    mf[2][3] = 0;
    mf[3][3] = 1;
}

void guLookAt(Mtx* m, f32 xEye, f32 yEye, f32 zEye, f32 xAt, f32 yAt, f32 zAt, f32 xUp, f32 yUp, f32 zUp) {
    f32 mf[4][4];

    guLookAtF(mf, xEye, yEye, zEye, xAt, yAt, zAt, xUp, yUp, zUp);

    guMtxF2L(mf, m);
}

bool isCulled(Camera* camera, CullZone* zone, Vec3f pos, f32 fovy, f32 near, f32 far) {
    // Projection matrix
    Mtx projectionMtx;
    u16 perspNorm;
    guPerspective(&projectionMtx, &perspNorm, fovy, 4.0f / 3.0f, near, far, 1.0f);

    // Viewing matrix
    Mtx viewingMtx;
    Vec3f up = {0.0f, 1.0f, 0.0f};
    guLookAt(&viewingMtx, camera->eye.x, camera->eye.y, camera->eye.z,
             camera->at.x, camera->at.y, camera->at.z, up.x, up.y, up.z);

    // Get combined projection matrix
    MtxF viewingMtxF;
    MtxF projectionMtxF;

    Matrix_MtxToMtxF(&viewingMtx, &viewingMtxF);
    Matrix_MtxToMtxF(&projectionMtx, &projectionMtxF);

    Matrix_Mult(&projectionMtxF, MTXMODE_NEW);
    Matrix_Mult(&viewingMtxF, MTXMODE_APPLY);
    Matrix_Get(&projectionMtxF);

    // Compute screen position
    Vec3f projectedPos;
    f32 projectedW;
    SkinMatrix_Vec3fMtxFMultXYZW(&projectionMtxF, &pos, &projectedPos, &projectedW);

    // Culling logic from func_800314D4
    if (projectedPos.z > -zone->scale && projectedPos.z < zone->forward + zone->scale) {
       f32 invProjectedW = (projectedW < 1.0f) ? 1.0f : 1.0f / projectedW;
        if ((fabsf(projectedPos.x) - zone->scale) * invProjectedW < 1.0f &&
            (projectedPos.y + zone->downward) * invProjectedW > -1.0f &&
            (projectedPos.y - zone->scale) * invProjectedW < 1.0f) {
            return false;
        }
    }

    return true;
}
