#include "sys_matrix.hpp"

#include "skin_matrix.hpp"
#include "sys_math.hpp"

// Casting a float to an integer, when the float value is larger than what the
// integer type can hold, leads to undefined behavior. For example (f32)0x8000
// doesn't fit in a s16, so it cannot be cast to s16. This isn't an issue with
// IDO, but is one with for example GCC. A partial workaround is to cast to s32
// then s16, hoping all binang values used will fit a s32.
#define TRUNCF_BINANG(f) (s16)(s32)(f)
#define RAD_TO_BINANG(radians) (s16) TRUNCF_BINANG((radians) * (0x8000 / M_PI))

#define ASSERT(...)

// clang-format off
MtxF gMtxFClear = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f,
};
// clang-format on

MtxF gMatrixStack[20];

MtxF* sMatrixStack = &gMatrixStack[0];    // "Matrix_stack"
MtxF* sCurrentMatrix = &gMatrixStack[0];  // "Matrix_now"

void Matrix_Push(void) {
  Matrix_MtxFCopy(sCurrentMatrix + 1, sCurrentMatrix);
  sCurrentMatrix++;
}

void Matrix_Pop(void) {
  sCurrentMatrix--;
  ASSERT(sCurrentMatrix >= sMatrixStack, "Matrix_now >= Matrix_stack",
         "../sys_matrix.c", 176);
}

void Matrix_Get(MtxF* dest) { Matrix_MtxFCopy(dest, sCurrentMatrix); }

void Matrix_Put(MtxF* src) { Matrix_MtxFCopy(sCurrentMatrix, src); }

MtxF* Matrix_GetCurrent(void) { return sCurrentMatrix; }

void Matrix_Mult(MtxF* mf, u8 mode) {
  MtxF* cmf = Matrix_GetCurrent();

  if (mode == MTXMODE_APPLY) {
    SkinMatrix_MtxFMtxFMult(cmf, mf, cmf);
  } else {
    Matrix_MtxFCopy(sCurrentMatrix, mf);
  }
}

void Matrix_Translate(f32 x, f32 y, f32 z, u8 mode) {
  MtxF* cmf = sCurrentMatrix;
  f32 tx;
  f32 ty;

  if (mode == MTXMODE_APPLY) {
    tx = cmf->xx;
    ty = cmf->xy;
    cmf->xw += tx * x + ty * y + cmf->xz * z;
    tx = cmf->yx;
    ty = cmf->yy;
    cmf->yw += tx * x + ty * y + cmf->yz * z;
    tx = cmf->zx;
    ty = cmf->zy;
    cmf->zw += tx * x + ty * y + cmf->zz * z;
    tx = cmf->wx;
    ty = cmf->wy;
    cmf->ww += tx * x + ty * y + cmf->wz * z;
  } else {
    SkinMatrix_SetTranslate(cmf, x, y, z);
  }
}

void Matrix_Scale(f32 x, f32 y, f32 z, u8 mode) {
  MtxF* cmf = sCurrentMatrix;

  if (mode == MTXMODE_APPLY) {
    cmf->xx *= x;
    cmf->yx *= x;
    cmf->zx *= x;
    cmf->xy *= y;
    cmf->yy *= y;
    cmf->zy *= y;
    cmf->xz *= z;
    cmf->yz *= z;
    cmf->zz *= z;
    cmf->wx *= x;
    cmf->wy *= y;
    cmf->wz *= z;
  } else {
    SkinMatrix_SetScale(cmf, x, y, z);
  }
}

void Matrix_RotateX(f32 x, u8 mode) {
  MtxF* cmf;
  f32 sin;
  f32 cos;
  f32 temp1;
  f32 temp2;

  if (mode == MTXMODE_APPLY) {
    if (x != 0) {
      cmf = sCurrentMatrix;

      sin = sinf(x);
      cos = cosf(x);

      temp1 = cmf->xy;
      temp2 = cmf->xz;
      cmf->xy = temp1 * cos + temp2 * sin;
      cmf->xz = temp2 * cos - temp1 * sin;

      temp1 = cmf->yy;
      temp2 = cmf->yz;
      cmf->yy = temp1 * cos + temp2 * sin;
      cmf->yz = temp2 * cos - temp1 * sin;

      temp1 = cmf->zy;
      temp2 = cmf->zz;
      cmf->zy = temp1 * cos + temp2 * sin;
      cmf->zz = temp2 * cos - temp1 * sin;

      temp1 = cmf->wy;
      temp2 = cmf->wz;
      cmf->wy = temp1 * cos + temp2 * sin;
      cmf->wz = temp2 * cos - temp1 * sin;
    }
  } else {
    cmf = sCurrentMatrix;

    if (x != 0) {
      sin = sinf(x);
      cos = cosf(x);
    } else {
      sin = 0.0f;
      cos = 1.0f;
    }

    cmf->yx = 0.0f;
    cmf->zx = 0.0f;
    cmf->wx = 0.0f;
    cmf->xy = 0.0f;
    cmf->wy = 0.0f;
    cmf->xz = 0.0f;
    cmf->wz = 0.0f;
    cmf->xw = 0.0f;
    cmf->yw = 0.0f;
    cmf->zw = 0.0f;
    cmf->xx = 1.0f;
    cmf->ww = 1.0f;
    cmf->yy = cos;
    cmf->zz = cos;
    cmf->zy = sin;
    cmf->yz = -sin;
  }
}

void Matrix_RotateY(f32 y, u8 mode) {
  MtxF* cmf;
  f32 sin;
  f32 cos;
  f32 temp1;
  f32 temp2;

  if (mode == MTXMODE_APPLY) {
    if (y != 0) {
      cmf = sCurrentMatrix;

      sin = sinf(y);
      cos = cosf(y);

      temp1 = cmf->xx;
      temp2 = cmf->xz;
      cmf->xx = temp1 * cos - temp2 * sin;
      cmf->xz = temp1 * sin + temp2 * cos;

      temp1 = cmf->yx;
      temp2 = cmf->yz;
      cmf->yx = temp1 * cos - temp2 * sin;
      cmf->yz = temp1 * sin + temp2 * cos;

      temp1 = cmf->zx;
      temp2 = cmf->zz;
      cmf->zx = temp1 * cos - temp2 * sin;
      cmf->zz = temp1 * sin + temp2 * cos;

      temp1 = cmf->wx;
      temp2 = cmf->wz;
      cmf->wx = temp1 * cos - temp2 * sin;
      cmf->wz = temp1 * sin + temp2 * cos;
    }
  } else {
    cmf = sCurrentMatrix;

    if (y != 0) {
      sin = sinf(y);
      cos = cosf(y);
    } else {
      sin = 0.0f;
      cos = 1.0f;
    }

    cmf->yx = 0.0f;
    cmf->wx = 0.0f;
    cmf->xy = 0.0f;
    cmf->zy = 0.0f;
    cmf->wy = 0.0f;
    cmf->yz = 0.0f;
    cmf->wz = 0.0f;
    cmf->xw = 0.0f;
    cmf->yw = 0.0f;
    cmf->zw = 0.0f;
    cmf->yy = 1.0f;
    cmf->ww = 1.0f;
    cmf->xx = cos;
    cmf->zz = cos;
    cmf->zx = -sin;
    cmf->xz = sin;
  }
}

void Matrix_RotateZ(f32 z, u8 mode) {
  MtxF* cmf;
  f32 sin;
  f32 cos;
  f32 temp1;
  f32 temp2;

  if (mode == MTXMODE_APPLY) {
    if (z != 0) {
      cmf = sCurrentMatrix;

      sin = sinf(z);
      cos = cosf(z);

      temp1 = cmf->xx;
      temp2 = cmf->xy;
      cmf->xx = temp1 * cos + temp2 * sin;
      cmf->xy = temp2 * cos - temp1 * sin;

      temp1 = cmf->yx;
      temp2 = cmf->yy;
      cmf->yx = temp1 * cos + temp2 * sin;
      cmf->yy = temp2 * cos - temp1 * sin;

      temp1 = cmf->zx;
      temp2 = cmf->zy;
      cmf->zx = temp1 * cos + temp2 * sin;
      cmf->zy = temp2 * cos - temp1 * sin;

      temp1 = cmf->wx;
      temp2 = cmf->wy;
      cmf->wx = temp1 * cos + temp2 * sin;
      cmf->wy = temp2 * cos - temp1 * sin;
    }
  } else {
    cmf = sCurrentMatrix;

    if (z != 0) {
      sin = sinf(z);
      cos = cosf(z);
    } else {
      sin = 0.0f;
      cos = 1.0f;
    }

    cmf->zx = 0.0f;
    cmf->wx = 0.0f;
    cmf->zy = 0.0f;
    cmf->wy = 0.0f;
    cmf->xz = 0.0f;
    cmf->yz = 0.0f;
    cmf->wz = 0.0f;
    cmf->xw = 0.0f;
    cmf->yw = 0.0f;
    cmf->zw = 0.0f;
    cmf->zz = 1.0f;
    cmf->ww = 1.0f;
    cmf->xx = cos;
    cmf->yy = cos;
    cmf->yx = sin;
    cmf->xy = -sin;
  }
}

/**
 * Rotate using ZYX Tait-Bryan angles.
 * This means a (column) vector is first rotated around X, then around Y, then
 * around Z, then (if `mode` is `MTXMODE_APPLY`) gets transformed according to
 * whatever the matrix was before adding the ZYX rotation. Original Name:
 * Matrix_RotateXYZ, changed to reflect rotation order.
 */
void Matrix_RotateZYX(s16 x, s16 y, s16 z, u8 mode) {
  MtxF* cmf = sCurrentMatrix;
  f32 temp1;
  f32 temp2;
  f32 sin;
  f32 cos;

  if (mode == MTXMODE_APPLY) {
    sin = Math_SinS(z);
    cos = Math_CosS(z);

    temp1 = cmf->xx;
    temp2 = cmf->xy;
    cmf->xx = temp1 * cos + temp2 * sin;
    cmf->xy = temp2 * cos - temp1 * sin;

    temp1 = cmf->yx;
    temp2 = cmf->yy;
    cmf->yx = temp1 * cos + temp2 * sin;
    cmf->yy = temp2 * cos - temp1 * sin;

    temp1 = cmf->zx;
    temp2 = cmf->zy;
    cmf->zx = temp1 * cos + temp2 * sin;
    cmf->zy = temp2 * cos - temp1 * sin;

    temp1 = cmf->wx;
    temp2 = cmf->wy;
    cmf->wx = temp1 * cos + temp2 * sin;
    cmf->wy = temp2 * cos - temp1 * sin;

    if (y != 0) {
      sin = Math_SinS(y);
      cos = Math_CosS(y);

      temp1 = cmf->xx;
      temp2 = cmf->xz;
      cmf->xx = temp1 * cos - temp2 * sin;
      cmf->xz = temp1 * sin + temp2 * cos;

      temp1 = cmf->yx;
      temp2 = cmf->yz;
      cmf->yx = temp1 * cos - temp2 * sin;
      cmf->yz = temp1 * sin + temp2 * cos;

      temp1 = cmf->zx;
      temp2 = cmf->zz;
      cmf->zx = temp1 * cos - temp2 * sin;
      cmf->zz = temp1 * sin + temp2 * cos;

      temp1 = cmf->wx;
      temp2 = cmf->wz;
      cmf->wx = temp1 * cos - temp2 * sin;
      cmf->wz = temp1 * sin + temp2 * cos;
    }

    if (x != 0) {
      sin = Math_SinS(x);
      cos = Math_CosS(x);

      temp1 = cmf->xy;
      temp2 = cmf->xz;
      cmf->xy = temp1 * cos + temp2 * sin;
      cmf->xz = temp2 * cos - temp1 * sin;

      temp1 = cmf->yy;
      temp2 = cmf->yz;
      cmf->yy = temp1 * cos + temp2 * sin;
      cmf->yz = temp2 * cos - temp1 * sin;

      temp1 = cmf->zy;
      temp2 = cmf->zz;
      cmf->zy = temp1 * cos + temp2 * sin;
      cmf->zz = temp2 * cos - temp1 * sin;

      temp1 = cmf->wy;
      temp2 = cmf->wz;
      cmf->wy = temp1 * cos + temp2 * sin;
      cmf->wz = temp2 * cos - temp1 * sin;
    }
  } else {
    SkinMatrix_SetRotateZYX(cmf, x, y, z);
  }
}

/**
 * Translate and rotate using ZYX Tait-Bryan angles.
 * This means a (column) vector is first rotated around X, then around Y, then
 * around Z, then translated, then gets transformed according to whatever the
 * matrix was previously.
 */
void Matrix_TranslateRotateZYX(Vec3f* translation, Vec3s* rotation) {
  MtxF* cmf = sCurrentMatrix;
  f32 sin = Math_SinS(rotation->z);
  f32 cos = Math_CosS(rotation->z);
  f32 temp1;
  f32 temp2;

  temp1 = cmf->xx;
  temp2 = cmf->xy;
  cmf->xw += temp1 * translation->x + temp2 * translation->y +
             cmf->xz * translation->z;
  cmf->xx = temp1 * cos + temp2 * sin;
  cmf->xy = temp2 * cos - temp1 * sin;

  temp1 = cmf->yx;
  temp2 = cmf->yy;
  cmf->yw += temp1 * translation->x + temp2 * translation->y +
             cmf->yz * translation->z;
  cmf->yx = temp1 * cos + temp2 * sin;
  cmf->yy = temp2 * cos - temp1 * sin;

  temp1 = cmf->zx;
  temp2 = cmf->zy;
  cmf->zw += temp1 * translation->x + temp2 * translation->y +
             cmf->zz * translation->z;
  cmf->zx = temp1 * cos + temp2 * sin;
  cmf->zy = temp2 * cos - temp1 * sin;

  temp1 = cmf->wx;
  temp2 = cmf->wy;
  cmf->ww += temp1 * translation->x + temp2 * translation->y +
             cmf->wz * translation->z;
  cmf->wx = temp1 * cos + temp2 * sin;
  cmf->wy = temp2 * cos - temp1 * sin;

  if (rotation->y != 0) {
    sin = Math_SinS(rotation->y);
    cos = Math_CosS(rotation->y);

    temp1 = cmf->xx;
    temp2 = cmf->xz;
    cmf->xx = temp1 * cos - temp2 * sin;
    cmf->xz = temp1 * sin + temp2 * cos;

    temp1 = cmf->yx;
    temp2 = cmf->yz;
    cmf->yx = temp1 * cos - temp2 * sin;
    cmf->yz = temp1 * sin + temp2 * cos;

    temp1 = cmf->zx;
    temp2 = cmf->zz;
    cmf->zx = temp1 * cos - temp2 * sin;
    cmf->zz = temp1 * sin + temp2 * cos;

    temp1 = cmf->wx;
    temp2 = cmf->wz;
    cmf->wx = temp1 * cos - temp2 * sin;
    cmf->wz = temp1 * sin + temp2 * cos;
  }

  if (rotation->x != 0) {
    sin = Math_SinS(rotation->x);
    cos = Math_CosS(rotation->x);

    temp1 = cmf->xy;
    temp2 = cmf->xz;
    cmf->xy = temp1 * cos + temp2 * sin;
    cmf->xz = temp2 * cos - temp1 * sin;

    temp1 = cmf->yy;
    temp2 = cmf->yz;
    cmf->yy = temp1 * cos + temp2 * sin;
    cmf->yz = temp2 * cos - temp1 * sin;

    temp1 = cmf->zy;
    temp2 = cmf->zz;
    cmf->zy = temp1 * cos + temp2 * sin;
    cmf->zz = temp2 * cos - temp1 * sin;

    temp1 = cmf->wy;
    temp2 = cmf->wz;
    cmf->wy = temp1 * cos + temp2 * sin;
    cmf->wz = temp2 * cos - temp1 * sin;
  }
}

/**
 * Set the current matrix to translate and rotate using YXZ Tait-Bryan angles.
 * This means a (column) vector is first rotated around Z, then around X, then
 * around Y, then translated.
 */
void Matrix_SetTranslateRotateYXZ(f32 translateX, f32 translateY,
                                  f32 translateZ, Vec3s* rot) {
  MtxF* cmf = sCurrentMatrix;
  f32 temp1 = Math_SinS(rot->y);
  f32 temp2 = Math_CosS(rot->y);
  f32 cos;
  f32 sin;

  cmf->xx = temp2;
  cmf->zx = -temp1;
  cmf->xw = translateX;
  cmf->yw = translateY;
  cmf->zw = translateZ;
  cmf->wx = 0.0f;
  cmf->wy = 0.0f;
  cmf->wz = 0.0f;
  cmf->ww = 1.0f;

  if (rot->x != 0) {
    sin = Math_SinS(rot->x);
    cos = Math_CosS(rot->x);

    cmf->zz = temp2 * cos;
    cmf->zy = temp2 * sin;
    cmf->xz = temp1 * cos;
    cmf->xy = temp1 * sin;
    cmf->yz = -sin;
    cmf->yy = cos;
  } else {
    cmf->zz = temp2;
    cmf->xz = temp1;
    cmf->yz = 0.0f;
    cmf->zy = 0.0f;
    cmf->xy = 0.0f;
    cmf->yy = 1.0f;
  }

  if (rot->z != 0) {
    sin = Math_SinS(rot->z);
    cos = Math_CosS(rot->z);

    temp1 = cmf->xx;
    temp2 = cmf->xy;
    cmf->xx = temp1 * cos + temp2 * sin;
    cmf->xy = temp2 * cos - temp1 * sin;

    temp1 = cmf->zx;
    temp2 = cmf->zy;
    cmf->zx = temp1 * cos + temp2 * sin;
    cmf->zy = temp2 * cos - temp1 * sin;

    temp2 = cmf->yy;
    cmf->yx = temp2 * sin;
    cmf->yy = temp2 * cos;
  } else {
    cmf->yx = 0.0f;
  }
}

void Matrix_MultVec3f(Vec3f* src, Vec3f* dest) {
  MtxF* cmf = sCurrentMatrix;

  dest->x = cmf->xw + (cmf->xx * src->x + cmf->xy * src->y + cmf->xz * src->z);
  dest->y = cmf->yw + (cmf->yx * src->x + cmf->yy * src->y + cmf->yz * src->z);
  dest->z = cmf->zw + (cmf->zx * src->x + cmf->zy * src->y + cmf->zz * src->z);
}

void Matrix_MtxFCopy(MtxF* dest, MtxF* src) {
  dest->xx = src->xx;
  dest->yx = src->yx;
  dest->zx = src->zx;
  dest->wx = src->wx;
  dest->xy = src->xy;
  dest->yy = src->yy;
  dest->zy = src->zy;
  dest->wy = src->wy;
  dest->xx = src->xx;
  dest->yx = src->yx;
  dest->zx = src->zx;
  dest->wx = src->wx;
  dest->xy = src->xy;
  dest->yy = src->yy;
  dest->zy = src->zy;
  dest->wy = src->wy;
  dest->xz = src->xz;
  dest->yz = src->yz;
  dest->zz = src->zz;
  dest->wz = src->wz;
  dest->xw = src->xw;
  dest->yw = src->yw;
  dest->zw = src->zw;
  dest->ww = src->ww;
  dest->xz = src->xz;
  dest->yz = src->yz;
  dest->zz = src->zz;
  dest->wz = src->wz;
  dest->xw = src->xw;
  dest->yw = src->yw;
  dest->zw = src->zw;
  dest->ww = src->ww;
}

void Matrix_MultVec3fExt(Vec3f* src, Vec3f* dest, MtxF* mf) {
  dest->x = mf->xw + (mf->xx * src->x + mf->xy * src->y + mf->xz * src->z);
  dest->y = mf->yw + (mf->yx * src->x + mf->yy * src->y + mf->yz * src->z);
  dest->z = mf->zw + (mf->zx * src->x + mf->zy * src->y + mf->zz * src->z);
}

void Matrix_Transpose(MtxF* mf) {
  f32 temp;

  temp = mf->yx;
  mf->yx = mf->xy;
  mf->xy = temp;

  temp = mf->zx;
  mf->zx = mf->xz;
  mf->xz = temp;

  temp = mf->zy;
  mf->zy = mf->yz;
  mf->yz = temp;
}

/**
 * Changes the 3x3 part of the current matrix to `mf` * S, where S is the scale
 * in the current matrix.
 *
 * In details, S is a diagonal where each coefficient is the norm of the column
 * in the 3x3 current matrix. The 3x3 part can then be written as R * S where R
 * has its columns normalized. Since R is typically a rotation matrix, and the
 * 3x3 part is changed from R * S to `mf` * S, this operation can be seen as
 * replacing the R rotation with `mf`, hence the function name.
 */
void Matrix_ReplaceRotation(MtxF* mf) {
  MtxF* cmf = sCurrentMatrix;
  f32 acc;
  f32 temp;
  f32 curColNorm;

  // compute the Euclidean norm of the first column of the current matrix
  acc = cmf->xx;
  acc *= acc;
  temp = cmf->yx;
  acc += SQ(temp);
  temp = cmf->zx;
  acc += SQ(temp);
  curColNorm = sqrtf(acc);

  cmf->xx = mf->xx * curColNorm;
  cmf->yx = mf->yx * curColNorm;
  cmf->zx = mf->zx * curColNorm;

  // second column
  acc = cmf->xy;
  acc *= acc;
  temp = cmf->yy;
  acc += SQ(temp);
  temp = cmf->zy;
  acc += SQ(temp);
  curColNorm = sqrtf(acc);

  cmf->xy = mf->xy * curColNorm;
  cmf->yy = mf->yy * curColNorm;
  cmf->zy = mf->zy * curColNorm;

  // third column
  acc = cmf->xz;
  acc *= acc;
  temp = cmf->yz;
  acc += SQ(temp);
  temp = cmf->zz;
  acc += SQ(temp);
  curColNorm = sqrtf(acc);

  cmf->xz = mf->xz * curColNorm;
  cmf->yz = mf->yz * curColNorm;
  cmf->zz = mf->zz * curColNorm;
}

/**
 * Gets the rotation the specified matrix represents, using Tait-Bryan YXZ
 * angles. The flag value doesn't matter for a rotation matrix. Not 0 does extra
 * calculation.
 */
void Matrix_MtxFToYXZRotS(MtxF* mf, Vec3s* rotDest, s32 flag) {
  f32 temp;
  f32 temp2;
  f32 temp3;
  // f32 temp4;

  temp = mf->xz;
  temp *= temp;
  temp += SQ(mf->zz);
  rotDest->x = RAD_TO_BINANG(Math_FAtan2F(-mf->yz, sqrtf(temp)));

  if ((rotDest->x == 0x4000) || (rotDest->x == -0x4000)) {
    rotDest->z = 0;

    rotDest->y = RAD_TO_BINANG(Math_FAtan2F(-mf->zx, mf->xx));
  } else {
    rotDest->y = RAD_TO_BINANG(Math_FAtan2F(mf->xz, mf->zz));

    if (!flag) {
      rotDest->z = RAD_TO_BINANG(Math_FAtan2F(mf->yx, mf->yy));
    } else {
      temp = mf->xx;
      temp2 = mf->zx;
      temp3 = mf->zy;

      temp *= temp;
      temp += SQ(temp2);
      temp2 = mf->yx;
      temp += SQ(temp2);
      /* temp = xx^2+zx^2+yx^2 == 1 for a rotation matrix */
      temp = sqrtf(temp);
      temp = temp2 / temp;

      temp2 = mf->xy;
      temp2 *= temp2;
      temp2 += SQ(temp3);
      temp3 = mf->yy;
      temp2 += SQ(temp3);
      /* temp2 = xy^2+zy^2+yy^2 == 1 for a rotation matrix */
      temp2 = sqrtf(temp2);
      temp2 = temp3 / temp2;

      /* for a rotation matrix, temp == yx and temp2 == yy
       * which is the same as in the !flag branch */
      rotDest->z = RAD_TO_BINANG(Math_FAtan2F(temp, temp2));
    }
  }
}

/**
 * Gets the rotation the specified matrix represents, using Tait-Bryan ZYX
 * angles. The flag value doesn't matter for a rotation matrix. Not 0 does extra
 * calculation.
 */
void Matrix_MtxFToZYXRotS(MtxF* mf, Vec3s* rotDest, s32 flag) {
  f32 temp;
  f32 temp2;
  f32 temp3;
  // f32 temp4;

  temp = mf->xx;
  temp *= temp;
  temp += SQ(mf->yx);
  rotDest->y = RAD_TO_BINANG(Math_FAtan2F(-mf->zx, sqrtf(temp)));

  if ((rotDest->y == 0x4000) || (rotDest->y == -0x4000)) {
    rotDest->x = 0;
    rotDest->z = RAD_TO_BINANG(Math_FAtan2F(-mf->xy, mf->yy));
  } else {
    rotDest->z = RAD_TO_BINANG(Math_FAtan2F(mf->yx, mf->xx));

    if (!flag) {
      rotDest->x = RAD_TO_BINANG(Math_FAtan2F(mf->zy, mf->zz));
    } else {
      // see Matrix_MtxFToYXZRotS
      temp = mf->xy;
      temp2 = mf->yy;
      temp3 = mf->yz;

      temp *= temp;
      temp += SQ(temp2);
      temp2 = mf->zy;
      temp += SQ(temp2);
      temp = sqrtf(temp);
      temp = temp2 / temp;

      temp2 = mf->xz;
      temp2 *= temp2;
      temp2 += SQ(temp3);
      temp3 = mf->zz;
      temp2 += SQ(temp3);
      temp2 = sqrtf(temp2);
      temp2 = temp3 / temp2;

      rotDest->x = RAD_TO_BINANG(Math_FAtan2F(temp, temp2));
    }
  }
}

/*
 * Rotate the matrix by `angle` radians around a unit vector `axis`.
 * NB: `axis` is assumed to be a unit vector.
 */
void Matrix_RotateAxis(f32 angle, Vec3f* axis, u8 mode) {
  MtxF* cmf;
  f32 sin;
  f32 cos;
  f32 rCos;
  f32 temp1;
  f32 temp2;
  f32 temp3;
  f32 temp4;

  if (mode == MTXMODE_APPLY) {
    if (angle != 0) {
      cmf = sCurrentMatrix;

      sin = sinf(angle);
      cos = cosf(angle);

      temp1 = cmf->xx;
      temp2 = cmf->xy;
      temp3 = cmf->xz;
      temp4 =
          (axis->x * temp1 + axis->y * temp2 + axis->z * temp3) * (1.0f - cos);
      cmf->xx = temp1 * cos + axis->x * temp4 +
                sin * (temp2 * axis->z - temp3 * axis->y);
      cmf->xy = temp2 * cos + axis->y * temp4 +
                sin * (temp3 * axis->x - temp1 * axis->z);
      cmf->xz = temp3 * cos + axis->z * temp4 +
                sin * (temp1 * axis->y - temp2 * axis->x);

      temp1 = cmf->yx;
      temp2 = cmf->yy;
      temp3 = cmf->yz;
      temp4 =
          (axis->x * temp1 + axis->y * temp2 + axis->z * temp3) * (1.0f - cos);
      cmf->yx = temp1 * cos + axis->x * temp4 +
                sin * (temp2 * axis->z - temp3 * axis->y);
      cmf->yy = temp2 * cos + axis->y * temp4 +
                sin * (temp3 * axis->x - temp1 * axis->z);
      cmf->yz = temp3 * cos + axis->z * temp4 +
                sin * (temp1 * axis->y - temp2 * axis->x);

      temp1 = cmf->zx;
      temp2 = cmf->zy;
      temp3 = cmf->zz;
      temp4 =
          (axis->x * temp1 + axis->y * temp2 + axis->z * temp3) * (1.0f - cos);
      cmf->zx = temp1 * cos + axis->x * temp4 +
                sin * (temp2 * axis->z - temp3 * axis->y);
      cmf->zy = temp2 * cos + axis->y * temp4 +
                sin * (temp3 * axis->x - temp1 * axis->z);
      cmf->zz = temp3 * cos + axis->z * temp4 +
                sin * (temp1 * axis->y - temp2 * axis->x);
    }
  } else {
    cmf = sCurrentMatrix;

    if (angle != 0) {
      sin = sinf(angle);
      cos = cosf(angle);
      rCos = 1.0f - cos;

      cmf->xx = axis->x * axis->x * rCos + cos;
      cmf->yy = axis->y * axis->y * rCos + cos;
      cmf->zz = axis->z * axis->z * rCos + cos;

      if (0) {
      }

      temp2 = axis->x * rCos * axis->y;
      temp3 = axis->z * sin;
      cmf->yx = temp2 + temp3;
      cmf->xy = temp2 - temp3;

      temp2 = axis->x * rCos * axis->z;
      temp3 = axis->y * sin;
      cmf->zx = temp2 - temp3;
      cmf->xz = temp2 + temp3;

      temp2 = axis->y * rCos * axis->z;
      temp3 = axis->x * sin;
      cmf->zy = temp2 + temp3;
      cmf->yz = temp2 - temp3;

      cmf->wx = cmf->wy = cmf->wz = cmf->xw = cmf->yw = cmf->zw = 0.0f;
      cmf->ww = 1.0f;
    } else {
      cmf->yx = 0.0f;
      cmf->zx = 0.0f;
      cmf->wx = 0.0f;
      cmf->xy = 0.0f;
      cmf->zy = 0.0f;
      cmf->wy = 0.0f;
      cmf->xz = 0.0f;
      cmf->yz = 0.0f;
      cmf->wz = 0.0f;
      cmf->xw = 0.0f;
      cmf->yw = 0.0f;
      cmf->zw = 0.0f;
      cmf->xx = 1.0f;
      cmf->yy = 1.0f;
      cmf->zz = 1.0f;
      cmf->ww = 1.0f;
    }
  }
}

void Matrix_SetTranslateUniformScaleMtxF(MtxF* mf, f32 scale, f32 translateX,
                                         f32 translateY, f32 translateZ) {
  mf->yx = 0.0f;
  mf->zx = 0.0f;
  mf->wx = 0.0f;
  mf->xy = 0.0f;
  mf->zy = 0.0f;
  mf->wy = 0.0f;
  mf->xz = 0.0f;
  mf->yz = 0.0f;
  mf->wz = 0.0f;
  mf->xx = scale;
  mf->yy = scale;
  mf->zz = scale;
  mf->xw = translateX;
  mf->yw = translateY;
  mf->zw = translateZ;
  mf->ww = 1.0f;
}