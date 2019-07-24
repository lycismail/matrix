#include "YUVColorConvert.h"

/* yuv -> rgb def's */

#define RGB_Y_OUT		1
#define B_U_OUT			1.773
#define Y_ADD_OUT		0

#define G_U_OUT			0.714
#define G_V_OUT			0.344
#define U_ADD_OUT		128

#define R_V_OUT			1.403
#define V_ADD_OUT		128

#define SCALEBITS_OUT	13
#define FIX_OUT(x)		((short) ((x) * (1L<<SCALEBITS_OUT) + 0.5))

#define fast_clamp(a ,lo,hi) (a + ((lo-a) & (lo-a < 0) - 1) + (hi-a & (hi-a > 0) - 1))

YUVColorConvert YUVColorConvert::converter;

bool YUVColorConvert::bTableInited = false;
int YUVColorConvert::RGB_Y_tab[256] = { 0 };
int YUVColorConvert::B_U_tab[256] = { 0 };
int YUVColorConvert::G_U_tab[256] = { 0 };
int YUVColorConvert::G_V_tab[256] = { 0 };
int YUVColorConvert::R_V_tab[256] = { 0 };

float YUVColorConvert::RGB_Y_tab2[256] = { 0 };
float YUVColorConvert::R_Cr_tab[256] = { 0 };
float YUVColorConvert::G_Cr_tab[256] = { 0 };
float YUVColorConvert::G_Cb_tab[256] = { 0 };
float YUVColorConvert::B_Cb_tab[256] = { 0 };

void YUVColorConvert::_colorspace_init(void)
{
  if (bTableInited)
    return;

  int i;

  for (i = 0; i < 256; i++) {

#if RGB_RANGE  // if rgb is in (16,235)
    R_Cr_tab[i] = 1.371 * i;
    G_Cr_tab[i] = -0.698 * i;
    G_Cb_tab[i] = -0.336 * i;
    B_Cb_tab[i] = 1.732 * i;
#else // if rgb is in (0, 255)
    RGB_Y_tab2[i] = 1.164*(i > 16 ? i - 16 : 0);
    R_Cr_tab[i] = 1.596 * i;
    G_Cr_tab[i] = -0.813 * i;
    G_Cb_tab[i] = -0.392 * i;
    B_Cb_tab[i] = 2.017 * i;
#endif 
  }
#if 1
  for (i = 0; i < 256; i++) {
    RGB_Y_tab[i] = FIX_OUT(RGB_Y_OUT) * (i - Y_ADD_OUT);
    B_U_tab[i] = FIX_OUT(B_U_OUT) * (i - U_ADD_OUT);
    G_U_tab[i] = FIX_OUT(G_U_OUT) * (i - U_ADD_OUT);
    G_V_tab[i] = FIX_OUT(G_V_OUT) * (i - V_ADD_OUT);
    R_V_tab[i] = FIX_OUT(R_V_OUT) * (i - V_ADD_OUT);
  }
#endif

  bTableInited = true;
}

YUVColorConvert::YUVColorConvert(void)
{
  this->_colorspace_init();
}

YUVColorConvert::~YUVColorConvert(void)
{
}

void YUVColorConvert::YUV420sp_2_RGB(const unsigned char*  pYUVBuffer,
                                     const int nSrcWidth, const int nSrcHeight, const int nSrcStep,
                                     unsigned char* pRGBBuffer, const int nDestStep,
                                     bool uFirst/* = true*/) const
{
  int x, y, rTemp, gTemp, bTemp;
  unsigned char* pR, *pG, *pB;
  unsigned char Y, U, V, Y2;
  const unsigned char* pYBuffer = pYUVBuffer;
  const unsigned char* pUVBuffer = pYUVBuffer + nSrcHeight*nSrcWidth;
  if (uFirst) {

    for (y = 0; y < nSrcHeight; y++) {
      pB = pRGBBuffer;
      pG = pB + 1;
      pR = pG + 1;
      for (x = 0; x < nSrcWidth / 2; x++) {
        Y = *pYBuffer++;
        Y2 = *pYBuffer++;
        U = *pUVBuffer++;
        V = *pUVBuffer++;
#if RGB_RANGE //rgb is in (16,235)
        rTemp = Y + R_Cr_tab[V] + R_diff_value;
        gTemp = Y + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
        bTemp = Y + B_Cb_tab[U] + B_diff_value;
#else // rgb is in (0, 255)
        rTemp = RGB_Y_tab2[Y] + R_Cr_tab[V] + R_diff_value;
        gTemp = RGB_Y_tab2[Y] + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
        bTemp = RGB_Y_tab2[Y] + B_Cb_tab[U] + B_diff_value;
#endif
#if 0
        rTemp = (RGB_Y_tab[Y] + R_V_tab[V]) >> SCALEBITS_OUT;
        gTemp = (RGB_Y_tab[Y] - G_U_tab[U] - G_V_tab[V]) >> SCALEBITS_OUT;
        bTemp = (RGB_Y_tab[Y] + B_U_tab[U]) >> SCALEBITS_OUT;
#endif
        // 			*pR = (unsigned char)fast_clamp( rTemp, 0, 255 );
        // 			*pG = (unsigned char)fast_clamp( gTemp, 0, 255 );
        // 			*pB = (unsigned char)fast_clamp( bTemp, 0, 255 );

#if RGB_RANGE //rgb is in (16,235)
        * pR = (unsigned char)fast_clamp((rTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
        *pG = (unsigned char)fast_clamp((gTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
        *pB = (unsigned char)fast_clamp((bTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
#else // rgb is in (0, 255)
        *pR = (unsigned char)fast_clamp(rTemp, 0, 255);
        *pG = (unsigned char)fast_clamp(gTemp, 0, 255);
        *pB = (unsigned char)fast_clamp(bTemp, 0, 255);
#endif

        pR += 3;
        pG += 3;
        pB += 3;

        // the second 
#if RGB_RANGE //rgb is in (16,235)
        rTemp = Y2 + R_Cr_tab[V] + R_diff_value;
        gTemp = Y2 + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
        bTemp = Y2 + B_Cb_tab[U] + B_diff_value;
#else // rgb is in (0, 255)
        rTemp = RGB_Y_tab2[Y2] + R_Cr_tab[V] + R_diff_value;
        gTemp = RGB_Y_tab2[Y2] + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
        bTemp = RGB_Y_tab2[Y2] + B_Cb_tab[U] + B_diff_value;
#endif
#if 0
        rTemp = (RGB_Y_tab[Y2] + R_V_tab[V]) >> SCALEBITS_OUT;
        gTemp = (RGB_Y_tab[Y2] - G_U_tab[U] - G_V_tab[V]) >> SCALEBITS_OUT;
        bTemp = (RGB_Y_tab[Y2] + B_U_tab[U]) >> SCALEBITS_OUT;
#endif
        // 			*pR = (unsigned char)fast_clamp( rTemp, 0, 255 );
        // 			*pG = (unsigned char)fast_clamp( gTemp, 0, 255 );
        // 			*pB = (unsigned char)fast_clamp( bTemp, 0, 255 );
#if RGB_RANGE //rgb is in (16,235)
        *pR = (unsigned char)fast_clamp((rTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
        *pG = (unsigned char)fast_clamp((gTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
        *pB = (unsigned char)fast_clamp((bTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
#else // rgb is in (0, 255)
        *pR = (unsigned char)fast_clamp(rTemp, 0, 255);
        *pG = (unsigned char)fast_clamp(gTemp, 0, 255);
        *pB = (unsigned char)fast_clamp(bTemp, 0, 255);
#endif
        pR += 3;
        pG += 3;
        pB += 3;
      }

      pRGBBuffer += nDestStep;
      // AS Y-Direction is half sample so adjust pUBuffer and pYBuffer
      pUVBuffer -= (y % 2 == 0 ? nSrcWidth : 0);
    }
  } else {
    // v first
    for (y = 0; y < nSrcHeight; y++) {
      pB = pRGBBuffer;
      pG = pB + 1;
      pR = pG + 1;
      for (x = 0; x < nSrcWidth / 2; x++) {
        Y = *pYBuffer++;
        Y2 = *pYBuffer++;
        V = *pUVBuffer++;
        U = *pUVBuffer++;
#if RGB_RANGE //rgb is in (16,235)
        rTemp = Y + R_Cr_tab[V] + R_diff_value;
        gTemp = Y + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
        bTemp = Y + B_Cb_tab[U] + B_diff_value;
#else // rgb is in (0, 255)
        rTemp = RGB_Y_tab2[Y] + R_Cr_tab[V] + R_diff_value;
        gTemp = RGB_Y_tab2[Y] + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
        bTemp = RGB_Y_tab2[Y] + B_Cb_tab[U] + B_diff_value;
#endif
#if 0
        rTemp = (RGB_Y_tab[Y] + R_V_tab[V]) >> SCALEBITS_OUT;
        gTemp = (RGB_Y_tab[Y] - G_U_tab[U] - G_V_tab[V]) >> SCALEBITS_OUT;
        bTemp = (RGB_Y_tab[Y] + B_U_tab[U]) >> SCALEBITS_OUT;
#endif
        // 			*pR = (unsigned char)fast_clamp( rTemp, 0, 255 );
        // 			*pG = (unsigned char)fast_clamp( gTemp, 0, 255 );
        // 			*pB = (unsigned char)fast_clamp( bTemp, 0, 255 );

#if RGB_RANGE //rgb is in (16,235)
        * pR = (unsigned char)fast_clamp((rTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
        *pG = (unsigned char)fast_clamp((gTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
        *pB = (unsigned char)fast_clamp((bTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
#else // rgb is in (0, 255)
        *pR = (unsigned char)fast_clamp(rTemp, 0, 255);
        *pG = (unsigned char)fast_clamp(gTemp, 0, 255);
        *pB = (unsigned char)fast_clamp(bTemp, 0, 255);
#endif

        pR += 3;
        pG += 3;
        pB += 3;

        // the second 
#if RGB_RANGE //rgb is in (16,235)
        rTemp = Y2 + R_Cr_tab[V] + R_diff_value;
        gTemp = Y2 + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
        bTemp = Y2 + B_Cb_tab[U] + B_diff_value;
#else // rgb is in (0, 255)
        rTemp = RGB_Y_tab2[Y2] + R_Cr_tab[V] + R_diff_value;
        gTemp = RGB_Y_tab2[Y2] + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
        bTemp = RGB_Y_tab2[Y2] + B_Cb_tab[U] + B_diff_value;
#endif
#if 0
        rTemp = (RGB_Y_tab[Y2] + R_V_tab[V]) >> SCALEBITS_OUT;
        gTemp = (RGB_Y_tab[Y2] - G_U_tab[U] - G_V_tab[V]) >> SCALEBITS_OUT;
        bTemp = (RGB_Y_tab[Y2] + B_U_tab[U]) >> SCALEBITS_OUT;
#endif
        // 			*pR = (unsigned char)fast_clamp( rTemp, 0, 255 );
        // 			*pG = (unsigned char)fast_clamp( gTemp, 0, 255 );
        // 			*pB = (unsigned char)fast_clamp( bTemp, 0, 255 );
#if RGB_RANGE //rgb is in (16,235)
        *pR = (unsigned char)fast_clamp((rTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
        *pG = (unsigned char)fast_clamp((gTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
        *pB = (unsigned char)fast_clamp((bTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
#else // rgb is in (0, 255)
        *pR = (unsigned char)fast_clamp(rTemp, 0, 255);
        *pG = (unsigned char)fast_clamp(gTemp, 0, 255);
        *pB = (unsigned char)fast_clamp(bTemp, 0, 255);
#endif
        pR += 3;
        pG += 3;
        pB += 3;
      }

      pRGBBuffer += nDestStep;
      // AS Y-Direction is half sample so adjust pUBuffer and pYBuffer
      pUVBuffer -= (y % 2 == 0 ? nSrcWidth : 0);
    }
  }
}


void YUVColorConvert::YUV422I_2_RGB(const unsigned char*  pYUVBuffer, const int nSrcWidth, const int nSrcHeight, const int nSrcStep,
                                    unsigned char* pRGBBuffer, const int nDestStep) const {
  int x, y, rTemp, gTemp, bTemp;
  unsigned char* pR, *pG, *pB;
  unsigned char Y, U, V, Y2;
  const unsigned char* pYBuffer = pYUVBuffer;
  const unsigned char* pUBuffer = pYUVBuffer + nSrcHeight*nSrcWidth;
  const unsigned char* pVBuffer = pYUVBuffer + ((nSrcHeight*nSrcWidth * 5) >> 2);
  for (y = 0; y < nSrcHeight; y++) {
    pB = pRGBBuffer;
    pG = pB + 1;
    pR = pG + 1;
    for (x = 0; x < nSrcWidth / 2; x++) {
      Y = *pYBuffer++;
      Y2 = *pYBuffer++;
      U = *pUBuffer++;
      V = *pVBuffer++;
#if RGB_RANGE //rgb is in (16,235)
      rTemp = Y + R_Cr_tab[V] + R_diff_value;
      gTemp = Y + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
      bTemp = Y + B_Cb_tab[U] + B_diff_value;
#else // rgb is in (0, 255)
      rTemp = RGB_Y_tab2[Y] + R_Cr_tab[V] + R_diff_value;
      gTemp = RGB_Y_tab2[Y] + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
      bTemp = RGB_Y_tab2[Y] + B_Cb_tab[U] + B_diff_value;
#endif
#if 0
      rTemp = (RGB_Y_tab[Y] + R_V_tab[V]) >> SCALEBITS_OUT;
      gTemp = (RGB_Y_tab[Y] - G_U_tab[U] - G_V_tab[V]) >> SCALEBITS_OUT;
      bTemp = (RGB_Y_tab[Y] + B_U_tab[U]) >> SCALEBITS_OUT;
#endif
      // 			*pR = (unsigned char)fast_clamp( rTemp, 0, 255 );
      // 			*pG = (unsigned char)fast_clamp( gTemp, 0, 255 );
      // 			*pB = (unsigned char)fast_clamp( bTemp, 0, 255 );

#if RGB_RANGE //rgb is in (16,235)
      * pR = (unsigned char)fast_clamp((rTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
      *pG = (unsigned char)fast_clamp((gTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
      *pB = (unsigned char)fast_clamp((bTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
#else // rgb is in (0, 255)
      *pR = (unsigned char)fast_clamp(rTemp, 0, 255);
      *pG = (unsigned char)fast_clamp(gTemp, 0, 255);
      *pB = (unsigned char)fast_clamp(bTemp, 0, 255);
#endif

      pR += 3;
      pG += 3;
      pB += 3;

      // the second 
#if RGB_RANGE //rgb is in (16,235)
      rTemp = Y2 + R_Cr_tab[V] + R_diff_value;
      gTemp = Y2 + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
      bTemp = Y2 + B_Cb_tab[U] + B_diff_value;
#else // rgb is in (0, 255)
      rTemp = RGB_Y_tab2[Y2] + R_Cr_tab[V] + R_diff_value;
      gTemp = RGB_Y_tab2[Y2] + G_Cr_tab[V] + G_Cb_tab[U] + G_diff_value;
      bTemp = RGB_Y_tab2[Y2] + B_Cb_tab[U] + B_diff_value;
#endif
#if 0
      rTemp = (RGB_Y_tab[Y2] + R_V_tab[V]) >> SCALEBITS_OUT;
      gTemp = (RGB_Y_tab[Y2] - G_U_tab[U] - G_V_tab[V]) >> SCALEBITS_OUT;
      bTemp = (RGB_Y_tab[Y2] + B_U_tab[U]) >> SCALEBITS_OUT;
#endif
      // 			*pR = (unsigned char)fast_clamp( rTemp, 0, 255 );
      // 			*pG = (unsigned char)fast_clamp( gTemp, 0, 255 );
      // 			*pB = (unsigned char)fast_clamp( bTemp, 0, 255 );
#if RGB_RANGE //rgb is in (16,235)
      *pR = (unsigned char)fast_clamp((rTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
      *pG = (unsigned char)fast_clamp((gTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
      *pB = (unsigned char)fast_clamp((bTemp - 16) * 255 / (235 - 16 + 1), 0, 255);
#else // rgb is in (0, 255)
      *pR = (unsigned char)fast_clamp(rTemp, 0, 255);
      *pG = (unsigned char)fast_clamp(gTemp, 0, 255);
      *pB = (unsigned char)fast_clamp(bTemp, 0, 255);
#endif
      pR += 3;
      pG += 3;
      pB += 3;
    }

    pRGBBuffer += nDestStep;
    // AS Y-Direction is half sample so adjust pUBuffer and pYBuffer
    pUBuffer -= (y % 2 == 0 ? nSrcWidth / 2 : 0);
    pVBuffer -= (y % 2 == 0 ? nSrcWidth / 2 : 0);
  }
}
