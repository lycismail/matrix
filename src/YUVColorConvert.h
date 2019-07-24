#ifndef __YUV_COLOR_CONVERT_H__
#define __YUV_COLOR_CONVERT_H__

#if defined(WIN32) || defined(__arm__)
#define
#endif

#define RGB_RANGE 1  // 1 : rgb from 16 to 235 
// 0 : rgb from 0 to 255

class YUVColorConvert
{
public:
  ~YUVColorConvert(void);

  static YUVColorConvert *GetInstance()
  {
    //static YUVColorConvert converter;
    return &YUVColorConvert::converter;
  }

  //没有下采样
  void YUV422I_2_RGB(
    const unsigned char* pYUVBuffer, const int nSrcWidth,
    const int nSrcHeight, const int nSrcStep,
    unsigned char* pRGBBuffer, const int nDestStep
  ) const;
  
  void YUV420sp_2_RGB(
    const unsigned char* pYUVBuffer, const int nSrcWidth,
    const int nSrcHeight, const int nSrcStep,
    unsigned char* pRGBBuffer, const int nDestStep,
    bool uFirst = true
    ) const;
private:
  YUVColorConvert(void);

  static YUVColorConvert converter;

  //YUV to RGB lookup table
  static bool bTableInited;
  static int RGB_Y_tab[256];
  static int B_U_tab[256];
  static int G_U_tab[256];
  static int G_V_tab[256];
  static int R_V_tab[256];

  static float R_Cr_tab[256];
  static float G_Cr_tab[256];
  static float G_Cb_tab[256];
  static float B_Cb_tab[256];
  static float RGB_Y_tab2[256];
#if RGB_RANGE // if rgb is in (16,235)
  const float R_diff_value = -1.371 * 128;
  const float G_diff_value = (0.698 + 0.336) * 128;
  const float B_diff_value = -1.732 * 128;
#else // if rgb is in (0, 255)
  const float R_diff_value = -1.596 * 128;
  const float G_diff_value = (0.813 + 0.392) * 128;
  const float B_diff_value = -2.017 * 128;
#endif
  void _colorspace_init();
};

#endif
