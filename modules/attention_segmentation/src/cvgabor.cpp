/***************************************************************************
 *   Copyright (C) 2006 by Mian Zhou   *
 *   M.Zhou@reading.ac.uk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <glog/logging.h>

#include "v4r/attention_segmentation/cvgabor.h"

namespace v4r {

CvGabor::CvGabor()  // constructor
{}

CvGabor::~CvGabor()  // Deconstructor
{
  cvReleaseMat(&Real);
  cvReleaseMat(&Imag);
}

/**
 * @brief Construct a gabor
 * Create a gabor with a orientation iMu*PI/8, a scale iNu, and a sigma value dSigma.
 * The spatial frequence (F) is set to sqrt(2) defaultly. It calls Init() to generate
 * parameters and kernels.
 * @param iMu The orientation iMu*PI/8,
 * @param iNu The scale
 * @param dSigma The sigma value of Gabor,
 */
CvGabor::CvGabor(int iMu, int iNu, double dSigma) {
  F = sqrt(2.0);
  Init(iMu, iNu, dSigma, F);
}

/**
 * @brief Construct a gabor
 * Create a gabor with a orientation iMu*PI/8, a scale iNu, a sigma value dSigma,
 * and a spatial frequence dF. It calls Init() to generate parameters and kernels.
 * @param iMu The orientation iMu*PI/8,
 * @param iNu The scale
 * @param dSigma The sigma value of Gabor,
 * @param dF The spatial frequency
 */
CvGabor::CvGabor(int iMu, int iNu, double dSigma, double dF) {
  Init(iMu, iNu, dSigma, dF);
}

/**
 * @brief Construct a gabor
 * Create a gabor with a orientation dPhi, and with a scale iNu. The sigma (Sigma)
 * and the spatial frequence (F) are set to 2*PI and sqrt(2) defaultly. It calls
 * Init() to generate parameters and kernels.
 * @param dPhi The orientation in arc
 * @param iNu The scale
 */
CvGabor::CvGabor(double dPhi, int iNu) {
  Sigma = 2 * PI;
  F = sqrt(2.0);
  Init(dPhi, iNu, Sigma, F);
}

/**
 * @brief Construct a gabor
 * Create a gabor with a orientation dPhi, a scale iNu, and a sigma value dSigma.
 * The spatial frequence (F) is set to sqrt(2) defaultly. It calls Init() to
 * generate parameters and kernels.
 * @param dPhi The orientation in arc
 * @param iNu The scale
 * @param dSigma The sigma value of Gabor
 */
CvGabor::CvGabor(double dPhi, int iNu, double dSigma) {
  F = sqrt(2.0);
  Init(dPhi, iNu, dSigma, F);
}

/**
 * @brief Construct a gabor
 * Create a gabor with a orientation dPhi, a scale iNu, a sigma value dSigma,
 * and a spatial frequence dF. It calls Init() to generate parameters and kernels.
 * @param dPhi The orientation in arc
 * @param iNu The scale
 * @param dSigma The sigma value of Gabor
 * @param dF The spatial frequency
 */
CvGabor::CvGabor(double dPhi, int iNu, double dSigma, double dF) {
  Init(dPhi, iNu, dSigma, dF);
}

/**
 * @brief Determine whether the gabor has been initlized.
 * Variables F, K, Kmax, Phi, Sigma are filled.
 * @return A boolean value, TRUE is initialized or FALSE is non-initialized.
 */
bool CvGabor::IsInit() {
  return bInitialised;
}

/**
 * @brief Return the width of mask (should be NxN) by the value of Sigma and iNu.
 * @return The long type show the width.
 */
long CvGabor::mask_width() {
  long lWidth;
  if (IsInit() == false) {
    LOG(ERROR) << "The Object has not been initialized in mask_width()!";
    return 0;
  } else {
    // determine the width of Mask
    double dModSigma = Sigma / K;
    double dWidth = (int)(dModSigma * 6 + 1);
    // test whether dWidth is an odd.
    if (fmod(dWidth, 2.0) == 0.0)
      dWidth++;
    lWidth = (long)dWidth;
    VLOG(2) << "Gabor mask with: " << lWidth;
    return lWidth;
  }
}

/**
 * @brief Create 2 gabor kernels - REAL and IMAG, with an orientation and a scale
 */
void CvGabor::creat_kernel()  //创建gabor核
{
  if (IsInit() == false) {
    LOG(ERROR) << "The Object has not been initialized in creat_kernel()!";
  } else {
    CvMat *mReal, *mImag;
    mReal = cvCreateMat(Width, Width, CV_32FC1);  //实部窗口框的大小
    mImag = cvCreateMat(Width, Width, CV_32FC1);  //虚部窗口框的大小

    /**************************** Gabor Function ****************************/
    int x, y;
    double dReal;
    double dImag;
    double dTemp1, dTemp2, dTemp3;

    for (int i = 0; i < Width; i++) {
      for (int j = 0; j < Width; j++) {
        x = i - (Width - 1) / 2;
        y = j - (Width - 1) / 2;
        dTemp1 = (pow(K, 2) / pow(Sigma, 2)) *
                 exp(-(pow((double)x, 2) + pow((double)y, 2)) * pow(K, 2) / (2 * pow(Sigma, 2)));  //高斯窗口函数
        dTemp2 = cos(K * cos(Phi) * x + K * sin(Phi) * y) - exp(-(pow(Sigma, 2) / 2));             //实部，去噪
        dTemp3 = sin(K * cos(Phi) * x + K * sin(Phi) * y);                                         //虚部
        dReal = dTemp1 * dTemp2;                                                                   //求得的实部
        dImag = dTemp1 * dTemp3;                                                                   //求得的虚部
        // gan_mat_set_el(pmReal, i, j, dReal);
        // cvmSet( (CvMat*)mReal, i, j, dReal );
        cvSetReal2D((CvMat *)mReal, i, j, dReal);  //存储求得的结果
                                                   // gan_mat_set_el(pmImag, i, j, dImag);
        // cvmSet( (CvMat*)mImag, i, j, dImag );
        cvSetReal2D((CvMat *)mImag, i, j, dImag);
      }
    }
    /**************************** Gabor Function ****************************/
    bKernel = true;
    cvCopy(mReal, Real, NULL);  //拷贝求得的结果
    cvCopy(mImag, Imag, NULL);
    cvReleaseMat(&mReal);
    cvReleaseMat(&mImag);
  }
}

/**
 * @brief Return an Image (gandalf image class) with a specific Type
 * @param Type The Type of gabor kernel, e.g. REAL, IMAG, MAG, PHASE
 * @return Pointer to image structure, or NULL on failure
 */
IplImage *CvGabor::get_image(int Type) {
  if (IsKernelCreate() == false) {
    LOG(ERROR) << "The Gabor kernel has not been created in get_image()!";
    return nullptr;
  } else {
    IplImage *pImage;
    IplImage *newimage;
    newimage = cvCreateImage(cvSize(Width, Width), IPL_DEPTH_8U, 1);
    pImage = cvCreateImage(cvSize(Width, Width), IPL_DEPTH_32F, 1);

    CvMat *kernel = cvCreateMat(Width, Width, CV_32FC1);
    double ve;
    CvSize size = cvGetSize(kernel);
    int rows = size.height;
    int cols = size.width;
    switch (Type) {
      case 1:  // Real
        cvCopy((CvMat *)Real, (CvMat *)kernel, nullptr);
        for (int i = 0; i < rows; i++) {
          for (int j = 0; j < cols; j++) {
            ve = cvGetReal2D((CvMat *)kernel, i, j);
            cvSetReal2D((IplImage *)pImage, j, i, ve);
          }
        }
        break;
      case 2:  // Imag
        cvCopy((CvMat *)Imag, (CvMat *)kernel, nullptr);
        for (int i = 0; i < rows; i++) {
          for (int j = 0; j < cols; j++) {
            ve = cvGetReal2D((CvMat *)kernel, i, j);
            cvSetReal2D((IplImage *)pImage, j, i, ve);
          }
        }
        break;
      case 3:  // Magnitude
        ///@todo
        LOG(WARNING) << "No magnitude available.";
        break;
      case 4:  // Phase
        ///@todo
        LOG(WARNING) << "No phase available.";
        break;
    }

    cvNormalize((IplImage *)pImage, (IplImage *)pImage, 0, 255, CV_MINMAX, nullptr);
    cvConvertScaleAbs((IplImage *)pImage, (IplImage *)newimage, 1, 0);

    cvReleaseMat(&kernel);
    cvReleaseImage(&pImage);
    return newimage;
  }
}

/**
 * @brief Determine the gabor kernel is created or not
 * @return A boolean value, TRUE is created or FALSE is non-created.
 */
bool CvGabor::IsKernelCreate() {
  return bKernel;
}

/**
 * @brief Reads the width of Mask
 * @return Pointer to long type width of mask.
 */
long CvGabor::get_mask_width() {
  return Width;
}

/**
 * @brief Initialize the.gabor with the orientation iMu, the scale iNu,
 * the sigma dSigma, the frequency dF, it will call the function
 * creat_kernel(); So a gabor is created.
 * @param iMu   The orientations which is iMu*PI.8
 * @param iNu   The scale can be from -5 to infinite
 * @param dSigma  The Sigma value of gabor, Normally set to 2*PI
 * @param dF  The spatial frequence , normally is sqrt(2)
 */
void CvGabor::Init(int iMu, int iNu, double dSigma, double dF) {
  // Initialize the parameters
  bInitialised = false;
  bKernel = false;

  Sigma = dSigma;
  F = dF;

  Kmax = PI / 2;

  // Absolute value of K
  K = Kmax / pow(F, (double)iNu);
  Phi = PI * iMu / 8;
  bInitialised = true;
  Width = mask_width();
  Real = cvCreateMat(Width, Width, CV_32FC1);
  Imag = cvCreateMat(Width, Width, CV_32FC1);
  creat_kernel();
}

/**
 * @brief Initialize the.gabor with the orientation dPhi, the scale iNu, the sigma dSigma,
 * the frequency dF, it will call the function creat_kernel(); So a gabor is created.filename
 * The name of the image file
      file_format   The format of the file, e.g. GAN_PNG_FORMAT
      image   The image structure to be written to the file
      octrlstr  Format-dependent control structure
 * @param dPhi  The orientations
 * @param iNu   The scale can be from -5 to infinite
 * @param dSigma  The Sigma value of gabor, Normally set to 2*PI
 * @param dF  The spatial frequence , normally is sqrt(2)
 */
void CvGabor::Init(double dPhi, int iNu, double dSigma, double dF) {
  bInitialised = false;
  bKernel = false;
  Sigma = dSigma;
  F = dF;

  Kmax = PI / 2;

  // Absolute value of K
  K = Kmax / pow(F, (double)iNu);
  Phi = dPhi;
  bInitialised = true;
  Width = mask_width();
  Real = cvCreateMat(Width, Width, CV_32FC1);
  Imag = cvCreateMat(Width, Width, CV_32FC1);
  creat_kernel();
}

/**
 * @brief Return the gabor kernel.
 * @param Type    The type of kernel, e.g. REAL, IMAG, MAG, PHASE
 * @return Pointer to matrix structure, or NULL on failure.
 */
CvMat *CvGabor::get_matrix(int Type) {
  if (!IsKernelCreate()) {
    LOG(ERROR) << "The gabor kernel has not been created!";
    return nullptr;
  }
  switch (Type) {
    case CV_GABOR_REAL:
      return Real;
      break;
    case CV_GABOR_IMAG:
      return Imag;
      break;
    case CV_GABOR_MAG:
      LOG(ERROR) << "No gabor magnitude available.";
      return nullptr;
      break;
    case CV_GABOR_PHASE:
      LOG(ERROR) << "No gabor phase available.";
      return nullptr;
      break;
  }
  return nullptr;
}

/**
 * @brief Writes an image from the provided image structure into the
 * given file and the type of gabor kernel.
 * @param filename  The name of the image file
 * @param file_format   The format of the file, e.g. GAN_PNG_FORMAT
 * @param Type    The Type of gabor kernel, e.g. REAL, IMAG, MAG, PHASE
 * @return Pointer to matrix structure, or NULL on failure.
 */
void CvGabor::output_file(const char *filename, int Type) {
  IplImage *pImage;
  pImage = get_image(Type);
  if (pImage != nullptr) {
    if (cvSaveImage(filename, pImage))
      VLOG(1) << filename << " has been written successfully!";
    else
      LOG(ERROR) << "writing " << filename << " has failed!";
  } else
    LOG(ERROR) << "The image is empty in output_file()!";

  cvReleaseImage(&pImage);
}

/**
 * @brief CvGabor::show(int Type)
 */
void CvGabor::show(int Type) {
  if (!IsInit()) {
    LOG(ERROR) << "The gabor kernel has not been created!";
  } else {
    IplImage *pImage;
    pImage = get_image(Type);
    cvNamedWindow("Testing", 1);
    cvShowImage("Testing", pImage);
    cvWaitKey(0);
    cvDestroyWindow("Testing");
    cvReleaseImage(&pImage);
  }
}

/**
 * @brief CvGabor::conv_img_a(IplImage *src, IplImage *dst, int Type)
 */
void CvGabor::conv_img_a(IplImage *src, IplImage *dst, int Type)  //图像做gabor卷积  函数名：conv_img_a
{
  double ve, re, im;

  int width = src->width;
  int height = src->height;
  CvMat *mat = cvCreateMat(src->width, src->height, CV_32FC1);

  for (int i = 0; i < width; i++)  //对整幅图像进行图像坐标转换
  {
    for (int j = 0; j < height; j++) {
      ve = cvGetReal2D((IplImage *)src, j, i);
      cvSetReal2D((CvMat *)mat, i, j, ve);
    }
  }

  CvMat *rmat = cvCreateMat(width, height, CV_32FC1);  //存实部
  CvMat *imat = cvCreateMat(width, height, CV_32FC1);  //存虚部

  CvMat *kernel = cvCreateMat(Width, Width, CV_32FC1);  //创建核函数窗口

  switch (Type) {
    case CV_GABOR_REAL:  //实部卷积
      cvCopy((CvMat *)Real, (CvMat *)kernel, nullptr);
      cvFilter2D((CvMat *)mat, (CvMat *)mat, (CvMat *)kernel, cvPoint((Width - 1) / 2, (Width - 1) / 2));
      break;
    case CV_GABOR_IMAG:  //虚部卷积
      cvCopy((CvMat *)Imag, (CvMat *)kernel, nullptr);
      cvFilter2D((CvMat *)mat, (CvMat *)mat, (CvMat *)kernel, cvPoint((Width - 1) / 2, (Width - 1) / 2));
      break;
    case CV_GABOR_MAG:  //实部与虚部卷积
      /* Real Response */
      cvCopy((CvMat *)Real, (CvMat *)kernel, nullptr);
      cvFilter2D((CvMat *)mat, (CvMat *)rmat, (CvMat *)kernel, cvPoint((Width - 1) / 2, (Width - 1) / 2));
      /* Imag Response */
      cvCopy((CvMat *)Imag, (CvMat *)kernel, nullptr);
      cvFilter2D((CvMat *)mat, (CvMat *)imat, (CvMat *)kernel, cvPoint((Width - 1) / 2, (Width - 1) / 2));
      /* Magnitude response is the square root of the sum of the square of real response and imaginary response */
      for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
          re = cvGetReal2D((CvMat *)rmat, i, j);
          im = cvGetReal2D((CvMat *)imat, i, j);
          ve = sqrt(re * re + im * im);
          cvSetReal2D((CvMat *)mat, i, j, ve);
        }
      }
      break;
    case CV_GABOR_PHASE:
      break;
  }

  if (dst->depth == IPL_DEPTH_8U)  //归一化
  {
    cvNormalize((CvMat *)mat, (CvMat *)mat, 0, 255, CV_MINMAX, nullptr);
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
        ve = cvGetReal2D((CvMat *)mat, i, j);
        ve = cvRound(ve);
        cvSetReal2D((IplImage *)dst, j, i, ve);
      }
    }
  }

  if (dst->depth == IPL_DEPTH_32F) {
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
        ve = cvGetReal2D((CvMat *)mat, i, j);
        cvSetReal2D((IplImage *)dst, j, i, ve);
      }
    }
  }

  cvReleaseMat(&kernel);
  cvReleaseMat(&imat);
  cvReleaseMat(&rmat);
  cvReleaseMat(&mat);
}

/**
 * @brief CvGabor::CvGabor(int iMu, int iNu)
 */
CvGabor::CvGabor(int iMu, int iNu) {
  double dSigma = 2 * PI;
  F = sqrt(2.0);
  Init(iMu, iNu, dSigma, F);
}

/**
 * @brief CvGabor::normalize( const CvArr* src, CvArr* dst, double a, double b, int norm_type, const CvArr* mask )
 * @param src
 * @param dst
 * @param a
 * @param b
 * @param norm_type
 * @param mask
 */
void CvGabor::normalize(const CvArr *src, CvArr *dst, double a, double b, int norm_type, const CvArr *mask) {
  CvMat *tmp = 0;
  //     __BEGIN__;

  double scale = 1;
  double shift = 0;
  if (norm_type == CV_MINMAX) {
    double smin = 0, smax = 0;
    double dmin = MIN(a, b), dmax = MAX(a, b);
    cvMinMaxLoc(src, &smin, &smax, 0, 0, mask);
    scale = (dmax - dmin) * (smax - smin > DBL_EPSILON ? 1. / (smax - smin) : 0);
    shift = dmin - smin * scale;
  } else if (norm_type == CV_L2 || norm_type == CV_L1 || norm_type == CV_C) {
    //         CvMat *s = (CvMat*)src, *d = (CvMat*)dst;
    scale = cvNorm(src, 0, norm_type, mask);
    scale = scale > DBL_EPSILON ? 1. / scale : 0.;
    shift = 0;
  } else {
  }

  if (!mask)
    cvConvertScale(src, dst, scale, shift);
  else {
    //         CvMat stub, *dmat;
    cvConvertScale(src, tmp, scale, shift);
    cvCopy(tmp, dst, mask);
  }

  //    __END__;
  if (tmp)
    cvReleaseMat(&tmp);
}

/**
 * @brief CvGabor::conv_img(IplImage *src, IplImage *dst, int Type)
 * @param src
 * @param dst
 * @param Type
 */
void CvGabor::conv_img(IplImage *src, IplImage *dst, int Type)  //函数名：conv_img
{
  double ve;  //, re,im;

  CvMat *mat = cvCreateMat(src->width, src->height, CV_32FC1);
  for (int i = 0; i < src->width; i++) {
    for (int j = 0; j < src->height; j++) {
      ve = CV_IMAGE_ELEM(src, uchar, j, i);        // CV_IMAGE_ELEM 是取图像（j，i）位置的像素值
      CV_MAT_ELEM(*mat, float, i, j) = (float)ve;  //转化成float 类型
    }
  }

  CvMat *rmat = cvCreateMat(src->width, src->height, CV_32FC1);
  CvMat *imat = cvCreateMat(src->width, src->height, CV_32FC1);

  switch (Type) {
    case CV_GABOR_REAL:
      cvFilter2D((CvMat *)mat, (CvMat *)mat, (CvMat *)Real, cvPoint((Width - 1) / 2, (Width - 1) / 2));
      break;
    case CV_GABOR_IMAG:
      cvFilter2D((CvMat *)mat, (CvMat *)mat, (CvMat *)Imag, cvPoint((Width - 1) / 2, (Width - 1) / 2));
      break;
    case CV_GABOR_MAG:
      cvFilter2D((CvMat *)mat, (CvMat *)rmat, (CvMat *)Real, cvPoint((Width - 1) / 2, (Width - 1) / 2));
      cvFilter2D((CvMat *)mat, (CvMat *)imat, (CvMat *)Imag, cvPoint((Width - 1) / 2, (Width - 1) / 2));

      cvPow(rmat, rmat, 2);
      cvPow(imat, imat, 2);
      cvAdd(imat, rmat, mat);
      cvPow(mat, mat, 0.5);
      break;
    case CV_GABOR_PHASE:
      break;
  }

  if (dst->depth == IPL_DEPTH_8U) {
    cvNormalize((CvMat *)mat, (CvMat *)mat, 0, 255, CV_MINMAX);
    for (int i = 0; i < mat->rows; i++) {
      for (int j = 0; j < mat->cols; j++) {
        ve = CV_MAT_ELEM(*mat, float, i, j);
        CV_IMAGE_ELEM(dst, uchar, j, i) = (uchar)cvRound(ve);
      }
    }
  }

  if (dst->depth == IPL_DEPTH_32F) {
    for (int i = 0; i < mat->rows; i++) {
      for (int j = 0; j < mat->cols; j++) {
        ve = cvGetReal2D((CvMat *)mat, i, j);
        cvSetReal2D((IplImage *)dst, j, i, ve);
      }
    }
  }

  cvReleaseMat(&imat);
  cvReleaseMat(&rmat);
  cvReleaseMat(&mat);
}

}  // namespace v4r
