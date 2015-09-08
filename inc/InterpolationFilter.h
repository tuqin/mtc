#ifndef __INTERPOLATIONFILTER_H__
#define __INTERPOLATIONFILTER_H__

#define  USE_FRAC_INTER       0
typedef       void            Void;
typedef       bool            Bool;
typedef       short           Short;
typedef       short			  Pxl;
typedef       short           Pel;

//

#define NTAPS_LUMA        8 ///< Number of taps for luma
#define NTAPS_CHROMA      4 ///< Number of taps for chroma
#define IF_INTERNAL_PREC 14 ///< Number of bits for internal precision
#define IF_FILTER_PREC    6 ///< Log2 of sum of filter taps
#define IF_INTERNAL_OFFS (1<<(IF_INTERNAL_PREC-1)) ///< Offset used internally

// ====================================================================================================================
// Tables
// ====================================================================================================================

const short m_lumaFilter[4][NTAPS_LUMA] =
{
	{  0, 0,   0, 64,  0,   0, 0,  0 },
	{ -1, 4, -10, 58, 17,  -5, 1,  0 },
	{ -1, 4, -11, 40, 40, -11, 4, -1 },
	{  0, 1,  -5, 17, 58, -10, 4, -1 }
};

const short m_chromaFilter[8][NTAPS_CHROMA] =
{
	{  0, 64,  0,  0 },
	{ -2, 58, 10, -2 },
	{ -4, 54, 16, -2 },
	{ -6, 46, 28, -4 },
	{ -4, 36, 36, -4 },
	{ -4, 28, 46, -6 },
	{ -2, 16, 54, -4 },
	{ -2, 10, 58, -2 }
};
// ====================================================================================================================
// Bit-depth
// ====================================================================================================================

#define  g_bitDepthY 8
#define  g_bitDepthC 8

void filterCopy		(Int bitDepth, const Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast);
void filter			(Int bitDepth, Short const *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Short const *coeff,Int N, Bool isVertical, Bool isFirst, Bool isLast);
void filterHor		(Int bitDepth, Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height,         Bool isLast, Short const *coeff,Int N);
void filterVer		(Int bitDepth, Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, Short const *coeff,Int N);
void filterHorLuma			(Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Int frac,               Bool isLast );
void filterVerLuma			(Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast );
void filterHorChroma		(Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Int frac,               Bool isLast );
void filterVerChroma		(Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast );

//根据当前PU的帧间编码信息（包括参考帧、MV）给出帧间补偿MC
void xPredInterUni		(MTC265_t *h, UInt uiLCUIdx, UInt uiPartIdx, Int iWidth, Int iHeight, Int xFrac, Int yFrac, UInt8* ref,UInt8 eText);
#endif