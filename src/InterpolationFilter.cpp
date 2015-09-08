#include "MTC265.h"
#include "InterpolationFilter.h"
#include <assert.h>

/**
 * \brief Apply unit FIR filter to a block of samples
 *
 * \param bitDepth   bitDepth of samples
 * \param src        Pointer to source samples
 * \param srcStride  Stride of source samples
 * \param dst        Pointer to destination samples
 * \param dstStride  Stride of destination samples
 * \param width      Width of block
 * \param height     Height of block
 * \param isFirst    Flag indicating whether it is the first filtering operation
 * \param isLast     Flag indicating whether it is the last filtering operation
 */
void filterCopy(Int bitDepth, const Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast)
{
	Int row, col;

	if ( isFirst == isLast ) 
	{
		for (row = 0; row < height; row++)
		{
			for (col = 0; col < width; col++)
			{
				dst[col] = src[col];
			}
			src += srcStride;
			dst += dstStride;
		}              
	}
 
	else if ( isFirst )
	{
		Int shift = IF_INTERNAL_PREC - bitDepth;

		for (row = 0; row < height; row++)
		{
			for (col = 0; col < width; col++)
			{
				Short val = src[col] << shift;
				dst[col] = val - (Short)IF_INTERNAL_OFFS;
			}

			src += srcStride;
			dst += dstStride;
		}          
	}
	else
	{
		Int shift = IF_INTERNAL_PREC - bitDepth;
		Short offset = IF_INTERNAL_OFFS;
		offset += shift?(1 << (shift - 1)):0;
		Short maxVal = (1 << bitDepth) - 1;
		Short minVal = 0;
		for (row = 0; row < height; row++)
		{
			for (col = 0; col < width; col++)
			{
				Short val = src[ col ];
				val = ( val + offset ) >> shift;
				if (val < minVal) val = minVal;
				if (val > maxVal) val = maxVal;
				dst[col] = val;
			}
			src += srcStride;
			dst += dstStride;
		}              
	}
}

/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
Void filter(Int bitDepth, Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Short const *coeff,Int N, Bool isVertical, Bool isFirst, Bool isLast)
{
	Int row, col;

	Short c[8];
	c[0] = coeff[0];
	c[1] = coeff[1];
	if ( N >= 4 )
	{
		c[2] = coeff[2];
		c[3] = coeff[3];
	}
	if ( N >= 6 )
	{
		c[4] = coeff[4];
		c[5] = coeff[5];
	}
	if ( N == 8 )
	{
		c[6] = coeff[6];
		c[7] = coeff[7];
	}

	Int cStride = ( isVertical ) ? srcStride : 1;
	src -= ( N/2 - 1 ) * cStride;

	Int offset;
	Short maxVal;
	Int headRoom = IF_INTERNAL_PREC - bitDepth;
	Int shift = IF_FILTER_PREC;
	if ( isLast )
	{
		shift += (isFirst) ? 0 : headRoom;
		offset = 1 << (shift - 1);
		offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
		maxVal = (1 << bitDepth) - 1;
	}
	else
	{
		shift -= (isFirst) ? headRoom : 0;
		offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
		maxVal = 0;
	}

	for (row = 0; row < height; row++)
	{
		for (col = 0; col < width; col++)
		{
			Int sum;

			sum  = src[ col + 0 * cStride] * c[0];
			sum += src[ col + 1 * cStride] * c[1];
			if ( N >= 4 )
			{
				sum += src[ col + 2 * cStride] * c[2];
				sum += src[ col + 3 * cStride] * c[3];
			}
			if ( N >= 6 )
			{
				sum += src[ col + 4 * cStride] * c[4];
				sum += src[ col + 5 * cStride] * c[5];
			}
			if ( N == 8 )
			{
				sum += src[ col + 6 * cStride] * c[6];
				sum += src[ col + 7 * cStride] * c[7];        
			}

			Short val = ( sum + offset ) >> shift;
			if ( isLast )
			{
				val = ( val < 0 ) ? 0 : val;
				val = ( val > maxVal ) ? maxVal : val;        
			}
			dst[col] = val;
		}
		src += srcStride;
		dst += dstStride;
	}    
}
/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
Void filterHor(Int bitDepth, Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Bool isLast, Short const *coeff,Int N)
{
  if ( isLast )
  {
	  filter(bitDepth, src, srcStride, dst, dstStride, width, height, coeff,N, false, true, true);
  }
  else
  {
      filter(bitDepth, src, srcStride, dst, dstStride, width, height, coeff,N, false, true, false);
  }
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDpeth   Sample bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
Void filterVer(Int bitDepth, Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, Short const *coeff,Int N)
{
	if ( isFirst && isLast )
	{
		filter(bitDepth, src, srcStride, dst, dstStride, width, height, coeff,N, true, true, true);
	}
	else if ( isFirst && !isLast )
	{
		filter(bitDepth, src, srcStride, dst, dstStride, width, height, coeff,N, true, true, false);
	}
	else if ( !isFirst && isLast )
	{
		filter(bitDepth, src, srcStride, dst, dstStride, width, height, coeff,N, true, false, true);
	}
	else
	{
		filter(bitDepth, src, srcStride, dst, dstStride, width, height, coeff,N, true, false, false);
	}      
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * \brief Filter a block of luma samples (horizontal)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void filterHorLuma(Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast )
{
	assert(frac >= 0 && frac < 4);

	if ( frac == 0 )
	{
		filterCopy(g_bitDepthY, src, srcStride, dst, dstStride, width, height, true, isLast );
	}
	else
	{
		filterHor(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac],NTAPS_LUMA);
	}
}

/**
 * \brief Filter a block of luma samples (vertical)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void filterVerLuma(Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast )
{
	assert(frac >= 0 && frac < 4);

	if ( frac == 0 )
	{
		filterCopy(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isFirst, isLast );
	}
	else
	{
		filterVer(g_bitDepthY, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter[frac],NTAPS_LUMA);
	}
}

/**
 * \brief Filter a block of chroma samples (horizontal)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void filterHorChroma(Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast )
{
	assert(frac >= 0 && frac < 8);

	if ( frac == 0 )
	{
		filterCopy(g_bitDepthC, src, srcStride, dst, dstStride, width, height, true, isLast );
	}
	else
	{
		filterHor(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac],NTAPS_CHROMA);
	}
}

/**
 * \brief Filter a block of chroma samples (vertical)
 *
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 */
Void filterVerChroma(Pxl *src, Int srcStride, Pxl *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast )
{
	assert(frac >= 0 && frac < 8);

	if ( frac == 0 )
	{
		filterCopy(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isFirst, isLast );
	}
	else
	{
		filterVer(g_bitDepthC, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac],NTAPS_CHROMA);
	}
}
