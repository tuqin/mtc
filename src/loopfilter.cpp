#include "loopfilter.h"
#include <iostream>
#include "MTC265.h"

#define DEBLOCK_PRINT_DIF		0	//是否打印差值
#define DEBLOCK_OUTPUT_IMAGE	0	//是否输出去块测试图
#define DEBLOCK_OUTPUT_YUV		0	//是否输出测试序列

#include <fstream>

#define ORIGIN_Y_PATH	"F:\\y_origin.bmp"
#define FILTERED_Y_PATH	"F:\\y_filtered.bmp"
#define DIF_Y_PATH		"F:\\y_difference.bmpshifb"
#define ORIGIN_CB_PATH	"F:\\cb_origin.bmp"
#define FILTERED_CB_PATH	"F:\\cb_filtered.bmp"
#define DIF_CB_PATH		"F:\\cb_difference.bmp"
#define ORIGIN_CR_PATH	"F:\\cr_origin.bmp"
#define FILTERED_CR_PATH	"F:\\cr_filtered.bmp"
#define DIF_CR_PATH		"F:\\cr_difference.bmp"
#define ORIGIN_YUV_PATH	"F:\\yuv_origin.yuv"
#define FILTERED_YUV_PATH "F:\\yuv_filtered.yuv"
bool	firstFrame;
//临时定义输出BMP用

#define DEBLOCK_SMALLEST_BLOCK 8

using namespace std;

extern UInt g_auiZscanToRaster [];
extern UInt g_auiRasterToZscan [];
Int qp;

void LoopFilterPic(MTC265_t *h, MTC265_Frame *pic, UInt nWidth, UInt nHeight	,Int nQP)
{
	qp=nQP;

	h->m_disableDeblockingFilterFlag = false;
	h->m_uiNumPartitions = 1 << ( h->ucMaxCUDepth<<1 );
	h->numCUInLine = h->usWidth / h->ucMaxCUWidth;
	if( (nWidth % h->ucMaxCUWidth) != 0 )
		h->numCUInLine++;
	
	extern void tempWriteBmpY832x480(MTC265_Frame* pic, bool filtered);		//源输出测试
	extern void tempWriteBmpC832x480(MTC265_Frame* pic, bool filtered);
	extern void tempWriteYUV832x480(MTC265_Frame* pic, bool filtered);

#if DEBLOCK_OUTPUT_IMAGE
	tempWriteBmpY832x480(pic, false);
	tempWriteBmpC832x480(pic, false);
#endif

#if DEBLOCK_OUTPUT_YUV
	tempWriteYUV832x480(pic, false);
#endif

	if ( h->m_disableDeblockingFilterFlag )
	{
		return;
	}

	UInt m_uiNumPartitions = 1 << ( h->ucMaxCUDepth<<1 );
	for( UInt uiDir = 0; uiDir < 2; uiDir++ )
	{
		h->m_aapucBS[uiDir] = (UInt8*) malloc(sizeof(UInt8) * m_uiNumPartitions);
		h->m_aapbEdgeFilter[uiDir] = (bool*) malloc(sizeof(bool) * m_uiNumPartitions);
    }
	//垂直
	for ( UInt uiCUIdx = 0; uiCUIdx < (h->usWidth/h->ucMaxCUWidth) * (h->usHeight/h->ucMaxCUWidth); uiCUIdx++ )
	{
		memset( h->m_aapucBS[EDGE_VER], 0, sizeof(UInt8) * m_uiNumPartitions );
		memset( h->m_aapbEdgeFilter[EDGE_VER], 0, sizeof( bool  ) * m_uiNumPartitions );

    // CU-based deblocking
		xDeblockCU( h, pic, uiCUIdx, 0, 0, EDGE_VER, nWidth, nHeight );
	}
	//水平
	for ( UInt uiCUIdx = 0; uiCUIdx < (h->usWidth/h->ucMaxCUWidth) * (h->usHeight/h->ucMaxCUWidth); uiCUIdx++ )
	{
		memset( h->m_aapucBS[EDGE_VER], 0, sizeof(UInt8) * m_uiNumPartitions );
		memset( h->m_aapbEdgeFilter[EDGE_VER], 0, sizeof( bool  ) * m_uiNumPartitions );

    // CU-based deblocking
		xDeblockCU( h, pic, uiCUIdx, 0, 0, EDGE_HOR, nWidth, nHeight );
	}

	for( UInt uiDir = 0; uiDir < 2; uiDir++ )
	{
		free(h->m_aapucBS[uiDir]);
		free(h->m_aapbEdgeFilter[uiDir]);
    }

#if DEBLOCK_OUTPUT_IMAGE
	tempWriteBmpY832x480(pic, true);		//输出测试
	tempWriteBmpC832x480(pic, true);
#endif

#if DEBLOCK_OUTPUT_YUV
	tempWriteYUV832x480(pic, true);
#endif
}

void xDeblockCU( MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt uiDepth, Int Edge, UInt nWidth, UInt nHeight )
{
	UInt m_uiNumPartitions = 1 << ( h->ucMaxCUDepth<<1 );
	UInt uiCurNumParts = m_uiNumPartitions >> (uiDepth<<1);
	UInt uiQNumParts = uiCurNumParts>>2;
  
	if(xGetDepth(h, uiCUIdx, uiAbsZorderIdx) > uiDepth)
	{
		for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++, uiAbsZorderIdx+=uiQNumParts )
		{
			{
				xDeblockCU( h, pic, uiCUIdx, uiAbsZorderIdx, uiDepth+1, Edge, nWidth, nHeight );
			}
		}
		return;
	}

	if( Edge == EDGE_VER )
		h->bLeftEdge = xSetLoopfilterParam( h, pic, uiCUIdx, uiAbsZorderIdx, Edge);
	else
		h->bTopEdge = xSetLoopfilterParam( h, pic, uiCUIdx, uiAbsZorderIdx, Edge);

	xSetEdgefilterTU ( h, pic, uiAbsZorderIdx, uiAbsZorderIdx, uiDepth );
	xSetEdgefilterPU ( h, pic, uiCUIdx, uiAbsZorderIdx, uiDepth );
  
	Int iDir = Edge;
	for( UInt uiPartIdx = uiAbsZorderIdx; uiPartIdx < uiAbsZorderIdx + uiCurNumParts; uiPartIdx++ )
	{
		UInt uiBSCheck;
		if( (h->ucMaxCUWidth >> h->ucMaxCUDepth) == 4 ) 
		{
			uiBSCheck = (iDir == EDGE_VER && uiPartIdx%2 == 0) || (iDir == EDGE_HOR && (uiPartIdx-((uiPartIdx>>2)<<2))/2 == 0);
		}
		else
		{
			uiBSCheck = 1;
		}

		if ( h->m_aapbEdgeFilter[iDir][uiPartIdx] && uiBSCheck )
		{
			xGetBoundaryStrengthSingle ( h, pic, uiAbsZorderIdx, iDir, uiPartIdx );
		}
	}
	UInt uiPelsInPart = h->ucMaxCUWidth >> h->ucMaxCUDepth;
	UInt PartIdxIncr = DEBLOCK_SMALLEST_BLOCK / uiPelsInPart ? DEBLOCK_SMALLEST_BLOCK / uiPelsInPart : 1 ;

	UInt uiSizeInPU = 1 << (h->ucMaxCUDepth - uiDepth);
  
	for ( UInt iEdge = 0; iEdge < uiSizeInPU ; iEdge+=PartIdxIncr)
	{
		xEdgeFilterLuma( h, pic, uiCUIdx, uiAbsZorderIdx, uiDepth, iDir, iEdge, nWidth, nHeight);
		if ( (uiPelsInPart>DEBLOCK_SMALLEST_BLOCK) || (iEdge % ( (DEBLOCK_SMALLEST_BLOCK<<1)/uiPelsInPart ) ) == 0 )
		{
			xEdgeFilterChroma( h, pic, uiCUIdx, uiAbsZorderIdx, uiDepth, iDir, iEdge );
		}
	}
}

void xSetEdgefilterMultiple( MTC265_t *h, MTC265_Frame* pic, UInt uiScanIdx, UInt uiDepth, Int iDir, Int iEdgeIdx, bool bValue, UInt uiWidthInBaseUnits, UInt uiHeightInBaseUnits )
{  
	const UInt uiNumElem = iDir == 0 ? uiHeightInBaseUnits : uiWidthInBaseUnits;
	assert( uiNumElem > 0 );
	assert( uiWidthInBaseUnits > 0 );
	assert( uiHeightInBaseUnits > 0 );
	for( UInt ui = 0; ui < uiNumElem; ui++ )
	{
		const UInt uiBsIdx = xCalcBsIdx( h, pic, uiScanIdx, iDir, iEdgeIdx, ui );
		h->m_aapbEdgeFilter[iDir][uiBsIdx] = bValue;
		if (iEdgeIdx == 0)
			h->m_aapucBS[iDir][uiBsIdx] = bValue;
	}
}

void xSetEdgefilterTU(MTC265_t *h, MTC265_Frame* pic, UInt uiAbsZorderIdx, UInt absTUPartIdx, UInt uiDepth)
{
	Int trWidth  = h->ucMaxCUWidth >> uiDepth;
	Int trHeight = h->ucMaxCUWidth >> uiDepth;
 
	UInt uiWidthInBaseUnits  = trWidth / h->ucMinCUWidth;
	UInt uiHeightInBaseUnits = trHeight / h->ucMinCUWidth;
	xSetEdgefilterMultiple( h, pic, absTUPartIdx, uiDepth, EDGE_VER, 0, h->bInternalEdge, uiWidthInBaseUnits, uiHeightInBaseUnits );
	xSetEdgefilterMultiple( h, pic, absTUPartIdx, uiDepth, EDGE_HOR, 0, h->bInternalEdge, uiWidthInBaseUnits, uiHeightInBaseUnits );
}

void xSetEdgefilterPU(MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt uiDepth)
{
	const UInt uiWidthInBaseUnits  = 1 << (h->ucMaxCUDepth - uiDepth);
	const UInt uiHeightInBaseUnits = 1 << (h->ucMaxCUDepth - uiDepth);
	const UInt uiHWidthInBaseUnits  = uiWidthInBaseUnits  >> 1;
	const UInt uiHHeightInBaseUnits = uiHeightInBaseUnits >> 1;
	const UInt uiQWidthInBaseUnits  = uiWidthInBaseUnits  >> 2;
	const UInt uiQHeightInBaseUnits = uiHeightInBaseUnits >> 2;
  
	xSetEdgefilterMultiple( h, pic, uiAbsZorderIdx, uiDepth, EDGE_VER, 0, h->bLeftEdge, uiWidthInBaseUnits, uiHeightInBaseUnits );
	xSetEdgefilterMultiple( h, pic, uiAbsZorderIdx, uiDepth, EDGE_HOR, 0, h->bTopEdge, uiWidthInBaseUnits, uiHeightInBaseUnits );
	switch (xGetPartSize(h, uiCUIdx, uiAbsZorderIdx))
	{
		case SIZE_2Nx2N:
		{
			break;
		}
		case SIZE_2NxN:
		{
			xSetEdgefilterMultiple(h, pic, uiAbsZorderIdx, uiDepth, EDGE_HOR, uiHHeightInBaseUnits, h->bInternalEdge, uiWidthInBaseUnits, uiHeightInBaseUnits);
			break;
		}
		case SIZE_Nx2N:
		{
			xSetEdgefilterMultiple(h, pic, uiAbsZorderIdx, uiDepth, EDGE_VER, uiHWidthInBaseUnits, h->bInternalEdge, uiWidthInBaseUnits, uiHeightInBaseUnits);
			break;
		}
		case SIZE_NxN:
		{
			xSetEdgefilterMultiple(h, pic, uiAbsZorderIdx, uiDepth, EDGE_VER, uiHWidthInBaseUnits, h->bInternalEdge,  uiHWidthInBaseUnits, uiHHeightInBaseUnits);
			xSetEdgefilterMultiple(h, pic, uiAbsZorderIdx, uiDepth, EDGE_HOR, uiHHeightInBaseUnits, h->bInternalEdge,  uiHWidthInBaseUnits, uiHHeightInBaseUnits);
			break;
		}
		case SIZE_2NxnU:
		{
			xSetEdgefilterMultiple(h, pic, uiAbsZorderIdx, uiDepth, EDGE_HOR, uiQHeightInBaseUnits, h->bInternalEdge, uiWidthInBaseUnits - uiQWidthInBaseUnits, uiHeightInBaseUnits - uiQHeightInBaseUnits);
			break;
		}
		case SIZE_2NxnD:
		{
			xSetEdgefilterMultiple(h, pic, uiAbsZorderIdx, uiDepth, EDGE_HOR, uiHeightInBaseUnits - uiQHeightInBaseUnits, h->bInternalEdge, uiQWidthInBaseUnits, uiQHeightInBaseUnits);
			break;
		}
		case SIZE_nLx2N:
		{
			xSetEdgefilterMultiple(h, pic, uiAbsZorderIdx, uiDepth, EDGE_VER, uiQWidthInBaseUnits, h->bInternalEdge, uiWidthInBaseUnits - uiQWidthInBaseUnits, uiHeightInBaseUnits - uiQHeightInBaseUnits);
			break;
		}
		case SIZE_nRx2N:
		{
			xSetEdgefilterMultiple(h, pic, uiAbsZorderIdx, uiDepth, EDGE_VER, uiWidthInBaseUnits - uiQWidthInBaseUnits, h->bInternalEdge, uiQWidthInBaseUnits, uiQHeightInBaseUnits);
			break;
		}
		default:
		{
			break;
		}
	}
}


bool xSetLoopfilterParam( MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt iEdge )
{
	UInt pelOffset = xCalcPelYIdxOffset( h, uiCUIdx, uiAbsZorderIdx );
	h->bInternalEdge = !h->m_disableDeblockingFilterFlag;
	bool bTempEdge;
	
	if( iEdge == EDGE_VER )
	{
		if((pelOffset % h->usWidth) == 0)
		{
			bTempEdge = false;
		}
		else
		{
			bTempEdge = true;
		}
	}
	else
	{
		if(pelOffset < h->usWidth)
		{
			bTempEdge = false;
		}
		else
		{
			bTempEdge = true;
		}
	}
	return bTempEdge;
}

void xGetBoundaryStrengthSingle (MTC265_t *h, MTC265_Frame* pic, UInt uiAbsZorderIdx, Int iDir, UInt uiAbsPartIdx)
{
	h->m_aapucBS[iDir][uiAbsPartIdx] = 2;
}


void xEdgeFilterLuma( MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt uiDepth, Int iDir, Int iEdge, UInt nWidth, UInt nHeight )
{
	Pxl*	piSrc = pic->pucY + xCalcPelYIdxOffset( h, uiCUIdx, uiAbsZorderIdx );
	Pxl*	piTmpSrc = piSrc;

	Int		iStride = h->usWidth;
	Int		iQP = 0;
	Int		iQP_P = 0;
	Int		iQP_Q = 0;
	UInt	uiNumParts = ( h->ucMaxCUWidth / h->ucMinCUWidth )>>uiDepth;
  
	UInt	uiPelsInPart = h->ucMaxCUWidth >>  h->ucMaxCUDepth;
	UInt	uiBsAbsIdx = 0, uiBs = 0;
	Int		iOffset, iSrcStep;
  
	bool	bPartPNoFilter = false;
	bool	bPartQNoFilter = false; 
	UInt	uiPartPIdx = 0;
	UInt	uiPartQIdx = 0;
  
	if (iDir == EDGE_VER)
	{
		iOffset = 1;
		iSrcStep = iStride;
		piTmpSrc += iEdge*uiPelsInPart;
	}
	else  // (iDir == EDGE_HOR)
	{
		iOffset = iStride;
		iSrcStep = 1;
		piTmpSrc += iEdge*uiPelsInPart*iStride;
	}

	for ( UInt iIdx = 0; iIdx < uiNumParts; iIdx++ )
	{
		uiBsAbsIdx = xCalcBsIdx( h, pic, uiAbsZorderIdx, iDir, iEdge, iIdx );
		uiBs = h->m_aapucBS[iDir][uiBsAbsIdx];
		if ( uiBs )
		{
			iQP_Q = qp;
			uiPartQIdx = uiBsAbsIdx;
			iQP_P = iQP_Q;
			iQP = (iQP_P + iQP_Q + 1) >> 1;
			Int iBitdepthScale = 1;

			Int m_betaOffsetDiv2 = 0;	
			Int m_tcOffsetDiv2 = 0;		
			Int iIndexTC = Clip3(0, MAX_QP+DEFAULT_INTRA_TC_OFFSET, Int(iQP + DEFAULT_INTRA_TC_OFFSET*(uiBs-1) + (m_tcOffsetDiv2 << 1)));
			Int iIndexB = Clip3(0, MAX_QP, iQP + (m_betaOffsetDiv2 << 1));
      
			Int iTc =  tctable_8x8[iIndexTC]*iBitdepthScale;
			Int iBeta = betatable_8x8[iIndexB]*iBitdepthScale;
			Int iSideThreshold = (iBeta+(iBeta>>1))>>3;
			Int iThrCut = iTc*10;

			UInt  uiBlocksInPart = uiPelsInPart / 4 ? uiPelsInPart / 4 : 1;
			for (UInt iBlkIdx = 0; iBlkIdx<uiBlocksInPart; iBlkIdx ++)
			{
				Int dp0 = xCalcDP( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+0), iOffset);
				Int dq0 = xCalcDQ( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+0), iOffset);
				Int dp3 = xCalcDP( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+3), iOffset);
				Int dq3 = xCalcDQ( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+3), iOffset);
				Int d0 = dp0 + dq0;
				Int d3 = dp3 + dq3;
       
				Int dp = dp0 + dp3;
				Int dq = dq0 + dq3;
				Int d =  d0 + d3;

				if (d < iBeta)
				{ 
					bool bFilterP = (dp < iSideThreshold);
					bool bFilterQ = (dq < iSideThreshold);
          
					bool sw =  xUseStrongFiltering( iOffset, 2*d0, iBeta, iTc, piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+0))
							&& xUseStrongFiltering( iOffset, 2*d3, iBeta, iTc, piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+3));
          
					for ( Int i = 0; i < DEBLOCK_SMALLEST_BLOCK/2; i++)
					{
#if DEBLOCK_PRINT_DIF
						cout<<"Offset:"<<iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+i)<<endl;	//测试
#endif
						xPelFilterLuma( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+i), iOffset, d, iBeta, iTc, sw, bPartPNoFilter, bPartQNoFilter, iThrCut, bFilterP, bFilterQ);
					}
				}
			}
		}
	}
}


void xEdgeFilterChroma(MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt uiDepth, Int iDir, Int iEdge)
{
	Int iStride = h->usWidth >> 1;
	Pxl *piSrcCb = pic->pucU + (xCalcPelCIdxOffset( h, uiCUIdx, uiAbsZorderIdx ));
	Pxl *piSrcCr = pic->pucV + (xCalcPelCIdxOffset( h, uiCUIdx, uiAbsZorderIdx ));
	Int iQP = 0;
	Int iQP_P = 0;
	Int iQP_Q = 0;

	UInt uiPelsInPartChroma = h->ucMaxCUWidth >> (h->ucMaxCUDepth+1);
  
	Int iOffset, iSrcStep;
  
	const UInt uiLCUWidthInBaseUnits = h->ucMaxCUWidth / h->ucMinCUWidth;
  
	bool bPartPNoFilter = false;
	bool bPartQNoFilter = false; 
	UInt uiPartQIdx;

	UInt uiEdgeNumInLCUVert = g_auiZscanToRaster[uiAbsZorderIdx]%uiLCUWidthInBaseUnits + iEdge;
	UInt uiEdgeNumInLCUHor = g_auiZscanToRaster[uiAbsZorderIdx]/uiLCUWidthInBaseUnits + iEdge;
  
	if ( (uiPelsInPartChroma < DEBLOCK_SMALLEST_BLOCK) && (( (uiEdgeNumInLCUVert%(DEBLOCK_SMALLEST_BLOCK/uiPelsInPartChroma))&&(iDir==0) ) || ( (uiEdgeNumInLCUHor%(DEBLOCK_SMALLEST_BLOCK/uiPelsInPartChroma))&& iDir ) ))
	{
		return;
	}
  
	UInt uiNumParts = uiLCUWidthInBaseUnits >> uiDepth;
  
	UInt uiBsAbsIdx;
	UInt8 ucBs;
  
	Pxl *piTmpSrcCb = piSrcCb;
	Pxl *piTmpSrcCr = piSrcCr;
  
  
	if (iDir == EDGE_VER)
	{
		iOffset   = 1;
		iSrcStep  = iStride;
		piTmpSrcCb += iEdge*uiPelsInPartChroma;
		piTmpSrcCr += iEdge*uiPelsInPartChroma;
	}
	else  // (iDir == EDGE_HOR)
	{
		iOffset   = iStride;
		iSrcStep  = 1;
		piTmpSrcCb += iEdge*iStride*uiPelsInPartChroma;
		piTmpSrcCr += iEdge*iStride*uiPelsInPartChroma;
	}
  
	for( UInt iIdx = 0; iIdx < uiNumParts; iIdx++ )
	{
		ucBs = 0;

		uiBsAbsIdx = xCalcBsIdx( h, pic, uiAbsZorderIdx, iDir, iEdge, iIdx);
		ucBs = h->m_aapucBS[iDir][uiBsAbsIdx];
    
		if ( ucBs > 1)
		{
			iQP_Q = qp;
			uiPartQIdx = uiBsAbsIdx;

			iQP_P = qp;
			iQP = g_aucChromaScale[ Clip3(MIN_QP, MAX_QP, (iQP_P + iQP_Q + 1) >> 1) ];
			Int iBitdepthScale = 1;

			Int m_tcOffsetDiv2 = 0;	//临时

			Int iIndexTC = Clip3(0, MAX_QP+DEFAULT_INTRA_TC_OFFSET, iQP + DEFAULT_INTRA_TC_OFFSET*(ucBs - 1) + (m_tcOffsetDiv2 << 1));
			Int iTc = tctable_8x8[iIndexTC]*iBitdepthScale;

			for ( UInt uiStep = 0; uiStep < uiPelsInPartChroma; uiStep++ )
			{
				xPelFilterChroma( piTmpSrcCb + iSrcStep*(uiStep+iIdx*uiPelsInPartChroma), iOffset, iTc , bPartPNoFilter, bPartQNoFilter);
				xPelFilterChroma( piTmpSrcCr + iSrcStep*(uiStep+iIdx*uiPelsInPartChroma), iOffset, iTc , bPartPNoFilter, bPartQNoFilter);
			}
		}
	}
}

/**
 - Deblocking for the luminance component with strong or weak filter
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param d               d value
 \param beta            beta value
 \param tc              tc value
 \param sw              decision strong/weak filter
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param iThrCut         threshold value for weak filter decision
 \param bFilterSecondP  decision weak filter/no filter for partP
 \param bFilterSecondQ  decision weak filter/no filter for partQ
*/
__inline void xPelFilterLuma( Pxl* piSrc, Int iOffset, Int d, Int beta, Int tc , bool sw, bool bPartPNoFilter, bool bPartQNoFilter, Int iThrCut, bool bFilterSecondP, bool bFilterSecondQ)
{
	Int delta;

	Pxl m4  = piSrc[0];
	Pxl m3  = piSrc[-iOffset];
	Pxl m5  = piSrc[ iOffset];
	Pxl m2  = piSrc[-iOffset*2];
	Pxl m6  = piSrc[ iOffset*2];
	Pxl m1  = piSrc[-iOffset*3];
	Pxl m7  = piSrc[ iOffset*3];
	Pxl m0  = piSrc[-iOffset*4];

	if (sw)
	{
		piSrc[-iOffset]   = Clip3(m3-2*tc, m3+2*tc, ((m1 + 2*m2 + 2*m3 + 2*m4 + m5 + 4) >> 3));
		piSrc[0]          = Clip3(m4-2*tc, m4+2*tc, ((m2 + 2*m3 + 2*m4 + 2*m5 + m6 + 4) >> 3));
		piSrc[-iOffset*2] = Clip3(m2-2*tc, m2+2*tc, ((m1 + m2 + m3 + m4 + 2)>>2));
		piSrc[ iOffset]   = Clip3(m5-2*tc, m5+2*tc, ((m3 + m4 + m5 + m6 + 2)>>2));
		piSrc[-iOffset*3] = Clip3(m1-2*tc, m1+2*tc, ((2*m0 + 3*m1 + m2 + m3 + m4 + 4 )>>3));
		piSrc[ iOffset*2] = Clip3(m6-2*tc, m6+2*tc, ((m3 + m4 + m5 + 3*m6 + 2*m7 +4 )>>3));
	}
	else
	{
		/* Weak filter */
		delta = (9*(m4-m3) -3*(m5-m2) + 8)>>4 ;

		if ( abs(delta) < iThrCut )
		{
			delta = Clip3(-tc, tc, delta);        
			piSrc[-iOffset] = Clip((m3+delta));
			piSrc[0] = Clip((m4-delta));

			Int tc2 = tc>>1;
			if(bFilterSecondP)
			{
				Int delta1 = Clip3(-tc2, tc2, (( ((m1+m3+1)>>1)- m2+delta)>>1));
				piSrc[-iOffset*2] = Clip((m2+delta1));
			}
			if(bFilterSecondQ)
			{
				Int delta2 = Clip3(-tc2, tc2, (( ((m6+m4+1)>>1)- m5-delta)>>1));
				piSrc[ iOffset] = Clip((m5+delta2));
			}
		}
	}

	if(bPartPNoFilter)
	{
		piSrc[-iOffset] = m3;
		piSrc[-iOffset*2] = m2;
		piSrc[-iOffset*3] = m1;
	}
		if(bPartQNoFilter)
	{
		piSrc[0] = m4;
		piSrc[ iOffset] = m5;
		piSrc[ iOffset*2] = m6;
	}
}

/**
 - Deblocking of one line/column for the chrominance component
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 */
__inline void xPelFilterChroma( Pxl* piSrc, Int iOffset, Int tc, bool bPartPNoFilter, bool bPartQNoFilter)
{
	int delta;
  
	Pxl m4  = piSrc[0];
	Pxl m3  = piSrc[-iOffset];
	Pxl m5  = piSrc[ iOffset];
	Pxl m2  = piSrc[-iOffset*2];

	delta = Clip3(-tc,tc, (((( m4 - m3 ) << 2 ) + m2 - m5 + 4 ) >> 3) );
	piSrc[-iOffset] = Clip(m3+delta);
	piSrc[0] = Clip(m4-delta);

	if(bPartPNoFilter)
	{
		piSrc[-iOffset] = m3;
	}
	if(bPartQNoFilter)
	{
		piSrc[0] = m4;
	}
}

/**
 - Decision between strong and weak filter
 .
 \param offset         offset value for picture data
 \param d               d value
 \param beta            beta value
 \param tc              tc value
 \param piSrc           pointer to picture data
 */
__inline bool xUseStrongFiltering( Int offset, Int d, Int beta, Int tc, Pxl* piSrc)
{
	Pxl m4  = piSrc[0];
	Pxl m3  = piSrc[-offset];
	Pxl m7  = piSrc[ offset*3];
	Pxl m0  = piSrc[-offset*4];

	Int d_strong = abs(m0-m3) + abs(m7-m4);

	return ( (d_strong < (beta>>3)) && (d<(beta>>2)) && ( abs(m3-m4) < ((tc*5+1)>>1)) );
}

__inline Int xCalcDP( Pxl* piSrc, Int iOffset)
{
	return abs( piSrc[-iOffset*3] - 2*piSrc[-iOffset*2] + piSrc[-iOffset] ) ;
}
  
__inline Int xCalcDQ( Pxl* piSrc, Int iOffset)
{
	return abs( piSrc[0] - 2*piSrc[iOffset] + piSrc[iOffset*2] );
}

UInt xCalcBsIdx ( MTC265_t* h, MTC265_Frame* pic, UInt uiAbsZorderIdx, Int iDir, Int iEdgeIdx, Int iBaseUnitIdx )
{
    const UInt uiLCUWidthInBaseUnits = h->ucMaxCUWidth / h->ucMinCUWidth;
    if( iDir == 0 )
    {
      return g_auiRasterToZscan[g_auiZscanToRaster[uiAbsZorderIdx] + iBaseUnitIdx * uiLCUWidthInBaseUnits + iEdgeIdx ];
    }
    else
    {
      return g_auiRasterToZscan[g_auiZscanToRaster[uiAbsZorderIdx] + iEdgeIdx * uiLCUWidthInBaseUnits + iBaseUnitIdx ];
    }
  }

__inline UInt xCalcPelYIdxOffset( MTC265_t *h, UInt uiCUIdx, UInt uiAbsZorderIdx )
{
	UInt CU_X = h->ucMaxCUWidth * ( uiCUIdx % h->numCUInLine );
	UInt CU_Y = h->ucMaxCUWidth * ( uiCUIdx / h->numCUInLine );
	UInt block_X = (g_auiZscanToRaster[uiAbsZorderIdx] % (1 << h->ucMaxCUDepth)) * h->ucMinCUWidth;
	UInt block_Y = (g_auiZscanToRaster[uiAbsZorderIdx] / (1 << h->ucMaxCUDepth)) * h->ucMinCUWidth;
#if DEBLOCK_PRINT_DIF
	cout<<"CU"<<CU_X<<","<<CU_Y<<";block"<<block_X<<","<<block_Y<<endl;	//测试
#endif
	return CU_X + block_X + (CU_Y + block_Y) * h->usWidth;
}

__inline UInt xCalcPelCIdxOffset( MTC265_t *h, UInt uiCUIdx, UInt uiAbsZorderIdx )
{
	UInt CU_X = (h->ucMaxCUWidth * ( uiCUIdx % h->numCUInLine ))>>1;
	UInt CU_Y = (h->ucMaxCUWidth * ( uiCUIdx / h->numCUInLine ))>>1;
	UInt block_X = ((g_auiZscanToRaster[uiAbsZorderIdx] % (1 << h->ucMaxCUDepth)) * h->ucMinCUWidth)>>1;
	UInt block_Y = ((g_auiZscanToRaster[uiAbsZorderIdx] / (1 << h->ucMaxCUDepth)) * h->ucMinCUWidth)>>1;
#if DEBLOCK_PRINT_DIF
	cout<<"CU"<<CU_X<<","<<CU_Y<<";block"<<block_X<<","<<block_Y<<endl;	//测试
#endif
	return CU_X + block_X + (CU_Y + block_Y) * (h->usWidth >> 1);
}

__inline UInt8 xGetDepth(MTC265_t *h, UInt uiCUIdx, UInt uiAbsZorderIdx)
{
	return *(h->cache.FrameDepth + uiCUIdx * (h->m_uiNumPartitions) + uiAbsZorderIdx);
}

__inline UInt8 xGetPartSize(MTC265_t *h, UInt uiCUIdx, UInt uiAbsZorderIdx)
{
	return *(h->cache.FramePartSize + uiCUIdx * (h->m_uiNumPartitions) + uiAbsZorderIdx);
}

void tempWriteBmpY832x480(MTC265_Frame* pic, bool filtered)	
{
	unsigned char *buf;
	buf = (unsigned char *)malloc(sizeof(unsigned char)*832*480*3);
	memset( buf, 0, sizeof( unsigned char ) * 54 );
	buf[0] = 0x42;
	buf[1] = 0x4d;
	buf[10] = 0x36;
	buf[14] = 0x28;
	buf[18] = 0x40;
	buf[19] = 0x03;
	buf[22] = 0xe0;
	buf[23] = 0x01;
	buf[26] = 0x01;
	buf[28] = 0x18;
	buf[35] = 0x0c;
	buf[36] = 0x30;
	buf[38] = 0xc4;
	buf[39] = 0x0e;
	buf[42] = 0xc4;
	buf[43] = 0x0e;

	fstream bmpFile;
	if(filtered)
		bmpFile.open(FILTERED_Y_PATH,ios::out|ios::binary);
	else
		bmpFile.open(ORIGIN_Y_PATH,ios::out|ios::binary);
	bmpFile.write((char*)buf,54);
	for(int i = 0; i < 480; i++)
		for(int j = 0; j<832; j++)
		{
			buf[(i * 832 + j) * 3] = pic->pucY[(479 - i) * 832 + j];
			buf[(i * 832 + j) * 3 +1] = pic->pucY[(479 - i) * 832 + j];
			buf[(i * 832 + j) * 3 +2] = pic->pucY[(479 - i) * 832 + j];
		}
	bmpFile.write((char*)buf,832*480*3);
	bmpFile.close();
	if(filtered)
	{
		unsigned char *buf1,*difbuf;
		fstream difFile;
		buf1 = (unsigned char *)malloc(sizeof(unsigned char)*832*480*3);
		difbuf = (unsigned char *)malloc(sizeof(unsigned char)*832*480*3);
		bmpFile.open(ORIGIN_Y_PATH,ios::in|ios::binary);
		difFile.open(DIF_Y_PATH,ios::out|ios::binary);
		bmpFile.read((char*)buf1,54);
		difFile.write((char*)buf1,54);
		bmpFile.seekg(54,ios::beg);
		bmpFile.read((char*)buf1,832*480*3);
		bmpFile.close();
		int tempOut = 0;
		for(int i = 0; i < 480; i++)
			for(int j = 0; j<832; j++)
			{
				difbuf[(i * 832 + j) * 3] = 128 + (buf[(i * 832 + j) * 3] - buf1[(i * 832 + j) * 3])*5;
				difbuf[(i * 832 + j) * 3 + 1] = difbuf[(i * 832 + j) * 3];
				difbuf[(i * 832 + j) * 3 + 2] = difbuf[(i * 832 + j) * 3];
				if(difbuf[(i * 832 + j) * 3]!=128)
					tempOut++;
			}
		difFile.write((char*)difbuf,832*480*3);
		difFile.close();
#if DEBLOCK_PRINT_DIF
		cout<<"Difference of Y channel between origin and filtered: "<<tempOut<<endl;
#endif
		free(buf1);
		free(difbuf);
	}
	free(buf);
}

void tempWriteBmpC832x480(MTC265_Frame* pic, bool filtered)
{
	unsigned char *buf;
	buf = (unsigned char *)malloc(sizeof(unsigned char)*416*240*3);
	memset( buf, 0, sizeof( unsigned char ) * 54 );
	buf[0] = 0x42;
	buf[1] = 0x4d;
	buf[10] = 0x36;
	buf[14] = 0x28;
	buf[18] = 0xa0;
	buf[19] = 0x1;
	buf[22] = 0xf0;
	buf[26] = 0x1;
	buf[28] = 0x18;
	buf[35] = 0x0c;
	buf[36] = 0x30;
	buf[38] = 0xc4;
	buf[39] = 0x0e;
	buf[42] = 0xc4;
	buf[43] = 0x0e;

	fstream bmpFileCb;
	fstream bmpFileCr;
	if(filtered)
	{
		bmpFileCb.open(FILTERED_CB_PATH,ios::out|ios::binary);
		bmpFileCr.open(FILTERED_CR_PATH,ios::out|ios::binary);
	}
	else
	{
		bmpFileCb.open(ORIGIN_CB_PATH,ios::out|ios::binary);
		bmpFileCr.open(ORIGIN_CR_PATH,ios::out|ios::binary);
	}
	bmpFileCb.write((char*)buf,54);
	bmpFileCr.write((char*)buf,54);
	for(int i = 0; i < 240; i++)
		for(int j = 0; j< 416; j++)
		{
			buf[(i * 416 + j) * 3] = pic->pucU[(239 - i) * 416 + j];
			buf[(i * 416 + j) * 3 +1] = pic->pucU[(239 - i) * 416 + j];
			buf[(i * 416 + j) * 3 +2] = pic->pucU[(239 - i) * 416 + j];
		}
	bmpFileCb.write((char*)buf,416*240*3);
	bmpFileCb.close();
	if(filtered)
	{
		unsigned char *bufcb,*difbufcb;
		fstream difFileCb;
		bufcb = (unsigned char *)malloc(sizeof(unsigned char)*416*240*3);
		difbufcb = (unsigned char *)malloc(sizeof(unsigned char)*416*240*3);
		bmpFileCb.open(ORIGIN_CB_PATH,ios::in|ios::binary);
		difFileCb.open(DIF_CB_PATH,ios::out|ios::binary);
		bmpFileCb.read((char*)bufcb,54);
		difFileCb.write((char*)bufcb,54);
		bmpFileCb.seekg(54,ios::beg);
		bmpFileCb.read((char*)bufcb,416*240*3);
		bmpFileCb.close();
		int tempOut = 0;
		for(int i = 0; i < 240; i++)
			for(int j = 0; j< 416; j++)
			{
				difbufcb[(i * 416 + j) * 3] = 128 + (buf[(i * 416 + j) * 3] - bufcb[(i * 416 + j) * 3])*5;
				difbufcb[(i * 416 + j) * 3+1] = difbufcb[(i * 416 + j) * 3];
				difbufcb[(i * 416 + j) * 3+2] = difbufcb[(i * 416 + j) * 3];
				if(difbufcb[(i * 416 + j) * 3]!=128)
					tempOut++;
			}
		difFileCb.write((char*)difbufcb,416*240*3);
		difFileCb.close();
#if DEBLOCK_PRINT_DIF
		cout<<"Difference of Cb between origin and filtered: "<<tempOut<<endl;
#endif
		free(bufcb);
		free(difbufcb);
	}

	for(int i = 0; i < 240; i++)
		for(int j = 0; j< 416; j++)
		{
			buf[(i * 416 + j) * 3] = pic->pucV[(239 - i) * 416 + j];
			buf[(i * 416 + j) * 3 +1] = pic->pucV[(239 - i) * 416 + j];
			buf[(i * 416 + j) * 3 +2] = pic->pucV[(239 - i) * 416 + j];
		}
	bmpFileCr.write((char*)buf,416*240*3);
	bmpFileCr.close();
	if(filtered)
	{
		unsigned char *bufcr,*difbufcr;
		fstream difFileCr;
		bufcr = (unsigned char *)malloc(sizeof(unsigned char)*416*240*3);
		difbufcr = (unsigned char *)malloc(sizeof(unsigned char)*416*240*3);
		bmpFileCr.open(ORIGIN_CR_PATH,ios::in|ios::binary);
		difFileCr.open(DIF_CR_PATH,ios::out|ios::binary);
		bmpFileCr.read((char*)bufcr,54);
		difFileCr.write((char*)bufcr,54);
		bmpFileCr.seekg(54,ios::beg);
		bmpFileCr.read((char*)bufcr,416*240*3);
		bmpFileCr.close();
		int tempOut = 0;
		for(int i = 0; i < 240; i++)
			for(int j = 0; j< 416; j++)
			{
				difbufcr[(i * 416 + j) * 3] = 128 + (buf[(i * 416 + j) * 3] - bufcr[(i * 416 + j) * 3])*5;
				difbufcr[(i * 416 + j) * 3+1] = difbufcr[(i * 416 + j) * 3];
				difbufcr[(i * 416 + j) * 3+2] = difbufcr[(i * 416 + j) * 3];
				if(difbufcr[(i * 416 + j) * 3]!=128)
					tempOut++;
			}
		difFileCr.write((char*)difbufcr,416*240*3);
		difFileCr.close();
#if DEBLOCK_PRINT_DIF
		cout<<"Difference of Cr between origin and filtered: "<<tempOut<<endl;
#endif
		free(bufcr);
		free(difbufcr);
	}
	free(buf);
}

void tempWriteYUV832x480(MTC265_Frame* pic, bool filtered)
{
	unsigned char *buf;
	buf = (unsigned char*)malloc(sizeof(unsigned char)*599040);
	memcpy(buf, pic->pucY, sizeof(unsigned char)*399360);
	memcpy(buf+399360, pic->pucU, sizeof(unsigned char)*99840);
	memcpy(buf+499200, pic->pucV, sizeof(unsigned char)*99840);
	fstream yuvFile;
	if(!firstFrame)
	{
		if(filtered)
		{
			yuvFile.open(FILTERED_YUV_PATH,ios::out|ios::binary);
			firstFrame = 1;
		}
		else
			yuvFile.open(ORIGIN_YUV_PATH,ios::out|ios::binary);
	}
	else
	{
		if(filtered)
			yuvFile.open(FILTERED_YUV_PATH,ios::out|ios::binary|ios::app);
		else
			yuvFile.open(ORIGIN_YUV_PATH,ios::out|ios::binary|ios::app);
	}
	yuvFile.seekg(ios::end);
	yuvFile.write((char*)buf, sizeof(unsigned char)*599040);
	yuvFile.close();
	free(buf);
}
