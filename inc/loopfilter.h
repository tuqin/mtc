#ifndef _LOOPFILTER_H_
#define _LOOPFILTER_H_
#include "config.h"
#include "MTC265.h"

#define   EDGE_VER    0
#define   EDGE_HOR    1
//#include <cstdlib>
void LoopFilterPic(MTC265_t *h, MTC265_Frame *pic, UInt nWidth, UInt nHeight,Int nQP);
void xDeblockCU(MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt uiDepth, Int Edge, UInt nWidth, UInt nHeight);
void xSetEdgefilterTU(MTC265_t *h, MTC265_Frame* pic, UInt uiAbsZorderIdx, UInt absTUPartIdx, UInt uiDepth);
void xSetEdgefilterPU(MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt uiDepth);
void xGetBoundaryStrengthSingle(MTC265_t *h, MTC265_Frame* pic, UInt uiAbsZorderIdx, Int iDir, UInt uiAbsPartIdx);
void xEdgeFilterLuma(MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt uiDepth, Int iDir, Int iEdge, UInt nWidth, UInt nHeight);
void xEdgeFilterChroma(MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt uiDepth, Int iDir, Int iEdge);
bool xSetLoopfilterParam(MTC265_t *h, MTC265_Frame* pic, UInt uiCUIdx, UInt uiAbsZorderIdx, UInt iEdge);
void xSetEdgefilterMultiple(MTC265_t *h, MTC265_Frame* pic, UInt uiScanIdx, UInt uiDepth, Int iDir, Int iEdgeIdx, bool bValue, UInt uiWidthInBaseUnits, UInt uiHeightInBaseUnits);
UInt xCalcBsIdx ( MTC265_t* h, MTC265_Frame* pic, UInt uiAbsZorderIdx, Int iDir, Int iEdgeIdx, Int iBaseUnitIdx );
__inline void xPelFilterLuma( Pxl* piSrc, Int iOffset, Int d, Int beta, Int tc , bool sw, bool bPartPNoFilter, bool bPartQNoFilter, Int iThrCut, bool bFilterSecondP, bool bFilterSecondQ);
__inline void xPelFilterChroma( Pxl* piSrc, Int iOffset, Int tc, bool bPartPNoFilter, bool bPartQNoFilter);
__inline bool xUseStrongFiltering( Int offset, Int d, Int beta, Int tc, Pxl* piSrc);
__inline Int xCalcDP( Pxl* piSrc, Int iOffset );
__inline Int xCalcDQ( Pxl* piSrc, Int iOffset );
__inline UInt xCalcPelYIdxOffset( MTC265_t *h, UInt uiCUIdx, UInt uiAbsZorderIdx );
__inline UInt xCalcPelCIdxOffset( MTC265_t *h, UInt uiCUIdx, UInt uiAbsZorderIdx );
__inline UInt8 xGetDepth(MTC265_t *h, UInt uiCUIdx, UInt uiAbsZorderIdx);
__inline UInt8 xGetPartSize(MTC265_t *h, UInt uiCUIdx, UInt uiAbsZorderIdx);

static int alpha[52]={
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	4,4,5,6,7,8,9,10,12,13,15,17,20,
	22,25,28,32,36,40,45,50,56,63,71,
	80,90,101,113,127,144,162,182,203,
	226,255,255
};

static int beta[52]={
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	2,2,2,3,3,3,3,4,4,4,6,6,7,
	7,8,8,9,9,10,10,11,11,12,12,
	13,13,14,14,15,15,16,16,17,
	17,18,18
};

const UInt8 betatable_8x8[52] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
};

const UInt8 tctable_8x8[54] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,10,11,13,14,16,18,20,22,24
};

#endif