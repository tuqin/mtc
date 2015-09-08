#include "MTC265.h"

void xDefaultParams(MTC265_t *h)
{
    memset(h, 0, sizeof(*h));

    h->ucProfileIdc                 =  0;
    h->ucLevelIdc                   =  0;

    // Params
    h->usWidth                      =  0;
    h->usHeight                     =  0;
    h->ucMaxCUWidth                 = 64;
    h->ucMinCUWidth                 =  4;
    h->ucMaxCUDepth                 =  1;
    h->ucQuadtreeTULog2MinSize      =  2;
    h->ucQuadtreeTULog2MaxSize      =  5;
    h->ucQuadtreeTUMaxDepthInter    =  1;
    h->ucQuadtreeTUMaxDepthIntra    =  1;
    h->ucMaxNumRefFrames            =  1;
    h->ucBitsForPOC                 =  8;
    h->ucMaxNumMergeCand            = MRG_MAX_NUM_CANDS_SIGNALED;
    h->ucTSIG                       =  5;

    // Feature
    h->bUseNewRefSetting            = FALSE;
    h->bUseSAO                      = FALSE;
    h->bUseLMChroma                 = FALSE;
    h->bMRG                         = FALSE;
    h->bLoopFilterDisable           = TRUE;
    h->bSignHideFlag                = FALSE;
    h->bEnableTMVPFlag              = TRUE;
}

int confirmPara(int bflag, const char* message)
{
    if (!bflag)
        return false;
    
    printf("Error: %s\n",message);
    return true;
}

int xCheckParams( MTC265_t *h )
{
    int check_failed = false; /* abort if there is a fatal configuration problem */

#define xConfirmPara(a,b) check_failed |= confirmPara(a,b)
    xConfirmPara( (h->ucMaxCUWidth >> (h->ucMaxCUDepth+1)) == (1 << h->ucQuadtreeTULog2MaxSize), "Assume (QuadtreeTULog2MinSize >= MinCUWidth) fail" );
    xConfirmPara( h->usWidth  % h->ucMinCUWidth, "Frame width should be multiple of minimum CU size");
    xConfirmPara( h->usHeight % h->ucMinCUWidth, "Frame height should be multiple of minimum CU size");
    xConfirmPara( h->ucMaxNumRefFrames > MAX_REF_NUM, "Currently, MTC265 can not support so many reference");

	xConfirmPara( h->GOPSize < 1 ,                                                            "GOP Size must be greater or equal to 1" );
    xConfirmPara( h->GOPSize > 1 &&  h->GOPSize % 2,                                          "GOP Size must be a multiple of 2, if GOP Size is greater than 1" );
    xConfirmPara( (h->IntraPeriod > 0 && h->IntraPeriod < h->GOPSize) || h->IntraPeriod == 0, "Intra period must be more than GOP size, or -1 , not 0" );
	xConfirmPara( h->IntraPeriod >=0&&(h->IntraPeriod%h->GOPSize!=0), "Intra period must be a multiple of GOPSize, or -1" );
	xConfirmPara( (h->usWidth  % (h->ucMaxCUWidth  >> (h->ucMaxCUDepth-1)))!=0,             "Resulting coded frame width must be a multiple of the minimum CU size");
    xConfirmPara( (h->usHeight % (h->ucMaxCUWidth >> (h->ucMaxCUDepth-1)))!=0,             "Resulting coded frame height must be a multiple of the minimum CU size");

#undef xConfirmPara
    if (check_failed)
    {
        exit(EXIT_FAILURE);
    }
    return 0;
}
