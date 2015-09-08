#include "MTC265.h"
#ifdef CHECK_SEI
#include "md5.h"
#endif
#include "loopfilter.h"
#include "InterpolationFilter.h"

extern UInt totalbits;
UInt g_auiRasterToPelX  [ MAX_PU_XY*MAX_PU_XY ] = { 0, };
UInt g_auiRasterToPelY  [ MAX_PU_XY*MAX_PU_XY ] = { 0, };
UInt g_auiRasterToZscan [MAX_PU_XY*MAX_PU_XY] = { 0, }; 
UInt g_auiZscanToRaster [MAX_PU_XY*MAX_PU_XY] = { 0, };
UInt g_motionRefer   [ MAX_NUM_SPU_W*MAX_NUM_SPU_W ] = { 0, }; 
// ***************************************************************************
// * Interface Functions
// ***************************************************************************
void xEncInit( MTC265_t *h, UInt nFrames )
{
	const UInt32 uiWidth    = (h->usWidth + h->PAD_YSIZEw);
    const UInt32 uiHeight   = (h->usHeight + h->PAD_YSIZEh);

    const UInt32 uiYSize    = uiWidth * uiHeight;
    int i;

	for( i=0; i < MAX_REF_NUM+1; i++ ) 
	{
		Pxl *ptr = (Pxl *)MALLOC(sizeof(Pxl)*uiYSize * 3 / 2);
		assert(ptr != NULL);
		h->refn[i].pucY = (Pxl *)ptr;
		h->refn[i].pucU = (Pxl *)ptr + uiYSize;
		h->refn[i].pucV = (Pxl *)ptr + uiYSize * 5 / 4;
	}
    h->iPoc = -1;

	// init psnr
	h->FpsnrY = (double *) malloc (sizeof(double)*nFrames); 
	h->FpsnrU = (double *) malloc (sizeof(double)*nFrames); 
	h->FpsnrV = (double *) malloc (sizeof(double)*nFrames);

	h->PUnumber = 3;//PU中含具体小PU的总个数，目前为SIZE_2Nx2N(1)，SIZE_2NxN(1)，SIZE_Nx2N(2)
	h->usAddRefLength = 16 + h->ucMaxCUWidth;
	h->usLargeWidth = h->usWidth + h->PAD_YSIZEw + 2 * h->usAddRefLength;
	h->usLargeHeight = h->usHeight + h->PAD_YSIZEh + 2 * h->usAddRefLength;

	const Int32 uiLCUNum = ((h->usWidth + h->PAD_YSIZEw)/h->ucMaxCUWidth)*((h->usHeight + h->PAD_YSIZEh)/h->ucMaxCUWidth);
	const UInt32 uiLargeYSize = h->usLargeWidth * h->usLargeHeight;
	Int32 TempInterLength = uiLCUNum * h->NumPartition * h->PUnumber * h->ucMaxCUDepth;//LCU个数*64*3*3

	h->TempInter = (Interpara *) malloc (sizeof(Interpara)*TempInterLength);  
	h->FrameBestMv = (MVpara *) malloc (sizeof(MVpara)*uiLCUNum*h->NumPartition);
	h->FrameBestMvd = (MVpara *) malloc (sizeof(MVpara)*uiLCUNum*h->NumPartition);
	h->FrameBestPartSize = (UInt8*) malloc (sizeof(UInt8)*uiLCUNum*h->NumPartition);
	h->FrameBestDepth = (UInt8 *) malloc (sizeof(UInt8)*uiLCUNum*h->NumPartition);
	h->FrameBestMergeFlag = (Int32 *) malloc (sizeof(Int32)*uiLCUNum*h->NumPartition);
	h->FrameBestMergeIdx = (Int32 *) malloc (sizeof(Int32)*uiLCUNum*h->NumPartition);
	h->FrameBestSkipFlag = (Int32 *) malloc (sizeof(Int32)*uiLCUNum*h->NumPartition);
	
	memset(h->TempInter, 0, sizeof(Interpara)*TempInterLength);
	memset(h->FrameBestMv, 0, sizeof(MVpara) * uiLCUNum*h->NumPartition);
	memset(h->FrameBestMvd, 0, sizeof(MVpara) * uiLCUNum*h->NumPartition);
	memset(h->FrameBestPartSize, 0, sizeof(UInt8) * uiLCUNum*h->NumPartition);
	memset(h->FrameBestDepth, 0, sizeof(UInt8) * uiLCUNum*h->NumPartition);
	memset(h->FrameBestMergeFlag, 0, sizeof(Int32) * uiLCUNum*h->NumPartition);
	memset(h->FrameBestMergeIdx, 0, sizeof(Int32) * uiLCUNum*h->NumPartition);
	memset(h->FrameBestSkipFlag, 0, sizeof(Int32) * uiLCUNum*h->NumPartition);

	Pxl *ptr = (Pxl *)malloc(sizeof(Pxl)*uiLargeYSize * 3 / 2);
	assert(ptr != NULL);
	h->LargeInter.pucY = (Pxl *)ptr;
	h->LargeInter.pucU = (Pxl *)ptr + uiLargeYSize;
	h->LargeInter.pucV = (Pxl *)ptr + uiLargeYSize * 5 / 4;
	memset(h->LargeInter.pucY, 0, sizeof(Pxl) * h->usLargeWidth * h->usLargeHeight * 3 / 2);

	UInt8 * p_tmpBufRecFrame =(UInt8*) malloc(sizeof(UInt8)*(h->usWidth+h->PAD_YSIZEw)*(h->usHeight+h->PAD_YSIZEh)*3/2 );
	assert( p_tmpBufRecFrame != NULL);
	h->pFrameRec8Bit.pucY = (UInt8 *)p_tmpBufRecFrame;
	h->pFrameRec8Bit.pucU = (UInt8 *)p_tmpBufRecFrame + (h->usWidth+h->PAD_YSIZEw)*(h->usHeight+h->PAD_YSIZEh);
	h->pFrameRec8Bit.pucV = (UInt8 *)p_tmpBufRecFrame + (h->usWidth+h->PAD_YSIZEw)*(h->usHeight+h->PAD_YSIZEh) * 5 / 4;
	memset(h->pFrameRec8Bit.pucY, 0, sizeof(UInt8) * (h->usWidth+h->PAD_YSIZEw)*(h->usHeight+h->PAD_YSIZEh)*3/2);
	
	for( int j=0; j < MAX_REF_NUM+1; j++ ) 
	{
		h->PrevInter[j] = (Interpara *) malloc (sizeof(Interpara)*TempInterLength);//LCU个数*64*3*3
		h->PrevFrameBestMv[j] = (MVpara *) malloc (sizeof(MVpara)*uiLCUNum*h->NumPartition);
		h->PrevFrameBestMvd[j] = (MVpara *) malloc (sizeof(MVpara)*uiLCUNum*h->NumPartition);
		h->PrevFrameBestPartSize[j] = (UInt8*) malloc (sizeof(UInt8)*uiLCUNum*h->NumPartition);
		h->PrevFrameBestDepth[j] = (UInt8 *) malloc (sizeof(UInt8)*uiLCUNum*h->NumPartition);
		memset(h->PrevInter[j], 0, (sizeof(Interpara)*TempInterLength));	
		memset(h->PrevFrameBestMv[j], 0, sizeof(MVpara) * uiLCUNum*h->NumPartition);
		memset(h->PrevFrameBestMvd[j], 0, sizeof(MVpara) * uiLCUNum*h->NumPartition);
		memset(h->PrevFrameBestPartSize[j], 0, sizeof(UInt8) * uiLCUNum*h->NumPartition);
		memset(h->PrevFrameBestDepth[j], 0, sizeof(UInt8) * uiLCUNum*h->NumPartition);
	}
}

void xEncFree( MTC265_t *h )
{
    int i;
    for( i=0; i < MAX_REF_NUM+1; i++ ) 
	{
        assert( h->refn[i].pucY != NULL );
        FREE( h->refn[i].pucY );
    }
    memset( h->refn, 0, sizeof(h->refn) );
	free( h->FpsnrY ); h->FpsnrY = NULL;
	free( h->FpsnrU ); h->FpsnrU = NULL; 
	free( h->FpsnrV ); h->FpsnrV = NULL;

	free(h->pFrameRec8Bit.pucY);
	h->pFrameRec8Bit.pucY = NULL;
	free(h->TempInter);
	h->TempInter = NULL;
	free(h->FrameBestMv);
	h->FrameBestMv = NULL;
	free(h->FrameBestMvd);
	h->FrameBestMv = NULL;
	free(h->FrameBestPartSize);
	h->FrameBestPartSize = NULL;
	free(h->FrameBestDepth);
	h->FrameBestDepth = NULL;
	free(h->FrameBestMergeFlag);
	h->FrameBestMergeFlag = NULL;
	free(h->FrameBestMergeIdx);
	h->FrameBestMergeIdx = NULL;
	free(h->FrameBestSkipFlag);
	h->FrameBestSkipFlag = NULL;

	free(h->LargeInter.pucY);
	h->LargeInter.pucY = NULL;

	for( int j=0; j < MAX_REF_NUM+1; j++ ) 
	{
		free(h->PrevInter[j]);////LCU个数*64*3*3
		h->PrevInter[j] = NULL;
		free(h->PrevFrameBestMv[j]);
		h->PrevFrameBestMv[j] = NULL;
		free(h->PrevFrameBestMvd[j]);
		h->PrevFrameBestMvd[j] = NULL;
		free(h->PrevFrameBestPartSize[j]);
		h->PrevFrameBestPartSize[j] = NULL;
		free(h->PrevFrameBestDepth[j]);
		h->PrevFrameBestDepth[j] = NULL;
	}
}

// ***************************************************************************
// * Main Functions
// ***************************************************************************
void CompressCU(MTC265_t *h,UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY)
{     
	if( h->isIntra )
	{
		xEncIntraLoadRef(h, uiCUX, uiCUY, h->ucMaxCUWidth, 0, 0,0 );  
		xEncIntraCompressCU( h, uiDepth, uiCUX, uiCUY );
	}
	else
	{
		double SAD_BestCU = 0;
		xEncInterCompressCU( h, uiDepth, 0, 0, SAD_BestCU );
	}
}

Int32 xEncEncode( MTC265_t *h, MTC265_Frame *pFrame, UInt8 *pucOutBuf, UInt32 uiBufSize, bool g_bSeqFirst )
{
    const UInt32    uiWidth     = h->usWidth;
    const UInt32    uiHeight    = h->usHeight;
    const UInt32    nMaxCuWidth = h->ucMaxCUWidth;
    MTC265_Cabac     *pCabac      = &h->cabac;
    MTC265_BitStream *pBS         = &h->bs;
    MTC265_Cache     *pCache      = &h->cache;
          Int       nQP         = h->iQP;
          Int       nQPC        = g_aucChromaScale[nQP];
    const Int32     lambda      = nQP;
    UInt x, y;
    UInt i;

    #ifdef CHECK_SEI
    UInt nOffsetSEI = 0;
    #endif

    // Copy to local
    h->pFrameCur = pFrame;
    h->pFrameRec = &h->refn[MAX_REF_NUM];

    // Initial local
    h->iPoc++;
    xBitStreamInit( pBS, pucOutBuf, uiBufSize );
	if( g_bSeqFirst )
	{
		// Write SPS Header
		xPutBits32(pBS, 0x01000000);
		xPutBits(pBS, 0x47, 8);
		xPutBits(pBS, 0x01, 8); // temporal_id and reserved_one_5bits
		xWriteSPS(h);
		xBitFlush(pBS);

		// Write PPS Header
		xPutBits32(pBS, 0x01000000);
		xPutBits(pBS, 0x48, 8);
		xPutBits(pBS, 0x01, 8); // temporal_id and reserved_one_5bits
		xWritePPS(h);
		xBitFlush(pBS);
	}
    #ifdef CHECK_SEI
    // Write SEI Header
    xPutBits32(pBS, 0x06010000);
    xPutBits(pBS, 0x01,        8); // temporal_id and reserved_one_5bits
    xPutBits(pBS, 0xFF01,     16); // PICTURE_DIGEST
    xPutBits(pBS, 0x11,        8); // Payload length
    xPutBits(pBS, 0x00,        8); // Method = MD5
    nOffsetSEI = xBitFlush(pBS);
    xPutBits(pBS, 0xA0A1A2A3, 32);
    xPutBits(pBS, 0xB4B5B6B7, 32);
    xPutBits(pBS, 0xC8C9CACB, 32);
    xPutBits(pBS, 0xDCDDDEDF, 32);
    xWriteRBSPTrailingBits(pBS);
    xBitFlush(pBS);
    #endif

    // Write Silces Header
    xPutBits32(pBS, (h->iPoc == 0 ? 0x45010000 : 0x41010000));
    xPutBits(pBS, 0x01, 8); // temporal_id and reserved_one_5bits

	if( h->IntraPeriod == 1 )
	{
	    h->eSliceType = SLICE_I;
		h->isIntra = TRUE;
        xWriteSliceHeader(h);
	}
	else if( h->IntraPeriod == -1 ) 
	{
		if( h->iPoc == 0 )
		{
		    h->eSliceType = SLICE_I;
			h->isIntra = TRUE;
		}
		else
		{
			h->eSliceType = SLICE_P;
			h->isIntra = FALSE;
		}
		xWriteSliceHeader(h);
	}
	else
	{
		if( h->iPoc%h->IntraPeriod == 0 )
		{
		    h->eSliceType = SLICE_I;
			h->isIntra = TRUE;
		}
		else
		{
			h->eSliceType = SLICE_P;
			h->isIntra = FALSE;
		}
		xWriteSliceHeader(h);
	}			
	if( h->eSliceType == SLICE_I )
	{
		printf("SLICE I\n");
	}
	else if( h->eSliceType == SLICE_P )
	{
		printf("SLICE P\n");
	}
	else
	{
		printf("SLICE B\n");
	}

    //初始化Z扫描
    UInt* piTmp = &g_auiZscanToRaster[0];
    initZscanToRaster( h->ucMaxCUDepth+1, 1, 0, piTmp);
    initRasterToZscan( h->ucMaxCUWidth, h->ucMaxCUWidth, h->ucMaxCUDepth+1 );
	initRasterToPelXY( h->ucMaxCUWidth, h->ucMaxCUWidth, h->ucMaxCUDepth+1 );
	initMotionReferIdx ( h->ucMaxCUWidth, h->ucMaxCUWidth, h->ucMaxCUDepth+1 );
	int TotalSAD=0;
	MTC265_Cache *cache = &h->cache;

    // Encode loop
    xEncCahceInit( h );

	const Int32 uiLCUNum = ((h->usWidth + h->PAD_YSIZEw)/h->ucMaxCUWidth)*((h->usHeight + h->PAD_YSIZEh)/h->ucMaxCUWidth);
	Int32 TempInterLength = uiLCUNum * h->NumPartition * h->PUnumber * h->ucMaxCUDepth;//LCU个数*64*3*3
	Interpara * StartInter1 = h->TempInter;
	for ( Int32 j=0 ; j < TempInterLength ; j++ )
	{
		StartInter1->iSadBest = MAX_DOUBLE;
		StartInter1->iSrchCenBest.m_x = 0;
		StartInter1->iSrchCenBest.m_y = 0;
		StartInter1->iMvp_index = 0;
		StartInter1->iMvBestY.m_x = 0;
		StartInter1->iMvBestY.m_y = 0;
		StartInter1->iMvpBestY.m_x = 0;
		StartInter1->iMvpBestY.m_y = 0;
		StartInter1->iMvdBestY.m_x = 0;
		StartInter1->iMvdBestY.m_y = 0;
		StartInter1->Mvp_0.m_x = 0;
		StartInter1->Mvp_0.m_y = 0;
		StartInter1->Mvp_1.m_x = 0;
		StartInter1->Mvp_1.m_y = 0;
		StartInter1->ucPointNr = 0;
		StartInter1->MergeFlag = 0;
		StartInter1->MergeIdx = 0;
		StartInter1->SkipFlag = 0;
		++StartInter1;
	}

    xCabacInit( h );
    xCabacReset( &h->cabac );

	for( y=0; y < (uiHeight+h->PAD_YSIZEh); y+=nMaxCuWidth )
	{
        h->uiCUY = y;
        xEncCahceInitLine( h, y );
  
        for( x=0; x < uiWidth; x+=nMaxCuWidth ) 
		{
			const UInt   bLastCU     =  (y == (uiHeight+h->PAD_YSIZEh) - nMaxCuWidth) && (x == (uiWidth+h->PAD_YSIZEw) - nMaxCuWidth);
            const UInt   nCUSize     = h->ucMaxCUWidth;
            const UInt   nLog2CUSize = xLog2(nCUSize-1);

            h->uiCUX = x;

            xEncCacheLoadCU( h, x, y, nMaxCuWidth );

 			CompressCU( h, 0, x, y );

		    xEncCacheStoreCU( h, x, y, nMaxCuWidth, 0 );

			if( h->isIntra)
			{
                xEncCacheUpdate( h, x, y, nCUSize, nCUSize );
			}

			UInt8 *LCUbytes_start = pBS->pucBits;	

            xWriteCU( h, 0, 0 );
			// FinishCU
            xCabacEncodeTerminatingBit( pCabac, pBS, bLastCU );
			
	        UInt8 *LCUbytes_end  = pBS->pucBits;
	        pCache->TotalBits = (UInt)( LCUbytes_end - LCUbytes_start ) * 8;

			xEncCacheReset(h); // RESET 0
			pCache->uiOffset += nCUSize;
        }
    }

	memcpy( h->refn[0].pucY, h->pFrameRec->pucY,(uiWidth + h->PAD_YSIZEw)*(uiHeight + h->PAD_YSIZEh)*3/2*sizeof(Pxl));
	if(!h->isIntra)
	{
		memcpy( h->PrevInter[0], h->TempInter, sizeof(Interpara) * TempInterLength);
		PrevChangeMv(h, h->PrevFrameBestMv[0],h->FrameBestMv);
		memcpy( h->PrevFrameBestMvd[0],  h->FrameBestMvd, sizeof(MVpara) * uiLCUNum*h->NumPartition);
		memcpy( h->PrevFrameBestPartSize[0], h->FrameBestPartSize, sizeof(UInt8) * uiLCUNum*h->NumPartition);
		memcpy( h->PrevFrameBestDepth[0], h->FrameBestDepth, sizeof(UInt8) * uiLCUNum*h->NumPartition);
	}
	else
	{
		memset(h->PrevFrameBestMv[0], 0, sizeof(MVpara) * uiLCUNum*h->NumPartition);
	}
	PreLargeInter(h);

    xCabacFlush( pCabac, pBS );
    xWriteSliceEnd( h );

	for(UInt32 i = 0; i< (uiWidth+h->PAD_YSIZEw)*(uiHeight+h->PAD_YSIZEh)*3/2;i++)
	{
		h->pFrameRec8Bit.pucY[i] = h->pFrameRec->pucY[i]; //transform reconstructed frame from 16bit to 8bit
	}

    #ifdef CHECK_SEI
    MD5Context ctx;
    MD5Init( &ctx );
	MD5Update( &ctx, h->pFrameRec8Bit.pucY,(uiWidth+h->PAD_YSIZEw)*(uiHeight+h->PAD_YSIZEh)*3/2 );
    MD5Final( &ctx, &pucOutBuf[nOffsetSEI] );
    #endif

    // Save Restruct
	if(h->bWriteReconFile)
    {
        static FILE *fpx=NULL;
        if ( fpx == NULL )
            fpx = fopen("OX.YUV", "wb");
        assert( fpx != NULL );
		fwrite(h->pFrameRec8Bit.pucY, 1, (uiWidth+h->PAD_YSIZEw)*(uiHeight+h->PAD_YSIZEh)*3/2, fpx);

        fflush(fpx);
    }

    // Update Reference Frame Pointer
    MTC265_Frame tmp = h->refn[MAX_REF_NUM];

	//LoopFilter
	if(h->bUseLoopFilter)
	{
		if( h->isIntra )
		{
			LoopFilterPic(h,&tmp,uiWidth,uiHeight,nQP);		
		}
	}
   
	xEncCahceDelete( h );//释放指针
    for( i=MAX_REF_NUM; i>0; i-- ) 
	{
        h->refn[i] = h->refn[i-1];
    }
    h->refn[0] = tmp;
	
	UInt8 *RBSPbytes_start = pBS->pucBits0;
	UInt8 *RBSPbytes_end   = pBS->pucBits;
	UInt uibits = (UInt)( RBSPbytes_end - RBSPbytes_start ) * 8;
	totalbits += uibits;

	printf(" %d bits\n",uibits );

    return xBitFlush( pBS );
}

// ***************************************************************************
// * Cache Manage Functions
// ***************************************************************************
void xEncCahceInitLine( MTC265_t *h, UInt y )
{
	MTC265_Cache *pCache  = &h->cache;
	pCache->uiOffset    = 0;
	memset( pCache->pucLeftModeY, MODE_INVALID, sizeof(UInt8)*(pCache->NumPartition) );	
}

void xEncCacheLoadCU( MTC265_t *h, UInt uiX, UInt uiY ,UInt uiCUWidth)
{
	MTC265_Cache  *pCache     = &h->cache;
	MTC265_Frame  *pFrame     = h->pFrameCur;
	const UInt   nCUWidth   = uiCUWidth;
	const UInt32 uiWidth    = h->usWidth+h->PAD_YSIZEw;
	const UInt32 uiOffsetY  = uiWidth * uiY + uiX;
	const UInt32 uiOffsetC  = uiWidth * uiY / 4 + uiX / 2;
	Pxl *pucSY0 = pFrame->pucY + uiOffsetY + 0*uiWidth;
	Pxl *pucSY1 = pFrame->pucY + uiOffsetY + 1*uiWidth;
	Pxl *pucSU  = pFrame->pucU + uiOffsetC;
	Pxl  *pucSV  = pFrame->pucV + uiOffsetC;
	Pxl  *pucDY0 = pCache->pucPixY + 0*MAX_CU_SIZE;
	Pxl  *pucDY1 = pCache->pucPixY + 1*MAX_CU_SIZE;
	Pxl  *pucDU  = pCache->pucPixU;
	Pxl  *pucDV  = pCache->pucPixV;
	UInt y;UInt x_cut = 0, y_cut = 0;
	pCache->cuX = uiX;
	pCache->cuY = uiY;
	if (uiX +uiCUWidth > h->usWidth)
		x_cut = h->PAD_YSIZEw;
	if (uiY +uiCUWidth > h->usHeight)
		y_cut = h->PAD_YSIZEh;

	for( y=0; y <((uiCUWidth)/2); y++ )
	{
		memcpy( pucDY0, pucSY0, sizeof(Pxl )*nCUWidth  );
		memcpy( pucDY1, pucSY1, sizeof(Pxl )*nCUWidth  );
		memcpy( pucDU,  pucSU,  sizeof(Pxl )*(nCUWidth ) / 2 );
		memcpy( pucDV,  pucSV,  sizeof(Pxl )*(nCUWidth ) / 2 );
		pucSY0 += uiWidth * 2;
		pucSY1 += uiWidth * 2;
		pucSU  += uiWidth / 2;
		pucSV  += uiWidth / 2;
		pucDY0 += MAX_CU_SIZE * 2;
		pucDY1 += MAX_CU_SIZE * 2;
		pucDU  += MAX_CU_SIZE / 2;
		pucDV  += MAX_CU_SIZE / 2;
	}
}

void xEncCacheStoreCU( MTC265_t *h, UInt uiX, UInt uiY, UInt   nCUWidth , UInt8  uiDepth)
{
	MTC265_Cache  *pCache     = &h->cache;
	MTC265_Frame  *pFrame     = h->pFrameRec;
	const UInt32 uiWidth    = h->usWidth+h->PAD_YSIZEw;
	const UInt32 uiOffsetY  = uiWidth * uiY + uiX;
	const UInt32 uiOffsetC  = uiWidth * uiY /4 + uiX / 2;
	Pxl *pucSY0 = NULL;
	Pxl *pucSY1 = NULL;
	Pxl *pucSU  = NULL;
	Pxl *pucSV  = NULL;
	pucSY0 = pCache->pucTempRecY[uiDepth]+ pCache->cuX+pCache->cuY*MAX_CU_SIZE + 0*MAX_CU_SIZE;
	pucSY1 = pCache->pucTempRecY[uiDepth]+ pCache->cuX+pCache->cuY*MAX_CU_SIZE + 1*MAX_CU_SIZE;
	pucSU  = pCache->pucTempRecU[uiDepth]+ pCache->cuX/2+pCache->cuY*MAX_CU_SIZE /4;
	pucSV  = pCache->pucTempRecV[uiDepth]+ pCache->cuX/2+pCache->cuY*MAX_CU_SIZE /4;

	Pxl *pucDY0 = pFrame->pucY + uiOffsetY + 0*uiWidth;
	Pxl *pucDY1 = pFrame->pucY + uiOffsetY + 1*uiWidth;
	Pxl *pucDU  = pFrame->pucU + uiOffsetC;
	Pxl *pucDV  = pFrame->pucV + uiOffsetC;
	UInt y;UInt x_cut =0, y_cut =0;
	if (uiX + nCUWidth > h->usWidth)
		x_cut = h->PAD_YSIZEw;
	if (uiY + nCUWidth > h->usHeight)
		y_cut = h->PAD_YSIZEh;
	for( y=0; y < (nCUWidth )/2; y++ ) 
	{
		memcpy( pucDY0, pucSY0, sizeof(Pxl)*nCUWidth );
		memcpy( pucDY1, pucSY1, sizeof(Pxl)*nCUWidth );
		memcpy( pucDU,  pucSU,  sizeof(Pxl)*(nCUWidth)/ 2 );
		memcpy( pucDV,  pucSV,  sizeof(Pxl)*(nCUWidth )/ 2 );
		pucSY0 += MAX_CU_SIZE * 2;
		pucSY1 += MAX_CU_SIZE * 2;
		pucSU  += MAX_CU_SIZE / 2;
		pucSV  += MAX_CU_SIZE / 2;
		pucDY0 += uiWidth*2;
		pucDY1 += uiWidth*2;
		pucDU  += uiWidth / 2;
		pucDV  += uiWidth / 2;
	}
}

void xEncCahceInit( MTC265_t *h )//修改初始化cache
{
	//给cache中的指针分配内存空间
	Int SIZE1 =(h->NumPartition)*(h->usWidth)/(h->ucMaxCUWidth);
	Int SIZE4 =(h->NumPartition)*(h->usWidth+h->PAD_YSIZEw)/(h->ucMaxCUWidth)*(h->usHeight+h->PAD_YSIZEh)/(h->ucMaxCUWidth);
	Int SIZE2 =(h->NumPartition);
	Int SIZE3 =(h->NumPartition)*(h->ucMaxCUDepth +1);
	MTC265_Cache *pCache  = &h->cache;
	memset( pCache, 0, sizeof(MTC265_Cache) );
	pCache->FrameModeY = (UInt8 *) malloc (sizeof(UInt8)*SIZE4); 
	pCache->FrameDepth = (UInt8 *) malloc (sizeof(UInt8)*SIZE4); 
	pCache->FramePartSize = (UInt8 *) malloc (sizeof(UInt8)*SIZE4); 
	pCache->TempbCbfY = (UInt8 *) malloc (sizeof(UInt8)*SIZE3); 
	pCache->TempbCbfU = (UInt8 *) malloc (sizeof(UInt8)*SIZE3); 
	pCache->TempbCbfV = (UInt8 *) malloc (sizeof(UInt8)*SIZE3); 
	pCache->pucTopModeY = (UInt8 *) malloc (sizeof(UInt8)*SIZE1);                     
    pCache->pucLeftModeY = (UInt8 *) malloc (sizeof(UInt8)*SIZE2); 
	pCache->PartIdx = (Int *) malloc (sizeof(Int)*(h->ucMaxCUDepth+1));                 //初始化大小为MaxDepth
	pCache->RelativeIdx = (Int *) malloc (sizeof(Int)*(h->ucMaxCUDepth+1));             //初始化大小为MaxDepth
	pCache->TemppuhWidth = (UInt8 *) malloc (sizeof(UInt8)*SIZE3);                      //临时存储划分块的宽度 初始化时分配大小为NumPartition*(MaxDepth+1)*sizeof(char)
	pCache->TemppuhHeight = (UInt8 *) malloc (sizeof(UInt8)*SIZE3);                     //临时存储划分块的高度    如上
	pCache->TemppuhDepth = (UInt8 *) malloc (sizeof(UInt8)*SIZE3);                      //临时存储划分块的深度    如上
	pCache->TemppePartSize  = (UInt8 *) malloc (sizeof(UInt8)*SIZE3);                   //临时存储SIZE_2N*2N或者SIZE_N*N   如上
	pCache->TemppePredMode  = (UInt8 *) malloc (sizeof(UInt8)*SIZE3);  
	pCache->TemppuhLumaIntraDir = (UInt8*) malloc (sizeof(UInt8)*SIZE3);               //临时存储划分块的亮度预测模式   如上
	pCache->TemppuhChromaIntraDir = (UInt8 *) malloc (sizeof(UInt8)*SIZE3);            //临时存储划分块的色度预测模式   如上
	pCache->TempTotalCost = (double *) malloc (sizeof(double)*SIZE3);                  //临时存储每一深度的cu的总Cost   初始化时分配大小为MaxDepth*sizeof(Double)
	pCache->TempTotalDistortion =(double *) malloc (sizeof(double)*SIZE3);             //临时存储每一深度的cu的总失真
	pCache->TempSadC = (double *) malloc (sizeof(double)*SIZE3);  
	pCache->puhWidth = 0;              //存储划分块的宽度 初始化时分配大小为NumPartitionsizeof(char)
	pCache->puhHeight = 0;             //存储划分块的高度    如上
	pCache->puhDepth = 0 ;             //存储划分块的深度    如上
	pCache->pePartSize = 0;            //存储SIZE_2N*2N或者SIZE_N*N   如上
	pCache->puhLumaIntraDir = 0;       //存储划分块的亮度预测模式   如上
	pCache->puhChromaIntraDir = 0;     //存储划分块的色度预测模式   如上
	pCache->TotalCost = 0;             //存储cu的总Cost   初始化时分配大小为NumPartition*MaxDepth*sizeof(Double)
	pCache->TotalDistortion = 0;       //存储cu的总失真
	pCache->TotalDistortionC = 0;
	pCache->TotalBits = 0 ;            //存储cu的总比特数
	pCache->TotalBins = 0;           
	pCache->bCbfY = 0; 
	pCache->bCbfU = 0; 
	pCache->bCbfV = 0; 
	memset( pCache->bCbf, 0, sizeof(UInt8)*SIZE2*3 );
	memset( pCache->pucTopModeY, MODE_INVALID, sizeof(UInt8)*SIZE1 );
	memset( pCache->pucLeftModeY, MODE_INVALID, sizeof(UInt8)*SIZE2 );
    memset( pCache->TemppuhLumaIntraDir, MODE_INVALID, sizeof(UInt8)*SIZE3 );
	memset( pCache->TemppuhChromaIntraDir, MODE_INVALID, sizeof(UInt8)*SIZE3 );
	memset( pCache->TempTotalCost,0 , sizeof(double)*SIZE3 );
	memset( pCache->TempTotalDistortion,0 , sizeof(double)*SIZE3 );
	memset( pCache->TemppuhWidth,0 , sizeof(UInt8)*SIZE3 );
	memset( pCache->TemppuhHeight,0 , sizeof(UInt8)*SIZE3 );
	memset( pCache->TemppuhDepth,0 , sizeof(UInt8)*SIZE3 );
    memset( pCache->TemppePartSize,0 , sizeof(UInt8)*SIZE3 );
	memset( pCache->PartIdx, 0, sizeof(Int)*(h->ucMaxCUDepth+1) );
	memset( pCache->RelativeIdx, 0, sizeof(Int)*(h->ucMaxCUDepth+1) );
	pCache->NumPartition =h->NumPartition ;                                            //CU中的最大划分块数量
	pCache->NumPartitionInWidth = Int( sqrt(double(pCache->NumPartition))) ;           //CU中的宽度的分块数量
	pCache->uiMaxCUWidth = h->ucMaxCUWidth;
	pCache->uiMinCUWidth = h->ucMaxCUDepth;
	pCache->Lambda = 0.57*pow( 2.0, (h->iQP-12)/3.0 );                                 //求lambda

	Int32 iLCUNum = ((h->usWidth + h->PAD_YSIZEw)/h->ucMaxCUWidth)*((h->usHeight + h->PAD_YSIZEh)/h->ucMaxCUWidth);
	Int32 TempInterLength = iLCUNum * pCache->NumPartition * h->PUnumber * h->ucMaxCUDepth;//LCU个数*64*3*3
	Int SIZE5 =(h->NumPartition)*h->ucMaxCUDepth;
	Int SIZE6 =(h->ucMaxCUDepth +1)* MAX_CU_SIZE*MAX_CU_SIZE;
	const UInt32 uiYSize = (h->usWidth + h->PAD_YSIZEw) * (h->usHeight + h->PAD_YSIZEh);

	pCache->BestInterCbfY = (UInt8 *) malloc (sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优CbfY           
	pCache->BestInterCbfU = (UInt8 *) malloc (sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优CbfU               
	pCache->BestInterCbfV = (UInt8 *) malloc (sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优CbfV
	pCache->BestpuhMvd = (MVpara *) malloc (sizeof(MVpara)*SIZE2 );//NumPartition，64,存储最优MVd
	pCache->BestpuhMv = (MVpara *) malloc (sizeof(MVpara)*SIZE2 );//NumPartition，64,存储最优MV
	pCache->BestpuhMvpIdx = (Int32 *) malloc (sizeof(Int32)*SIZE2 );//NumPartition，64,存储最优MVpIdx
	pCache->BestpuhSkipFlag = (Int32 *) malloc (sizeof(Int32)*SIZE2 );//NumPartition，64,存储最优SkipFlag
	pCache->BestpuhMergeFlag = (Int32 *) malloc (sizeof(Int32)*SIZE2 );//NumPartition，64,存储最优MergeFlag
	pCache->BestpuhMergeIdx = (Int32 *) malloc (sizeof(Int32)*SIZE2 );//NumPartition，64,存储最优MergeIdx
	pCache->BestpuhInterDepth = (UInt8 *) malloc (sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优Depth
	pCache->BestpuhInterPartSize = (UInt8 *) malloc (sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优PartSize
	pCache->BestpuhInterSAD = (double *) malloc (sizeof(double)*SIZE2 );//NumPartition，64,存储各层CU的临时SAD

	pCache->TemppuhRefY = (Pxl *) malloc (sizeof(Pxl)*SIZE6);
	pCache->TemppuhRefU = (Pxl *) malloc (sizeof(Pxl)*SIZE6/4);
	pCache->TemppuhRefV = (Pxl *) malloc (sizeof(Pxl)*SIZE6/4);
	pCache->TemppuhMvd = (MVpara *) malloc (sizeof(MVpara)*SIZE5); 
	pCache->TemppuhMv = (MVpara *) malloc (sizeof(MVpara)*SIZE5); 
	pCache->TemppuhMvpIdx = (Int32 *) malloc (sizeof(Int32)*SIZE5); 
	pCache->TemppuhSkipFlag = (Int32 *) malloc (sizeof(Int32)*SIZE5 );//存储最优SkipFlag
	pCache->TemppuhMergeFlag = (Int32 *) malloc (sizeof(Int32)*SIZE5 );//存储最优MergeFlag
	pCache->TemppuhMergeIdx = (Int32 *) malloc (sizeof(Int32)*SIZE5 );//存储最优MergeIdx
	pCache->TemppuhInterPartSize = (UInt8 *) malloc (sizeof(UInt8)*SIZE5); 
	pCache->TemppuhInterSAD = (double *) malloc (sizeof(double)*SIZE5); 
	pCache->PUPartIdx = (Int32 *) malloc (sizeof(Int32)*h->ucMaxCUDepth); 
	pCache->PURelativeIdx = (Int32 *) malloc (sizeof(Int32)*h->ucMaxCUDepth);
	pCache->TranUsedFlag = (Int32 *) malloc (sizeof(Int32)*SIZE2);

	memset( pCache->BestInterCbfY, 0, sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优CbfY           
	memset( pCache->BestInterCbfU, 0, sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优CbfU               
	memset( pCache->BestInterCbfV, 0, sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优CbfV
	memset( pCache->BestpuhMvd, 0, sizeof(MVpara)*SIZE2 );//NumPartition，64,存储最优MVd
	memset( pCache->BestpuhMv, 0, sizeof(MVpara)*SIZE2 );//NumPartition，64,存储最优MV
	memset( pCache->BestpuhMvpIdx, 0, sizeof(Int32)*SIZE2 );//NumPartition，64,存储最优MVpIdx
	memset( pCache->BestpuhSkipFlag, 0, sizeof(Int32)*SIZE2 );//NumPartition，64,存储最优SkipFlag
	memset( pCache->BestpuhMergeFlag, 0, sizeof(Int32)*SIZE2 );//NumPartition，64,存储最优MergeFlag
	memset( pCache->BestpuhMergeIdx, 0, sizeof(Int32)*SIZE2 );//NumPartition，64,存储最优MergeIdx
	memset( pCache->BestpuhInterDepth, h->ucMaxCUDepth-1, sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优Depth
	memset( pCache->BestpuhInterPartSize, 0, sizeof(UInt8)*SIZE2 );//NumPartition，64,存储最优PartSize
	memset( pCache->BestpuhInterSAD, 0, sizeof(double)*SIZE2 );//NumPartition，64,存储各层CU的临时SAD

	memset( pCache->TemppuhRefY, 0, sizeof(Pxl)*SIZE6 );
	memset( pCache->TemppuhRefU, 0, sizeof(Pxl)*SIZE6/4 );
	memset( pCache->TemppuhRefV, 0, sizeof(Pxl)*SIZE6/4 );
	memset( pCache->TemppuhMvd, 0, sizeof(MVpara)*SIZE5 );
	memset( pCache->TemppuhMv, 0, sizeof(MVpara)*SIZE5 );
	memset( pCache->TemppuhMvpIdx, 0, sizeof(Int32)*SIZE5 );
	memset( pCache->TemppuhSkipFlag, 0, sizeof(Int32)*SIZE5 );//存储最优SkipFlag
	memset( pCache->TemppuhMergeFlag, 0, sizeof(Int32)*SIZE5 );//存储最优MergeFlag
	memset( pCache->TemppuhMergeIdx, 0, sizeof(Int32)*SIZE5 );//存储最优MergeIdx
	memset( pCache->TemppuhInterPartSize, 0, sizeof(UInt8)*SIZE5 );
	memset( pCache->TemppuhInterSAD, 0, sizeof(double)*SIZE5); 
	memset( pCache->PUPartIdx, 0, sizeof(Int32)*h->ucMaxCUDepth );
	memset( pCache->PURelativeIdx, 0, sizeof(Int32)*h->ucMaxCUDepth );
	memset( pCache->TranUsedFlag, 0, sizeof(Int32)*SIZE2 );
}

//给cache中的指针释放内存空间
void xEncCahceDelete( MTC265_t *h )
{  
	MTC265_Cache *pCache  = &h->cache;
	free(pCache->pucTopModeY);
	pCache->pucTopModeY = NULL;
	free(pCache->pucLeftModeY);
	pCache->pucLeftModeY = NULL;
	free(pCache->PartIdx);
	pCache->PartIdx = NULL;
	free(pCache->RelativeIdx);
	pCache->RelativeIdx = NULL;
	free(pCache->TemppuhWidth);
	pCache->TemppuhWidth = NULL;
	free(pCache->TemppuhHeight);
	pCache->TemppuhHeight = NULL;
	free(pCache->TemppuhDepth);
	pCache->TemppuhDepth = NULL;
	free(pCache->TemppePartSize);
	pCache->TemppePartSize = NULL;
	free(pCache->TemppePredMode);
	pCache->TemppePredMode = NULL;
	free(pCache->TemppuhLumaIntraDir);
	pCache->TemppuhLumaIntraDir = NULL;
	free(pCache->TemppuhChromaIntraDir);
	pCache->TemppuhChromaIntraDir = NULL;
	free(pCache->TempTotalCost);
	pCache->TempTotalCost = NULL;
	free(pCache->TempTotalDistortion);
	pCache->TempTotalDistortion = NULL;
	free(pCache->FrameModeY); 
	pCache->FrameModeY = NULL;
	free(pCache->FrameDepth );
	pCache->FrameDepth  = NULL;
	free(pCache->FramePartSize ); 
	pCache->FramePartSize = NULL;
	free(pCache->TempSadC);
	pCache->TempSadC = NULL;
	free(pCache->TempbCbfY ); 
	pCache->TempbCbfY = NULL;
	free(pCache->TempbCbfU ); 
	pCache->TempbCbfU = NULL;
	free(pCache->TempbCbfV );
	pCache->TempbCbfV = NULL;

	free(pCache->BestpuhMvd);//NumPartition，64,存储最优MVd
	pCache->BestpuhMvd = NULL;
	free(pCache->BestpuhMv);//NumPartition，64,存储最优MV
	pCache->BestpuhMv = NULL;
	free(pCache->BestpuhMvpIdx);//NumPartition，64,存储最优MVpIdx
	pCache->BestpuhMvpIdx = NULL;
	free(pCache->BestpuhSkipFlag);//NumPartition，64,存储最优SkipFlag
	pCache->BestpuhSkipFlag = NULL;
	free(pCache->BestpuhMergeFlag);//NumPartition，64,存储最优MergeFlag
	pCache->BestpuhMergeFlag= NULL;
	free(pCache->BestpuhMergeIdx);//NumPartition，64,存储最优MergeIdx
	pCache->BestpuhMergeIdx = NULL;
	free(pCache->BestpuhInterDepth);//NumPartition，64,存储最优Depth
	pCache->BestpuhInterDepth = NULL;
	free(pCache->BestpuhInterPartSize);//NumPartition，64,存储最优PartSize
	pCache->BestpuhInterPartSize = NULL;
	free(pCache->BestpuhInterSAD);//NumPartition，64,存储各层CU的临时SAD
	pCache->BestpuhInterSAD = NULL;
	free(pCache->BestInterCbfY);//NumPartition，64,存储最优CbfY 
	pCache->BestInterCbfY = NULL;
	free(pCache->BestInterCbfU);//NumPartition，64,存储最优CbfU  
	pCache->BestInterCbfU = NULL;
	free(pCache->BestInterCbfV);//NumPartition，64,存储最优CbfV
	pCache->BestInterCbfV = NULL;

	free(pCache->TemppuhRefY);//ucMaxCUDepth*ucMaxCUWidth*ucMaxCUWidth,3*64,存储各层CU的临时参考帧Y数据
	pCache->TemppuhRefY = NULL;
	free(pCache->TemppuhRefU);//ucMaxCUDepth*(ucMaxCUWidth/2)*(ucMaxCUWidth/2),3*32*32,存储各层CU的临时参考帧U数据
	pCache->TemppuhRefU = NULL;
	free(pCache->TemppuhRefV);//ucMaxCUDepth*(ucMaxCUWidth/2)*(ucMaxCUWidth/2),3*32*32存储各层CU的临时参考帧V数据
	pCache->TemppuhRefV = NULL;
	free(pCache->TemppuhMvd);
	pCache->TemppuhMvd = NULL;
	free(pCache->TemppuhMv);
	pCache->TemppuhMv = NULL;
	free(pCache->TemppuhMvpIdx);
	pCache->TemppuhMvpIdx = NULL;
	free(pCache->TemppuhSkipFlag);
	pCache->TemppuhSkipFlag = NULL;
	free(pCache->TemppuhMergeFlag);
	pCache->TemppuhMergeFlag = NULL;
	free(pCache->TemppuhMergeIdx);
	pCache->TemppuhMergeIdx = NULL;
	free(pCache->TemppuhInterPartSize);
	pCache->TemppuhInterPartSize = NULL;
	free(pCache->TemppuhInterSAD);
	pCache->TemppuhInterSAD = NULL;
	free(pCache->PUPartIdx);
	pCache->PUPartIdx = NULL;
	free(pCache->PURelativeIdx);
	pCache->PURelativeIdx = NULL;
	free(pCache->TranUsedFlag);
	pCache->TranUsedFlag= NULL;
}

void xEncCacheUpdate( MTC265_t *h, UInt32 uiX, UInt32 uiY, UInt nWidth, UInt nHeight )
{
	MTC265_Cache  *pCache         = &h->cache;
	MTC265_Frame  *pFrameRec      = h->pFrameRec; 
	UInt16 Stride               = h->usWidth;
	UInt8* pucTopModeY    =  pCache->pucTopModeY;                 //初始化大小为NumPartition*usWidth/MaxCuDepth 
	UInt8* pucLeftModeY   =  pCache->pucLeftModeY ;
	UInt8* TemppuhLumaIntraDir = pCache->TemppuhLumaIntraDir;
	UInt8* TemppuhDepth   = pCache->TemppuhDepth;

	//更新预测模式
	memcpy(pucTopModeY+ h->NumPartition*(uiX/(h->ucMaxCUWidth)), TemppuhLumaIntraDir,sizeof(UInt8)*h->NumPartition ); 
	memcpy(pucLeftModeY, TemppuhLumaIntraDir,sizeof(UInt8)*h->NumPartition );
	memcpy(pCache->FrameModeY+ h->NumPartition*(uiX/h->ucMaxCUWidth+(uiY/(h->ucMaxCUWidth)*(h->usWidth/h->ucMaxCUWidth))), TemppuhLumaIntraDir,sizeof(UInt8)*h->NumPartition ); 
	memcpy(pCache->FrameDepth+ h->NumPartition*(uiX/h->ucMaxCUWidth+(uiY/(h->ucMaxCUWidth)*(h->usWidth/h->ucMaxCUWidth))), TemppuhDepth,sizeof(UInt8)*h->NumPartition ); 
}

void xEncCacheReset( MTC265_t *h)
{
	Int SIZE3 =(h->NumPartition)*(h->ucMaxCUDepth +1);
	MTC265_Cache  *pCache         = &h->cache;
	memset( pCache->TemppuhChromaIntraDir, MODE_INVALID, sizeof(UInt8)*SIZE3 );
	memset( pCache->TemppuhLumaIntraDir, MODE_INVALID, sizeof(UInt8)*SIZE3 );
	memset( pCache->PartIdx, 0, sizeof(pCache->PartIdx) );
	memset( pCache->RelativeIdx, 0, sizeof(pCache->RelativeIdx) );
	memset( pCache->TemppuhWidth, 0, sizeof(UInt8)*SIZE3 );
	memset( pCache->TemppuhHeight, 0, sizeof(UInt8)*SIZE3 );
	memset( pCache->TemppuhDepth, 0, sizeof(UInt8)*SIZE3 );
	memset( pCache->TemppePartSize, 0, sizeof(UInt8)*SIZE3 );
	memset( pCache->TempTotalCost, 0, sizeof(UInt8)*SIZE3 );
	memset( pCache->TempTotalDistortion, 0, sizeof(UInt8)*SIZE3 );
	memset( pCache->pucTempRecY, 0, sizeof(pCache->pucTempRecY) );
	memset( pCache->pucTempRecU, 0, sizeof(pCache->pucTempRecU) );
	memset( pCache->pucTempRecV, 0, sizeof(pCache->pucTempRecV) );
	memset( pCache->psTempCoefY, 0, sizeof(pCache->psTempCoefY) );
	memset( pCache->psTempCoefU, 0, sizeof(pCache->psTempCoefU) );
	memset( pCache->psTempCoefV, 0, sizeof(pCache->psTempCoefV) );

	memset( pCache->TranUsedFlag, 0, sizeof(Int32)*pCache->NumPartition);//赋值已变换表示flag为0
	memset( pCache->BestpuhInterDepth, h->ucMaxCUDepth-1, sizeof(UInt8)*pCache->NumPartition );//NumPartition，64,存储最优Depth
}

// ***************************************************************************
// * Supplementary Functions
// ***************************************************************************
void initZscanToRaster ( Int iMaxDepth, UInt8 iDepth, UInt uiStartVal, UInt*& rpuiCurrIdx )
{
  Int iStride = 1 << ( iMaxDepth - 1 );
  
  if ( iDepth == iMaxDepth )
  {
    rpuiCurrIdx[0] = uiStartVal;
    rpuiCurrIdx++;
  }
  else
  {
    Int iStep = iStride >> iDepth;
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal,                     rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep,               rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep*iStride,       rpuiCurrIdx );
    initZscanToRaster( iMaxDepth, iDepth+1, uiStartVal+iStep*iStride+iStep, rpuiCurrIdx );
  }
}

void initRasterToZscan ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
  UInt  uiMinCUWidth  = uiMaxCUWidth  >> ( uiMaxDepth - 1 );
  UInt  uiMinCUHeight = uiMaxCUHeight >> ( uiMaxDepth - 1 );
  
  UInt  uiNumPartInWidth  = (UInt)uiMaxCUWidth  / uiMinCUWidth;
  UInt  uiNumPartInHeight = (UInt)uiMaxCUHeight / uiMinCUHeight;
  
  for ( UInt i = 0; i < uiNumPartInWidth*uiNumPartInHeight; i++ )
  {
    g_auiRasterToZscan[ g_auiZscanToRaster[i] ] = i;
  }
}

void initRasterToPelXY ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
  UInt    i;
  
  UInt* uiTempX = &g_auiRasterToPelX[0];
  UInt* uiTempY = &g_auiRasterToPelY[0];
  
  UInt  uiMinCUWidth  = uiMaxCUWidth  >> ( uiMaxDepth - 1 );
  UInt  uiMinCUHeight = uiMaxCUHeight >> ( uiMaxDepth - 1 );
  
  UInt  uiNumPartInWidth  = uiMaxCUWidth  / uiMinCUWidth;
  UInt  uiNumPartInHeight = uiMaxCUHeight / uiMinCUHeight;
  
  uiTempX[0] = 0; uiTempX++;
  for ( i = 1; i < uiNumPartInWidth; i++ )
  {
    uiTempX[0] = uiTempX[-1] + uiMinCUWidth; uiTempX++;
  }
  for ( i = 1; i < uiNumPartInHeight; i++ )
  {
    memcpy(uiTempX, uiTempX-uiNumPartInWidth, sizeof(UInt)*uiNumPartInWidth);
    uiTempX += uiNumPartInWidth;
  }
  
  for ( i = 1; i < uiNumPartInWidth*uiNumPartInHeight; i++ )
  {
    uiTempY[i] = ( i / uiNumPartInWidth ) * uiMinCUWidth;
  }
}

void initMotionReferIdx ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth )
{
	Int  minSUWidth  = (Int)uiMaxCUWidth  >> ( (Int)uiMaxDepth - 1 );
	Int  minSUHeight = (Int)uiMaxCUHeight >> ( (Int)uiMaxDepth - 1 );

	Int  numPartInWidth  = (Int)uiMaxCUWidth  / (Int)minSUWidth;
	Int  numPartInHeight = (Int)uiMaxCUHeight / (Int)minSUHeight;

	for ( Int i = 0; i < numPartInWidth*numPartInHeight; i++ )
	{
		g_motionRefer[i] = i;
	}

	UInt maxCUDepth = 3 - ( g_uiAddCUDepth - 1);
	Int  minCUWidth  = (Int)uiMaxCUWidth  >> ( (Int)maxCUDepth - 1);

	if(!(minCUWidth == 8 && minSUWidth == 4)) //check if Minimum PU width == 4
		return;

	Int compressionNum = 2;

	for ( Int i = numPartInWidth*(numPartInHeight-1); i < numPartInWidth*numPartInHeight; i += compressionNum*2)
	{
		for ( Int j = 1; j < compressionNum; j++ )
		{
			g_motionRefer[g_auiRasterToZscan[i+j]] = g_auiRasterToZscan[i];
		}
	}

	for ( Int i = numPartInWidth*(numPartInHeight-1)+compressionNum*2-1; i < numPartInWidth*numPartInHeight; i += compressionNum*2)
	{
		for ( Int j = 1; j < compressionNum; j++ )
		{
			g_motionRefer[g_auiRasterToZscan[i-j]] = g_auiRasterToZscan[i];
		}
	}
}

void xFramePadding(MTC265_t *h, MTC265_Frame *pFrm, UInt8 *buf, Pxl *ptr)
{
	UInt16 uiWidthY     = h->usWidth;
	UInt16 uiHeightY    = h->usHeight;
    UInt16 uiWidthC     = h->usWidth/2;
    UInt16 uiHeightC    = h->usHeight/2;
	UInt16 PAD_YSIZEw   = h->PAD_YSIZEw;
	UInt16 PAD_YSIZEh   = h->PAD_YSIZEh;
	UInt16 PAD_CSIZEw   = h->PAD_CSIZEw;
	UInt16 PAD_CSIZEh   = h->PAD_CSIZEh;

    UInt    nStrideY    = uiWidthY+PAD_YSIZEw;
    UInt    nStrideC    = nStrideY>>1;

	const UInt32 uiWidth    = h->usWidth + h->PAD_YSIZEw;
	const UInt32 uiHeight   = h->usHeight + h->PAD_YSIZEh;
	const UInt32 uiYSize    = uiWidth * uiHeight;
	pFrm->pucY = (Pxl *)ptr;
	pFrm->pucU = (Pxl *)ptr + uiYSize;
	pFrm->pucV = (Pxl *)ptr + uiYSize * 5 / 4;

	UInt8  *buf0 = buf;
	
    Pxl  *pucY        = pFrm->pucY;
    Pxl  *pucU        = pFrm->pucU;
    Pxl  *pucV        = pFrm->pucV;
	Int i,j;

	for( i=0; i<uiWidthY *uiHeightY; i++ ) {
		*( ptr+i ) = (Pxl)*( buf+i );
	}
	for( i=0; i<uiWidthY *uiHeightY/4; i++ ) {
		*( ptr+uiWidth*uiHeight+i ) = *( buf+uiWidthY*uiHeightY+i );
		*( ptr+uiWidth*uiHeight*5/4+i ) = *( buf+uiWidth*uiHeight*5/4+i );
	}
	
 	 pucY =  pucY +(uiHeightY-1)* nStrideY;
	 pucU =  pucU +(uiHeightY/2-1)* nStrideC;
	 pucV =  pucV +(uiHeightY/2-1)* nStrideC;

	  for( i=0; i<PAD_YSIZEh; i++ ) {
		for( j=0; j<nStrideY; j++ ) {
			*( pucY + (i+1) * nStrideY + j ) = *( pucY+j);
		}
    }
    for( i=0; i<PAD_CSIZEh; i++ ) {
		for( j=0; j<nStrideC; j++ ) {
			*( pucU + (i+1) * nStrideC + j ) = *( pucU+j);
			*( pucV + (i+1) * nStrideC + j ) = *( pucV+j);
		}
    }
}

void xCalculateAddPSNR(MTC265_t *h,UInt nFrame)
{
	MTC265_Frame *pOrigFrame = h->pFrameCur;
	MTC265_Frame *pRecFrame  = h->pFrameRec;
	UInt16  usWidth  = h->usWidth;
	UInt16  usHeight = h->usHeight;
	Pxl *pucSY  = pOrigFrame->pucY ;
	Pxl *pucSU  = pOrigFrame->pucU ;
	Pxl *pucSV  = pOrigFrame->pucV ;
	Pxl *pucDY  = pRecFrame->pucY ;
	Pxl *pucDU  = pRecFrame->pucU ;
	Pxl *pucDV  = pRecFrame->pucV ;
	UInt64 uiSSDY  = 0;
    UInt64 uiSSDU  = 0;
    UInt64 uiSSDV  = 0;
	double  dYPSNR  = 0.0;
    double  dUPSNR  = 0.0;
    double  dVPSNR  = 0.0;
	//求YPSNR
		 for(UInt16 y = 0; y < usHeight; y++ )
	  {
		for(UInt16 x = 0; x < usWidth; x++ )
		{
		  Int iDiff = (Int)(pucSY[x] - pucDY[x] );
		  uiSSDY   += iDiff * iDiff;
		}
		pucSY += usWidth;
		pucDY += usWidth;
	  }
   //求UPSNR
      for(UInt16 y = 0; y < usHeight/2; y++ )
	  {
		for(UInt16 x = 0; x < usWidth/2; x++ )
		{
		  Int iDiff = (Int)(pucSU[x] - pucDU[x] );
		  uiSSDU   += iDiff * iDiff;
		}
		pucSU += usWidth/2;
		pucDU += usWidth/2;
	  }
   //求VPSNR
	   for(UInt16 y = 0; y < usHeight/2; y++ )
	  {
		for(UInt16 x = 0; x < usWidth/2; x++ )
		{
		  Int iDiff = (Int)(pucSV[x] - pucDV[x] );
		  uiSSDV   += iDiff * iDiff;
		}
		pucSV += usWidth/2;
		pucDV += usWidth/2;
	  }
	  double fRefValueY = (double) 255 *255 * usWidth *usHeight;
	  double fRefValueC = fRefValueY / 4.0;
	  dYPSNR            = ( uiSSDY ? 10.0 * log10( fRefValueY / (double)uiSSDY ) : 99.99 );
	  dUPSNR            = ( uiSSDU ? 10.0 * log10( fRefValueC / (double)uiSSDU ) : 99.99 );
	  dVPSNR            = ( uiSSDV ? 10.0 * log10( fRefValueC / (double)uiSSDV ) : 99.99 );
	  h->FpsnrY[nFrame] = dYPSNR;
	  h->FpsnrU[nFrame] = dUPSNR;
	  h->FpsnrV[nFrame] = dVPSNR;
	  printf( " Y-PSNR    "  "U-PSNR    "  "V-PSNR \n" );
      printf( "%8.4lf  "   "%8.4lf  "    "%8.4lf\n",
            dYPSNR,dUPSNR,dVPSNR  );
}