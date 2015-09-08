#include "MTC265.h"

extern UInt g_auiRasterToPelX  [ MAX_PU_XY*MAX_PU_XY ]; 
extern UInt g_auiRasterToPelY  [ MAX_PU_XY*MAX_PU_XY ];
extern UInt g_auiRasterToZscan [MAX_PU_XY*MAX_PU_XY]; 
extern UInt g_auiZscanToRaster [MAX_PU_XY*MAX_PU_XY]; 

static inline bool isZeroCol( Int addr, Int numUnitsPerRow ) //求相邻块预测模式所用
{
    return ( addr & ( numUnitsPerRow - 1 ) ) == 0;
}

static inline bool isZeroRow( Int addr, Int numUnitsPerRow )
{
    return ( addr &~ ( numUnitsPerRow - 1 ) ) == 0;
}

UInt32 countNonZeroCoeffs( Int16 *psCoef, UInt nStride, UInt nSize )
{
    Int count = 0;
    UInt x, y;

    for( y=0; y<nSize; y++ )
	{
        for( x=0; x<nSize; x++ ) 
		{
            count += (psCoef[y*nStride+x] != 0);
        }
    }

    return count;
}

UInt getCtxQtCbf( UInt bLuma )
{
    if( bLuma ) 
	{
        return 1;
    }
    else 
	{
        return 0;
    }
}

void codeSplitFlag( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx, UInt uiDepth )
{
	UInt8 *puhDepth;
	UInt8 *FrameDepth;
    if (h->isIntra)
	{
		puhDepth = pCache->TemppuhDepth;
	    FrameDepth = pCache->FrameDepth;
	}
	else
	{
		puhDepth = pCache->BestpuhInterDepth;
		FrameDepth = h->FrameBestDepth;
	}

	UInt g_uiMaxCUDepth = h->ucMaxCUDepth;
	Int NumPartition = pCache->NumPartition;
	Int NumPartInCUWidth = pCache->NumPartitionInWidth;
    UInt uiLPartUnitIdx,uiAPartUnitIdx;
	bool pcTempCU_L,pcTempCU_T;

	if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
		return;

	UInt temp = g_auiZscanToRaster[uiAbsPartIdx];
	if( (!isZeroCol( temp, NumPartInCUWidth )) )
	{
		uiLPartUnitIdx = g_auiRasterToZscan[ temp - 1 ];
	}
	else
	{
		uiLPartUnitIdx = g_auiRasterToZscan[ temp + NumPartInCUWidth - 1 ];
	}
	if( (!isZeroRow( temp, NumPartInCUWidth )) )
	{
		uiAPartUnitIdx = g_auiRasterToZscan[ temp - NumPartInCUWidth ];
	}
	else
	{
		uiAPartUnitIdx = g_auiRasterToZscan[ temp + NumPartition - NumPartInCUWidth ];
	}  
  
	UInt8 uiDepthLeft, uiDepthTop;
	if( h->uiCUX == 0 && h->uiCUY == 0 )
	{
		if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
		{
			pcTempCU_L = 0;
			pcTempCU_T = 0;
			uiDepthLeft = 2;
			uiDepthTop = 2;
		}
		else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
		{
			pcTempCU_L = 1;
			pcTempCU_T = 0;
			uiDepthLeft = puhDepth[uiLPartUnitIdx];
			uiDepthTop = 2;
		}
		else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
		{
			pcTempCU_L = 0;
			pcTempCU_T = 1;
			uiDepthLeft = 2;
			uiDepthTop = puhDepth[uiAPartUnitIdx];
		}
		else
		{
			pcTempCU_L = 1;
			pcTempCU_T = 1;
			uiDepthLeft = puhDepth[uiLPartUnitIdx];
			uiDepthTop = puhDepth[uiAPartUnitIdx];
		}
	}
	else if( h->uiCUX != 0 && h->uiCUY == 0 )
	{
		if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
		{
			pcTempCU_L = 1;
			pcTempCU_T = 0;
			uiDepthLeft = FrameDepth[uiLPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)-1)*NumPartition];
			uiDepthTop = 2;
		}
		else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
		{
			pcTempCU_L = 1;
			pcTempCU_T = 0;
			uiDepthLeft = puhDepth[uiLPartUnitIdx];
			uiDepthTop = 2;
		}
		else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
		{
			pcTempCU_L = 1;
			pcTempCU_T = 1;
			uiDepthLeft = FrameDepth[uiLPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)-1)*NumPartition];
			uiDepthTop = puhDepth[uiAPartUnitIdx];
		}
		else
		{
			pcTempCU_L = 1;
			pcTempCU_T = 1;
			uiDepthLeft = puhDepth[uiLPartUnitIdx];
			uiDepthTop = puhDepth[uiAPartUnitIdx];
		}
	}
	else if( h->uiCUX == 0 && h->uiCUY != 0 )
	{
		if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
		{
			pcTempCU_L = 0;
			pcTempCU_T = 1;
			uiDepthLeft = 2;
			uiDepthTop = FrameDepth[uiAPartUnitIdx+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth)*NumPartition];
		}
		else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
		{
			pcTempCU_L = 1;
			pcTempCU_T = 1;
			uiDepthLeft = puhDepth[uiLPartUnitIdx];
			uiDepthTop = FrameDepth[uiAPartUnitIdx+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth)*NumPartition];
		}
		else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
		{
			pcTempCU_L = 0;
			pcTempCU_T = 1;
			uiDepthLeft = 2;
			uiDepthTop = puhDepth[uiAPartUnitIdx];
		}
		else
		{
			pcTempCU_L = 1;
			pcTempCU_T = 1;
			uiDepthLeft = puhDepth[uiLPartUnitIdx];
			uiDepthTop = puhDepth[uiAPartUnitIdx];
		}
	}
	else
	{
		pcTempCU_L = 1;
		pcTempCU_T = 1;
		if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
		{
			uiDepthLeft = FrameDepth[uiLPartUnitIdx+(((h->uiCUX/h->ucMaxCUWidth)-1)+(h->uiCUY/h->ucMaxCUWidth)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
			uiDepthTop = FrameDepth[uiAPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
		}
		else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
		{
			uiDepthLeft = puhDepth[uiLPartUnitIdx];
			uiDepthTop = FrameDepth[uiAPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
		}
		else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
		{
			uiDepthLeft = FrameDepth[uiLPartUnitIdx+(((h->uiCUX/h->ucMaxCUWidth)-1)+(h->uiCUY/h->ucMaxCUWidth)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
			uiDepthTop = puhDepth[uiAPartUnitIdx];
		}
		else
		{
			uiDepthLeft = puhDepth[uiLPartUnitIdx];
			uiDepthTop = puhDepth[uiAPartUnitIdx];
		}
	}

	UInt uiCtx           = (( pcTempCU_L ) ? ( ( uiDepthLeft > uiDepth ) ? 1 : 0 ) : 0) + (( pcTempCU_T ) ? ( ( uiDepthTop > uiDepth ) ? 1 : 0 ) : 0);
	UInt uiCurrSplitFlag = ( puhDepth[uiAbsPartIdx] > uiDepth ) ? 1 : 0;

	assert( uiCtx < 3 );
	xCabacEncodeBin( pCabac, pBS, uiCurrSplitFlag, uiCtx );
	return;
}

void codePredMode( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx )
{
  // get context function
  if ( h->isIntra )
  {
    return;
  }
  //for all inter LCUs use inter mode
  xCabacEncodeBin( pCabac, pBS, 0, OFF_PRED_MODE_CTX );
}

void codePartSize( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx, UInt uiDepth )
{
	UInt8 eSize;
	if (h->isIntra)
	{
		eSize = pCache->TemppePartSize[uiAbsPartIdx];
	}
	else
	{
		eSize = pCache->BestpuhInterPartSize[uiAbsPartIdx];
	}

	UInt g_uiMaxCUDepth = h->ucMaxCUDepth;
	if ( h->isIntra )
	{
		if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
		{
			xCabacEncodeBin( pCabac, pBS, eSize == SIZE_2Nx2N? 1 : 0, OFF_PART_SIZE_CTX );
		}
		return;
	}

	switch(eSize)
	{
	case SIZE_2Nx2N:
		{
			xCabacEncodeBin( pCabac, pBS, 1, OFF_PART_SIZE_CTX );
			break;
		}	
	case SIZE_2NxN:
		{
			xCabacEncodeBin( pCabac, pBS, 0, OFF_PART_SIZE_CTX );
			xCabacEncodeBin( pCabac, pBS, 1, OFF_PART_SIZE_CTX+1 );
			break;
		}
	case SIZE_Nx2N:
		{
			xCabacEncodeBin( pCabac, pBS, 0, OFF_PART_SIZE_CTX );
			xCabacEncodeBin(  pCabac, pBS, 0, OFF_PART_SIZE_CTX+1 );
			break;
		}
	case SIZE_NxN:
		{
			break;
		}
	default:
		{
			assert(0);
		}
	}
}

void codeIntraDirLumaAng( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx )
{
	UInt8 mode = pCache->TemppePartSize[uiAbsPartIdx];
	UInt partNum = mode==SIZE_NxN?4:1;
	UInt8 *nModeY = pCache->TemppuhLumaIntraDir;
	Int iPredIdx[4] ={ -1,-1,-1,-1};
	
	Int NumPartInCUWidth = pCache->NumPartitionInWidth;
	Int NumPartition = pCache->NumPartition;
	UInt uiLPartUnitIdx,uiAPartUnitIdx;
	Int iLeftIntraDir, iAboveIntraDir;
	bool pcTempCU_L,pcTempCU_T;
	UInt8 nBestMode[4][3] = {{-1, -1, -1},{-1, -1, -1},{-1, -1, -1},{-1, -1, -1}};
	UInt8* FrameModeY = pCache->FrameModeY;

	for (UInt j=0;j<partNum;j++)
	{
		UInt temp = g_auiZscanToRaster[uiAbsPartIdx+j];
        if( (!isZeroCol( temp, NumPartInCUWidth )) )
        {
	        uiLPartUnitIdx = g_auiRasterToZscan[ temp - 1 ];
        }
        else
        {
            uiLPartUnitIdx = g_auiRasterToZscan[ temp + NumPartInCUWidth - 1 ];
        }
        if( (!isZeroRow( temp, NumPartInCUWidth )) )
        {
            uiAPartUnitIdx = g_auiRasterToZscan[ temp - NumPartInCUWidth ];
        }
        else
        {
            uiAPartUnitIdx = g_auiRasterToZscan[ temp + NumPartition - NumPartInCUWidth ];
        }
		if( h->uiCUX == 0 && h->uiCUY == 0 )
        {
	        if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
	        {
	            pcTempCU_L = 0;
                pcTempCU_T = 0;
		        iLeftIntraDir = -1;
		        iAboveIntraDir = -1;
	        }
	        else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
			{
				pcTempCU_L = 1;
				pcTempCU_T = 0;
				iLeftIntraDir = nModeY[uiLPartUnitIdx];
				iAboveIntraDir = -1;
			}
			else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
			{
				pcTempCU_L = 0;
				pcTempCU_T = 1;
				iLeftIntraDir = -1;
			    iAboveIntraDir = nModeY[uiAPartUnitIdx];
			}
			else
			{
				pcTempCU_L = 1;
				pcTempCU_T = 1;
				iLeftIntraDir = nModeY[uiLPartUnitIdx];
				iAboveIntraDir = nModeY[uiAPartUnitIdx];
			}
		}
		else if( h->uiCUX != 0 && h->uiCUY == 0 )
		{
			if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
			{
				pcTempCU_L = 1;
				pcTempCU_T = 0;
				iLeftIntraDir = FrameModeY[uiLPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)-1)*NumPartition];
				iAboveIntraDir = -1;
			}
			else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
			{
				pcTempCU_L = 1;
				pcTempCU_T = 0;
				iLeftIntraDir = nModeY[uiLPartUnitIdx];
				iAboveIntraDir = -1;
			}
			else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
			{
				pcTempCU_L = 1;
				pcTempCU_T = 1;
				iLeftIntraDir = FrameModeY[uiLPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)-1)*NumPartition];
				iAboveIntraDir = nModeY[uiAPartUnitIdx];
			}
			else
			{
				pcTempCU_L = 1;
				pcTempCU_T = 1;
				iLeftIntraDir = nModeY[uiLPartUnitIdx];
				iAboveIntraDir = nModeY[uiAPartUnitIdx];
			}
		}
		else if( h->uiCUX == 0 && h->uiCUY != 0 )
		{
			if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
			{
				pcTempCU_L = 0;
				pcTempCU_T = 0;
				iLeftIntraDir = -1;
				iAboveIntraDir = FrameModeY[uiAPartUnitIdx+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth)*NumPartition];
			}
			else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
			{
				pcTempCU_L = 1;
				pcTempCU_T = 0;
				iLeftIntraDir = nModeY[uiLPartUnitIdx];
				iAboveIntraDir = FrameModeY[uiAPartUnitIdx+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth)*NumPartition];
			}
			else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
			{
				pcTempCU_L = 0;
				pcTempCU_T = 1;
				iLeftIntraDir = -1;
				iAboveIntraDir = nModeY[uiAPartUnitIdx];
			}
			else
			{
				pcTempCU_L = 1;
				pcTempCU_T = 1;
				iLeftIntraDir = nModeY[uiLPartUnitIdx];
				iAboveIntraDir = nModeY[uiAPartUnitIdx];
			}
		}
		else
		{
			pcTempCU_L = 1;
			pcTempCU_T = 1;
			if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
			{
				pcTempCU_L = 1;
				pcTempCU_T = 0;
				iLeftIntraDir = FrameModeY[uiLPartUnitIdx+(((h->uiCUX/h->ucMaxCUWidth)-1)+(h->uiCUY/h->ucMaxCUWidth)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
				iAboveIntraDir = FrameModeY[uiAPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
			}
			else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
			{
				pcTempCU_L = 1;
				pcTempCU_T = 0;
				iLeftIntraDir = nModeY[uiLPartUnitIdx];
				iAboveIntraDir = FrameModeY[uiAPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
			}
			else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
			{
				iLeftIntraDir = FrameModeY[uiLPartUnitIdx+(((h->uiCUX/h->ucMaxCUWidth)-1)+(h->uiCUY/h->ucMaxCUWidth)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
				iAboveIntraDir = nModeY[uiAPartUnitIdx];
			}
			else
			{
				iLeftIntraDir = nModeY[uiLPartUnitIdx];
				iAboveIntraDir = nModeY[uiAPartUnitIdx];
			}
		}
		iLeftIntraDir = pcTempCU_L ? iLeftIntraDir : DC_IDX;
		iAboveIntraDir = pcTempCU_T ? iAboveIntraDir : DC_IDX;
	
		if(iLeftIntraDir == iAboveIntraDir)
        {    
            if (iLeftIntraDir > 1) // angular modes
            {
                nBestMode[j][0]  = iLeftIntraDir;
                nBestMode[j][1] = ((iLeftIntraDir + 29) % 32) + 2;
                nBestMode[j][2] = ((iLeftIntraDir - 1 ) % 32) + 2;
            }
            else //non-angular
            {
                nBestMode[j][0] = PLANAR_IDX;
                nBestMode[j][1] = DC_IDX;
                nBestMode[j][2] = VER_IDX; 
            }
        }
        else
        {
            nBestMode[j][0] = iLeftIntraDir;
            nBestMode[j][1] = iAboveIntraDir;
            if (iLeftIntraDir && iAboveIntraDir ) //both modes are non-planar
            {
                nBestMode[j][2] = PLANAR_IDX;
            }
            else
            {
                nBestMode[j][2] =  (iLeftIntraDir+iAboveIntraDir)<2? VER_IDX : DC_IDX;
            }
        }
	
		if( nModeY[uiAbsPartIdx+j] == nBestMode[j][0])
			iPredIdx[j] = 0;
		else if( nModeY[uiAbsPartIdx+j] == nBestMode[j][1] )
			iPredIdx[j]= 1;
		else if( nModeY[uiAbsPartIdx+j] == nBestMode[j][2] )
			iPredIdx[j] = 2;
		else
			iPredIdx[j] = -1;
	
		xCabacEncodeBin( pCabac, pBS, (iPredIdx[j] != -1), OFF_INTRA_PRED_CTX );
	}
    for (UInt j=0;j<partNum;j++)
	{
		if( iPredIdx[j] != -1 ) 
		{
			// 0 -> 0
			// 1 -> 10
			// 2 -> 11
			if( iPredIdx[j] == 0 )
				xCabacEncodeBinEP( pCabac, pBS, 0 );
			else
				xCabacEncodeBinsEP( pCabac, pBS, iPredIdx[j]+1, 2 );
		}
		else 
		{
			// Sort MostModeY
			UInt8 tmp;
			if( nBestMode[j][0] > nBestMode[j][1] ) 
			{
				tmp = nBestMode[j][0];
				nBestMode[j][0] = nBestMode[j][1];
				nBestMode[j][1] = tmp;
			}
			if( nBestMode[j][0] > nBestMode[j][2] ) 
			{
				tmp = nBestMode[j][0];
				nBestMode[j][0] = nBestMode[j][2];
				nBestMode[j][2] = tmp;
			}
			if( nBestMode[j][1] > nBestMode[j][2] ) 
			{
				tmp = nBestMode[j][1];
				nBestMode[j][1] = nBestMode[j][2];
				nBestMode[j][2] = tmp;
			}
			UInt8 nModeYTemp = nModeY[uiAbsPartIdx+j];
			if(nModeY[uiAbsPartIdx+j] > nBestMode[j][2])
				nModeYTemp--;
			if(nModeY[uiAbsPartIdx+j] > nBestMode[j][1])
				nModeYTemp--;
			if(nModeY[uiAbsPartIdx+j] > nBestMode[j][0])
				nModeYTemp--;
			xCabacEncodeBinsEP( pCabac, pBS, nModeYTemp, 5 );
		}
	}
}

void codeIntraDirChroma( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx )
{
	UInt8 *nModeC = pCache->TemppuhChromaIntraDir;
	xCabacEncodeBin( pCabac, pBS, (nModeC[uiAbsPartIdx] != NUM_CHROMA_MODE - 1), OFF_CHROMA_PRED_CTX );
	if ( nModeC[uiAbsPartIdx] != NUM_CHROMA_MODE - 1 ) 
	{
		if ( h->bUseLMChroma )
			xCabacEncodeBin( pCabac, pBS, (nModeC[uiAbsPartIdx] != NUM_CHROMA_MODE - 2), OFF_CHROMA_PRED_CTX+1 );
		// Non DM_CHROMA_IDX and LM_CHROMA_IDX
		if( nModeC[uiAbsPartIdx] < NUM_CHROMA_MODE - 2 ) 
		{
			xCabacEncodeBinsEP( pCabac, pBS, nModeC[uiAbsPartIdx], 2 );
		}
	}
}

void codeSkipFlag( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx )
{
  // get context function
  Int32* SkipFlag = pCache->BestpuhSkipFlag;
  Int32* FrameSkipFlag = h->FrameBestSkipFlag;
  
  Int NumPartition = pCache->NumPartition;
  Int NumPartInCUWidth = pCache->NumPartitionInWidth;
  UInt uiLPartUnitIdx,uiAPartUnitIdx;
  bool pcTempCU_L,pcTempCU_T;
  UInt temp = g_auiZscanToRaster[uiAbsPartIdx];

  if( (!isZeroCol( temp, NumPartInCUWidth )) )
  {
	  uiLPartUnitIdx = g_auiRasterToZscan[ temp - 1 ];
  }
  else
  {
	  uiLPartUnitIdx = g_auiRasterToZscan[ temp + NumPartInCUWidth - 1 ];
  }
  if( (!isZeroRow( temp, NumPartInCUWidth )) )
  {
	  uiAPartUnitIdx = g_auiRasterToZscan[ temp - NumPartInCUWidth ];
  }
  else
  {
	  uiAPartUnitIdx = g_auiRasterToZscan[ temp + NumPartition - NumPartInCUWidth ];
  }  

  UInt8 uiSkipFlagLeft, uiSkipFlagTop;
  if( h->uiCUX == 0 && h->uiCUY == 0 )
  {
	  if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
	  {
		  pcTempCU_L = 0;
		  pcTempCU_T = 0;
		  uiSkipFlagLeft = 2;
		  uiSkipFlagTop = 2;
	  }
	  else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
	  {
		  pcTempCU_L = 1;
		  pcTempCU_T = 0;
		  uiSkipFlagLeft = SkipFlag[uiLPartUnitIdx];
		  uiSkipFlagTop = 2;
	  }
	  else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
	  {
		  pcTempCU_L = 0;
		  pcTempCU_T = 1;
		  uiSkipFlagLeft = 2;
		  uiSkipFlagTop = SkipFlag[uiAPartUnitIdx];
	  }
	  else
	  {
		  pcTempCU_L = 1;
		  pcTempCU_T = 1;
		  uiSkipFlagLeft = SkipFlag[uiLPartUnitIdx];
		  uiSkipFlagTop = SkipFlag[uiAPartUnitIdx];
	  }
  }
  else if( h->uiCUX != 0 && h->uiCUY == 0 )
  {
	  if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
	  {
		  pcTempCU_L = 1;
		  pcTempCU_T = 0;
		  uiSkipFlagLeft = FrameSkipFlag[uiLPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)-1)*NumPartition];
		  uiSkipFlagTop = 2;
	  }
	  else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
	  {
		  pcTempCU_L = 1;
		  pcTempCU_T = 0;
		  uiSkipFlagLeft = SkipFlag[uiLPartUnitIdx];
		  uiSkipFlagTop = 2;
	  }
	  else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
	  {
		  pcTempCU_L = 1;
		  pcTempCU_T = 1;
		  uiSkipFlagLeft = FrameSkipFlag[uiLPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)-1)*NumPartition];
		  uiSkipFlagTop = SkipFlag[uiAPartUnitIdx];
	  }
	  else
	  {
		  pcTempCU_L = 1;
		  pcTempCU_T = 1;
		  uiSkipFlagLeft = SkipFlag[uiLPartUnitIdx];
		  uiSkipFlagTop = SkipFlag[uiAPartUnitIdx];
	  }
  }
  else if( h->uiCUX == 0 && h->uiCUY != 0 )
  {
	  if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
	  {
		  pcTempCU_L = 0;
		  pcTempCU_T = 1;
		  uiSkipFlagLeft = 2;
		  uiSkipFlagTop = FrameSkipFlag[uiAPartUnitIdx+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth)*NumPartition];
	  }
	  else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
	  {
		  pcTempCU_L = 1;
		  pcTempCU_T = 1;
		  uiSkipFlagLeft = SkipFlag[uiLPartUnitIdx];
		  uiSkipFlagTop = FrameSkipFlag[uiAPartUnitIdx+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth)*NumPartition];
	  }
	  else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
	  {
		  pcTempCU_L = 0;
		  pcTempCU_T = 1;
		  uiSkipFlagLeft = 2;
		  uiSkipFlagTop = SkipFlag[uiAPartUnitIdx];
	  }
	  else
	  {
		  pcTempCU_L = 1;
		  pcTempCU_T = 1;
		  uiSkipFlagLeft = SkipFlag[uiLPartUnitIdx];
		  uiSkipFlagTop = SkipFlag[uiAPartUnitIdx];
	  }
  }
  else
  {
	  pcTempCU_L = 1;
	  pcTempCU_T = 1;
	  if( isZeroCol( temp, NumPartInCUWidth ) && isZeroRow( temp, NumPartInCUWidth ) )
	  {
		  uiSkipFlagLeft = FrameSkipFlag[uiLPartUnitIdx+(((h->uiCUX/h->ucMaxCUWidth)-1)+(h->uiCUY/h->ucMaxCUWidth)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
		  uiSkipFlagTop = FrameSkipFlag[uiAPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
	  }
	  else if( (!isZeroCol( temp, NumPartInCUWidth )) && isZeroRow( temp, NumPartInCUWidth ) )
	  {
		  uiSkipFlagLeft = SkipFlag[uiLPartUnitIdx];
		  uiSkipFlagTop = FrameSkipFlag[uiAPartUnitIdx+((h->uiCUX/h->ucMaxCUWidth)+((h->uiCUY/h->ucMaxCUWidth)-1)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
	  }
	  else if( isZeroCol( temp, NumPartInCUWidth ) && (!isZeroRow( temp, NumPartInCUWidth )) )
	  {
		  uiSkipFlagLeft = FrameSkipFlag[uiLPartUnitIdx+(((h->uiCUX/h->ucMaxCUWidth)-1)+(h->uiCUY/h->ucMaxCUWidth)*(h->usWidth/h->ucMaxCUWidth))*NumPartition];
		  uiSkipFlagTop = SkipFlag[uiAPartUnitIdx];
	  }
	  else
	  {
		  uiSkipFlagLeft = SkipFlag[uiLPartUnitIdx];
		  uiSkipFlagTop = SkipFlag[uiAPartUnitIdx];
	  }
  }
  UInt uiCtxSkip = ((pcTempCU_L) ? uiSkipFlagLeft : 0) + ((pcTempCU_T) ? uiSkipFlagTop : 0);

  xCabacEncodeBin( pCabac, pBS, SkipFlag[uiAbsPartIdx], OFF_SKIP_FLAG_CTX + uiCtxSkip );
}

void codeMergeIndex( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx )
{
	UInt uiNumCand = MRG_MAX_NUM_CANDS;
	UInt uiUnaryIdx = pCache->BestpuhMergeIdx[uiAbsPartIdx];
	if ( uiNumCand > 1 )
	{
		for( UInt ui = 0; ui < uiNumCand - 1; ++ui )
		{
			const UInt uiSymbol = ui == uiUnaryIdx ? 0 : 1;
			if ( ui==0 )
			{
				xCabacEncodeBin( pCabac, pBS, uiSymbol, OFF_MERGE_IDX_EXT_CTX );
			}
			else
			{
				xCabacEncodeBinEP( pCabac, pBS, uiSymbol );
			}
			if( uiSymbol == 0 )
			{
				break;
			}
		}
	}
}

void codeMergeFlag( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx )
{
	const UInt uiSymbol = pCache->BestpuhMergeFlag[uiAbsPartIdx];
	xCabacEncodeBin( pCabac, pBS, uiSymbol, OFF_MERGE_FLAG_EXT_CTX );
}

void codeInterDir( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx )
{
	if( h->eSliceType != SLICE_B )
	{
		return;
	}
}

void codeRefFrmIdx( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx, UInt NumRefIdx )
{
	if ( NumRefIdx  == 1 ) 
	{
		return;
	}
}

void codeMvd( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx  )
{
	MVpara ParaMvd = pCache->BestpuhMvd[uiAbsPartIdx];

	const Int iHor = ParaMvd.m_x ;
	const Int iVer = ParaMvd.m_y ;

	xCabacEncodeBin( pCabac, pBS, iHor != 0 ? 1 : 0, OFF_MVD_CTX );
	xCabacEncodeBin( pCabac, pBS, iVer != 0 ? 1 : 0, OFF_MVD_CTX );

	const bool bHorAbsGr0 = iHor != 0;
	const bool bVerAbsGr0 = iVer != 0;
	const UInt uiHorAbs   = 0 > iHor ? -iHor : iHor;
	const UInt uiVerAbs   = 0 > iVer ? -iVer : iVer;
	UInt OFF_MVD_CTX_STATE = OFF_MVD_CTX + 1;

	if( bHorAbsGr0 )
	{
		xCabacEncodeBin( pCabac, pBS, uiHorAbs > 1 ? 1 : 0, OFF_MVD_CTX_STATE );
	}

	if( bVerAbsGr0 )
	{
		xCabacEncodeBin( pCabac, pBS, uiVerAbs > 1 ? 1 : 0, OFF_MVD_CTX_STATE );
	}

	if( bHorAbsGr0 )
	{
		if( uiHorAbs > 1 )
		{
			xWriteEpExGolomb( pCabac, pBS, uiHorAbs-2, 1 );
		}

		xCabacEncodeBinEP( pCabac, pBS, 0 > iHor ? 1 : 0 );
	}

	if( bVerAbsGr0 )
	{
		if( uiVerAbs > 1 )
		{
			xWriteEpExGolomb( pCabac, pBS, uiVerAbs-2, 1 );
		}

		xCabacEncodeBinEP( pCabac, pBS, 0 > iVer ? 1 : 0 );
	}

	return;
}

void codeMVPIdx( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx )
{
	Int32 ParaMvpIdx = pCache->BestpuhMvpIdx[uiAbsPartIdx];
	Int iSymbol = ParaMvpIdx;
	xWriteUnaryMaxSymbol( pCabac, pBS, iSymbol, OFF_MVP_IDX_CTX, 1, AMVP_MAX_NUM_CANDS-1 );
}

void codePUWise( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx, UInt uiDepth )
{ 
	UInt8 ePartSize = pCache->BestpuhInterPartSize[uiAbsPartIdx];
	UInt uiNumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
	UInt g_auiPUOffset[8] = { 0, 8, 4, 4, 2, 10, 1, 5};
	UInt uiPUOffset = ( g_auiPUOffset[UInt( ePartSize )] << ( ( h->ucMaxCUDepth - uiDepth ) << 1 ) ) >> 4;

	UInt NumRefIdx = 1;

	for ( UInt uiPartIdx = 0, uiSubPartIdx = uiAbsPartIdx; uiPartIdx < uiNumPU; uiPartIdx++, uiSubPartIdx += uiPUOffset )
	{
		codeMergeFlag( h, pCache, pCabac, pBS, uiSubPartIdx );
		if ( pCache->BestpuhMergeFlag[uiSubPartIdx] )
		{
			codeMergeIndex( h, pCache, pCabac, pBS, uiSubPartIdx );
		}
		else
		{
			codeInterDir( h, pCache, pCabac, pBS, uiSubPartIdx );
			for ( UInt uiRefListIdx = 0; uiRefListIdx < 1; uiRefListIdx++ )
			{
				if ( NumRefIdx > 0 )
				{
					codeRefFrmIdx ( h, pCache, pCabac, pBS, uiSubPartIdx, NumRefIdx );
					codeMvd       ( h, pCache, pCabac, pBS, uiSubPartIdx );
					codeMVPIdx    ( h, pCache, pCabac, pBS, uiSubPartIdx );
				}
			}
		}
	}
	return;
}

void codePredInfo( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, UInt uiAbsPartIdx, UInt uiDepth )
{
	if( h->isIntra )                                 // If it is Intra mode, encode intra prediction mode.
	{
		codeIntraDirLumaAng  ( h, pCache, pCabac, pBS, uiAbsPartIdx );
		codeIntraDirChroma ( h, pCache, pCabac, pBS, uiAbsPartIdx );
	}
	else                                                                // if it is Inter mode, encode motion vector and reference index
	{
		codePUWise( h, pCache, pCabac, pBS, uiAbsPartIdx, uiDepth );
	}
}


UInt getCoefScanIdx( X265_Cache *pCache, UInt nWidth, UInt8 bIsIntra, UInt8 bIsLuma, UInt nMode, UInt uiAbsPartIdx )
{
    UInt uiCTXIdx;
    UInt nScanIdx;
    
    if( !bIsIntra ) 
	{
        return SCAN_ZIGZAG;
    }
    
    switch( nWidth ) 
	{
		case  2: uiCTXIdx = 6; break;
		case  4: uiCTXIdx = 5; break;
		case  8: uiCTXIdx = 4; break;
		case 16: uiCTXIdx = 3; break;
		case 32: uiCTXIdx = 2; break;
		case 64: uiCTXIdx = 1; break;
		default: uiCTXIdx = 0; break;
    }
    
    if( bIsLuma ) 
	{
        nScanIdx = SCAN_ZIGZAG;
        if( uiCTXIdx > 3 && uiCTXIdx < 6 ) 
		{
            //if multiple scans supported for PU size
            nScanIdx = abs((Int) nMode - VER_IDX) < 5 ? 1 : (abs((Int)nMode - HOR_IDX) < 5 ? 2 : 0);
        }
    }
    else 
	{
		if( nMode == 0)
		{
			if( pCache->TemppuhLumaIntraDir[uiAbsPartIdx] == PLANAR_IDX)
			{
				nMode = 34;
			}
		}
		if( nMode == 1)
		{
			nMode = VER_IDX;
			if( pCache->TemppuhLumaIntraDir[uiAbsPartIdx] == VER_IDX)
			{
				nMode = 34;
			}
		}
		if( nMode == 2)
		{
			nMode = HOR_IDX;
			if( pCache->TemppuhLumaIntraDir[uiAbsPartIdx] == HOR_IDX)
			{
				nMode = 34;
			}
		}
		if( nMode == 3)
		{
			nMode = DC_IDX;
			if( pCache->TemppuhLumaIntraDir[uiAbsPartIdx] == DC_IDX)
			{
				nMode = 34;
			}
		}
		if( nMode == 5)
		{
			nMode = pCache->TemppuhLumaIntraDir[uiAbsPartIdx];
		}
		
        nScanIdx = SCAN_ZIGZAG;
        if( uiCTXIdx > 4 && uiCTXIdx < 7 ) 
		{
            //if multiple scans supported for PU size
            nScanIdx = abs((Int) nMode - VER_IDX) < 5 ? 1 : (abs((Int)nMode - HOR_IDX) < 5 ? 2 : 0);
        }
    }
    return nScanIdx;
}

Int getSigCtxInc(Int16 *psCoeff, Int posX, Int posY, Int blockType, Int nSize, UInt8 bIsLuma)
{
    const Int width  = nSize;
    const Int height = nSize;
    const UInt nStride = nSize;
    if( blockType == 2 ) 
	{
        //LUMA map
        const Int ctxIndMap4x4Luma[15] = 
		{
            0, 1, 4, 5,
            2, 3, 4, 5,
            6, 6, 8, 8,
            7, 7, 8
        };
        //CHROMA map
        const Int ctxIndMap4x4Chroma[15] = 
		{
            0, 1, 2, 4,
            1, 1, 2, 4,
            3, 3, 5, 5,
            4, 4, 5
        };
        
        if( bIsLuma ) 
		{
            return ctxIndMap4x4Luma[ 4 * posY + posX ];
        }
        else 
		{
            return ctxIndMap4x4Chroma[ 4 * posY + posX ];
        }
    }
    else if( blockType == 3 ) 
	{
        const Int map8x8[16] = 
		{
            0,  1,  2,  3,
            4,  5,  6,  3,
            8,  6,  6,  7,
            9,  9,  7,  7
        };
        
        Int offset = bIsLuma ? 9 : 6;    
        if( posX + posY == 0 ) 
		{
            return offset + 10;
        }
        else 
		{
            return offset + map8x8[4 * (posY >> 1) + (posX >> 1)];
        }
    }
    
    Int offset = bIsLuma ? 20 : 17;
    if( posX + posY == 0 ) 
	{
        return offset;
    }
    Int thredHighFreq = 3*(MAX(width, height)>>4);
    if( (posX>>2) + (posY>>2) >= thredHighFreq ) 
	{
        return bIsLuma ? 24 : 18;
    }
    
    const Int16 *pData = psCoeff + posY * nStride + posX;
    
    Int cnt = 0;
    if( posX < width - 1 ) 
	{
        cnt += (pData[1] != 0);
        if( posY < height - 1 ) 
		{
            cnt += (pData[nStride+1] != 0);
        }
        if( posX < width - 2 ) 
		{
            cnt += (pData[2] != 0);
        }
    }
    if( posY < height - 1 ) 
	{
        if( ( ( posX & 3 ) || ( posY & 3 ) ) && ( ( (posX+1) & 3 ) || ( (posY+2) & 3 ) ) ) {
            cnt += pData[nStride] != 0;
        }
        if( posY < height - 2 && cnt < 4 ) 
		{
            cnt += pData[2*nStride] != 0;
        }
    }
    
    cnt = ( cnt + 1 ) >> 1;
    return (( bIsLuma && ((posX>>2) + (posY>>2)) > 0 ) ? 4 : 1) + offset + cnt;
}

UInt getSigCoeffGroupCtxInc(const UInt8 *uiSigCoeffGroupFlag, const UInt nCGPosX, const UInt nCGPosY, const UInt scanIdx, UInt nSize)
{
    UInt uiRight = 0;
    UInt uiLower = 0;
    UInt width = nSize;
    UInt height = nSize;
    
    width >>= 2;
    height >>= 2;
    // 8x8
    if( nSize == 8 ) {
        if( scanIdx == SCAN_HOR ) 
		{
            width = 1;
            height = 4;
        }
        else if( scanIdx == SCAN_VER ) 
		{
            width = 4;
            height = 1;
        }
    }
    if( nCGPosX < width - 1 ) 
	{
        uiRight = (uiSigCoeffGroupFlag[ nCGPosY * width + nCGPosX + 1 ] != 0);
    }
    if( nCGPosY < height - 1 ) 
	{
        uiLower = (uiSigCoeffGroupFlag[ (nCGPosY  + 1 ) * width + nCGPosX ] != 0);
    }
    return (uiRight || uiLower);
}

void codeLastSignificantXY( X265_Cabac *pCabac, X265_BitStream *pBS, UInt nPosX, UInt nPosY, UInt nSize, UInt8 bIsLuma, UInt nScanIdx )
{
    const UInt nLog2Size = xLog2( nSize - 1 );

    // swap
    if( nScanIdx == SCAN_VER ) 
	{
		UInt tmp = nPosY;
        nPosY = nPosX;
        nPosX = tmp;
    }

    UInt nCtxLast;
    UInt nCtxX = OFF_LAST_X_CTX + (bIsLuma ? 0 : NUM_LAST_FLAG_XY_CTX);
    UInt nCtxY = OFF_LAST_Y_CTX + (bIsLuma ? 0 : NUM_LAST_FLAG_XY_CTX);
    UInt uiGroupIdxX    = g_uiGroupIdx[ nPosX ];
    UInt uiGroupIdxY    = g_uiGroupIdx[ nPosY ];
    
    // posX
    Int iLog2WidthCtx = bIsLuma ? nLog2Size-2 : 0;
    const UInt8 *puiCtxIdxX = g_uiLastCtx + ( iLog2WidthCtx * ( iLog2WidthCtx + 3 ) );
    for( nCtxLast = 0; nCtxLast < uiGroupIdxX; nCtxLast++ ) 
	{
        xCabacEncodeBin( pCabac, pBS, 1, nCtxX + ( bIsLuma ? puiCtxIdxX[nCtxLast] : (nCtxLast>>(nLog2Size-2)) ) );
    }
    if( uiGroupIdxX < g_uiGroupIdx[nSize-1]) 
	{
        xCabacEncodeBin( pCabac, pBS, 0, nCtxX + ( bIsLuma ? puiCtxIdxX[nCtxLast] : (nCtxLast>>(nLog2Size-2)) ) );
    }
    
    // posY
    Int iLog2HeightCtx = bIsLuma ? nLog2Size-2 : 0;
    const UInt8 *puiCtxIdxY = g_uiLastCtx + ( iLog2WidthCtx * ( iLog2WidthCtx + 3 ) );
    for( nCtxLast = 0; nCtxLast < uiGroupIdxY; nCtxLast++ ) 
	{
        xCabacEncodeBin( pCabac, pBS, 1, nCtxY + ( bIsLuma ? puiCtxIdxY[nCtxLast] : (nCtxLast>>(nLog2Size-2)) ) );
    }
    if( uiGroupIdxY < g_uiGroupIdx[ nSize - 1 ]) 
	{
        xCabacEncodeBin( pCabac, pBS, 0, nCtxY + ( bIsLuma ? puiCtxIdxY[nCtxLast] : (nCtxLast>>(nLog2Size-2)) ) );
    }

    if( uiGroupIdxX > 3 ) 
	{      
        UInt nCount = ( uiGroupIdxX - 2 ) >> 1;
        nPosX       = nPosX - g_uiMinInGroup[ uiGroupIdxX ];
        xCabacEncodeBinsEP( pCabac, pBS, nPosX, nCount );
    }
    if( uiGroupIdxY > 3 )
	{      
        UInt nCount = ( uiGroupIdxY - 2 ) >> 1;
        nPosY       = nPosY - g_uiMinInGroup[ uiGroupIdxY ];
        xCabacEncodeBinsEP( pCabac, pBS, nPosY, nCount );
    }
}



void xEncodeCoeffNxN( X265_t *h, X265_Cache *pCache, X265_Cabac *pCabac, X265_BitStream *pBS, Int16 *psTempCoef, UInt uiAbsPartIdx, UInt nSize, UInt nDepth, UInt8 bIsLuma, UInt nMode )
{
	Int NumPartitionInWidth = pCache->NumPartitionInWidth;
	Int16* psCoef = (Int16*)malloc (sizeof(Int16)*nSize*nSize);
	memset( psCoef, 0, sizeof(Int16)*nSize*nSize );
	UInt position;

	if( bIsLuma )
	{
		position = g_auiZscanToRaster[uiAbsPartIdx];
		for( UInt i=0; i<nSize; i++ )
		{
			for( UInt j=0; j<nSize; j++ )
			{
				psCoef[i*nSize+j] = psTempCoef[((position/NumPartitionInWidth)*MIN_CU_SIZE+i)*MAX_CU_SIZE+(position%NumPartitionInWidth)*MIN_CU_SIZE+j];
			}
		}
	}
	else
	{
		position = g_auiZscanToRaster[uiAbsPartIdx/4];
		for( UInt i=0; i<nSize; i++ )
		{
			for( UInt j=0; j<nSize; j++ )
			{
				psCoef[i*nSize+j] = psTempCoef[((position/NumPartitionInWidth)*MIN_CU_SIZE+i)*(MAX_CU_SIZE/2)+(position%NumPartitionInWidth)*MIN_CU_SIZE+j];
			}
		}
	}
	
	const UInt      nStride      = nSize;
          UInt      nLog2Size    = xLog2( nSize - 1 );
          UInt32    uiNumSig     = countNonZeroCoeffs( psCoef, nStride, nSize );
		  UInt      nScanIdx     = getCoefScanIdx( pCache, nSize, h->isIntra, bIsLuma, nMode, uiAbsPartIdx );
    const UInt16   *scan         = NULL;
    const UInt16   *scanCG       = NULL;
    const UInt      uiShift      = MLS_CG_SIZE >> 1;
    const UInt      uiNumBlkSide = nSize >> uiShift;
    const Int       blockType    = nLog2Size;
    UInt8 uiSigCoeffGroupFlag[MLS_GRP_NUM];
    Int idx;
	
    // Map zigzag to diagonal scan
    if( nScanIdx == SCAN_ZIGZAG ) {
        nScanIdx = SCAN_DIAG;
    }

    assert( nLog2Size <= 5 );
    scan   = g_ausScanIdx[ nScanIdx ][ nLog2Size - 1 ];
    scanCG = g_ausScanIdx[ nScanIdx ][ nLog2Size < 3 ? 0 : 1 ];
    if( nLog2Size == 3 ) 
	{
      scanCG = g_sigLastScan8x8[ nScanIdx ];
    }
    else if( nLog2Size == 5 ) 
	{
      scanCG = g_sigLastScanCG32x32;
    }

    memset( uiSigCoeffGroupFlag, 0, sizeof(uiSigCoeffGroupFlag) );

    // Find position of last coefficient
    Int scanPosLast = -1;
    Int iRealPos = -1;
    Int posLast;
    do 
	{
        posLast = scan[ ++scanPosLast ];

        // get L1 sig map
        UInt nPosY    = posLast >> nLog2Size;
        UInt nPosX    = posLast - ( nPosY << nLog2Size );
        UInt nBlkIdx  = uiNumBlkSide * (nPosY >> uiShift) + (nPosX >> uiShift);

        iRealPos = nPosY * nStride + nPosX;

        if( nSize == 8 && (nScanIdx == SCAN_HOR || nScanIdx == SCAN_VER) ) 
		{
            if( nScanIdx == SCAN_HOR ) 
			{
                nBlkIdx = nPosY >> 1;
            }
            else if( nScanIdx == SCAN_VER ) 
			{
                nBlkIdx = nPosX >> 1;
            }
        }

        if( psCoef[iRealPos] )
		{
            uiSigCoeffGroupFlag[nBlkIdx] = 1;
        }

        uiNumSig -= ( psCoef[iRealPos] != 0 );
    } while( uiNumSig > 0 );

    // Code position of last coefficient
    UInt posLastY = posLast >> nLog2Size;
    UInt posLastX = posLast - ( posLastY << nLog2Size );
    codeLastSignificantXY( pCabac, pBS, posLastX, posLastY, nSize, bIsLuma, nScanIdx );

    //===== code significance flag =====
    UInt nBaseCoeffGroupCtx = OFF_SIG_CG_FLAG_CTX + (bIsLuma ? 0 : NUM_SIG_CG_FLAG_CTX);
    UInt nBaseCtx = OFF_SIG_FLAG_CTX + (bIsLuma ? 0 : NUM_SIG_FLAG_CTX_LUMA);
    
    const Int  iLastScanSet      = scanPosLast >> LOG2_SCAN_SET_SIZE;
    UInt uiNumOne                = 0;
    UInt uiGoRiceParam           = 0;
    Int  iScanPosSig             = scanPosLast;
    Int  iSubSet;
    
    for( iSubSet = iLastScanSet; iSubSet >= 0; iSubSet-- ) 
	{
        Int numNonZero = 0;
        Int  iSubPos     = iSubSet << LOG2_SCAN_SET_SIZE;
        uiGoRiceParam    = 0;
        Int16 absCoeff[16];
        UInt32 coeffSigns = 0;
        
        if( iScanPosSig == scanPosLast ) 
		{
            absCoeff[0] = abs( psCoef[iRealPos] );
            coeffSigns  = ( psCoef[iRealPos] < 0 );
            numNonZero  = 1;
            iScanPosSig--;
        }
        
        // encode significant_coeffgroup_flag
        Int iCGBlkPos = scanCG[ iSubSet ];
        Int iCGPosY   = iCGBlkPos / uiNumBlkSide;
        Int iCGPosX   = iCGBlkPos - (iCGPosY * uiNumBlkSide);
        if( nSize == 8 && (nScanIdx == SCAN_HOR || nScanIdx == SCAN_VER) ) 
		{
            iCGPosY = (nScanIdx == SCAN_HOR ? iCGBlkPos : 0);
            iCGPosX = (nScanIdx == SCAN_VER ? iCGBlkPos : 0);
        }
        if( iSubSet == iLastScanSet || iSubSet == 0) 
		{
            uiSigCoeffGroupFlag[ iCGBlkPos ] = 1;
        }
        else 
		{
            UInt nSigCoeffGroup   = (uiSigCoeffGroupFlag[ iCGBlkPos ] != 0);
            UInt nCtxSig  = getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, iCGPosX, iCGPosY, nScanIdx, nSize );
            xCabacEncodeBin( pCabac, pBS, nSigCoeffGroup, nBaseCoeffGroupCtx + nCtxSig );
        }
        
        // encode significant_coeff_flag
        if( uiSigCoeffGroupFlag[ iCGBlkPos ] ) 
		{
            UInt nBlkPos, nPosY, nPosX, nSig, nCtxSig;
            UInt nRealBlkPos;
            for( ; iScanPosSig >= iSubPos; iScanPosSig-- ) 
			{
                nBlkPos     = scan[ iScanPosSig ]; 
                nPosY       = nBlkPos >> nLog2Size;
                nPosX       = nBlkPos - ( nPosY << nLog2Size );
                nRealBlkPos = nPosY * nStride + nPosX;
                nSig        = (psCoef[ nRealBlkPos ] != 0);
                if( (iScanPosSig != iSubPos) || iSubSet == 0 || numNonZero ) 
				{
                    nCtxSig  = getSigCtxInc( psCoef, nPosX, nPosY, blockType, nSize, bIsLuma );
                    xCabacEncodeBin( pCabac, pBS, nSig, nBaseCtx + nCtxSig );
                }
                if( nSig ) 
				{
                    absCoeff[numNonZero] = abs( psCoef[nRealBlkPos] );
                    coeffSigns = (coeffSigns << 1) + ( psCoef[nRealBlkPos] < 0 );
                    numNonZero++;
                }
            }
        }
        else 
		{
            iScanPosSig = iSubPos - 1;
        }
        
        if( numNonZero > 0 ) 
		{
            UInt c1 = 1;
            UInt uiCtxSet = (iSubSet > 0 && bIsLuma) ? 2 : 0;
            
            if( uiNumOne > 0 ) 
			{
                uiCtxSet++;
            }
            
            uiNumOne       >>= 1;
            UInt nBaseCtxMod = OFF_ONE_FLAG_CTX + 4 * uiCtxSet + ( bIsLuma ? 0 : NUM_ONE_FLAG_CTX_LUMA);
            
            Int numC1Flag = MIN(numNonZero, C1FLAG_NUMBER);
            Int firstC2FlagIdx = 16;
            for( idx = 0; idx < numC1Flag; idx++ ) 
			{
                UInt uiSymbol = absCoeff[ idx ] > 1;
                xCabacEncodeBin( pCabac, pBS, uiSymbol, nBaseCtxMod + c1 );
                if( uiSymbol ) 
				{
                    c1 = 0;
                    firstC2FlagIdx = MIN(firstC2FlagIdx, idx);
                }
                else if( c1 != 0 ) 
				{
                    c1 = MIN(c1+1, 3);
                }
            }

            if( c1 == 0 ) 
			{
                nBaseCtxMod = OFF_ABS_FLAG_CTX + uiCtxSet + (bIsLuma ? 0 : NUM_ABS_FLAG_CTX_LUMA);
				if( firstC2FlagIdx != 16 ) 
				{
                    UInt symbol = absCoeff[ firstC2FlagIdx ] > 2;
                    xCabacEncodeBin( pCabac, pBS, symbol, nBaseCtxMod + 0 );
                }
            }

            xCabacEncodeBinsEP( pCabac, pBS, coeffSigns, numNonZero );		

            Int iFirstCoeff2 = 1;    
            if( c1 == 0 || numNonZero > C1FLAG_NUMBER ) 
			{
                for( idx = 0; idx < numNonZero; idx++ ) 
				{
                    Int baseLevel = (idx < C1FLAG_NUMBER) ? (2 + iFirstCoeff2 ) : 1;
                    if( absCoeff[ idx ] >= baseLevel ) 
					{
                        xWriteGoRiceExGolomb( pCabac, pBS, absCoeff[ idx ] - baseLevel, uiGoRiceParam ); 
                    }
                    if( absCoeff[ idx ] >= 2 ) 
					{
                        iFirstCoeff2 = 0;
                        uiNumOne++;
                    }
                }        
            }
        }
        else 
		{
            uiNumOne >>= 1;
        }
    }
	free(psCoef);
	psCoef = NULL;

}

void xWriteCU( X265_t *h, UInt uiDepth, UInt uiAbsPartIdx, UInt bLastCU  )
{
    X265_BitStream *pBS         = &h->bs;
    X265_Cabac     *pCabac      = &h->cabac;
    X265_Cache     *pCache      = &h->cache;
	UInt            nCUWidth    = h->ucMaxCUWidth;

	UInt uiNumSubdiv = 4;
	UInt uiNumSubdiv_NS = 2; 
	UInt8 *puhDepth = 0;
	UInt8 eSize = 0;
	UInt8 *CbfY = 0;             
	UInt8 *CbfU = 0;          
	UInt8 *CbfV = 0;
	if (h->isIntra)
	{
		puhDepth = pCache->TemppuhDepth;
		eSize = pCache->TemppePartSize[uiAbsPartIdx];
		CbfY = pCache->TempbCbfY; 
		CbfU = pCache->TempbCbfU; 
		CbfV = pCache->TempbCbfV; 
	}
	else
	{
		puhDepth = pCache->BestpuhInterDepth;
		eSize = pCache->BestpuhInterPartSize[uiAbsPartIdx];
		CbfY = pCache->BestInterCbfY; 
		CbfU = pCache->BestInterCbfU; 
		CbfV = pCache->BestInterCbfV; 
	}

	UInt g_uiMaxCUDepth = h->ucMaxCUDepth;

	//for sequence which size is not multiple of 32 
	bool bBoundary = false;
	UInt uiLPelX   = h->uiCUX + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
    UInt uiRPelX   = uiLPelX + (nCUWidth>>uiDepth);
    UInt uiTPelY   = h->uiCUY + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
    UInt uiBPelY   = uiTPelY + (nCUWidth>>uiDepth);
	if( uiRPelX > h->usWidth + h->PAD_YSIZEw || uiBPelY > h->usHeight + h->PAD_YSIZEh )
	{
		bBoundary = true;
	}

	if( !bBoundary )
	{
        codeSplitFlag( h, pCache, pCabac, pBS, uiAbsPartIdx, uiDepth );
	}

    if( uiDepth < puhDepth[uiAbsPartIdx] || bBoundary )
	{
		UInt uiQNumParts = ( pCache->NumPartition >> (uiDepth<<1) )>>2;
		for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
		{
			if( uiLPelX < h->usWidth + h->PAD_YSIZEw && uiTPelY < h->usHeight + h->PAD_YSIZEh )
			{
				xWriteCU( h, uiDepth+1, uiAbsPartIdx, bLastCU );
			}
		}
        return;
	}

	if( !h ->isIntra )
	{
		codeSkipFlag( h, pCache, pCabac, pBS, uiAbsPartIdx );
	}
		
	if( pCache->BestpuhSkipFlag[uiAbsPartIdx] )
    {
        codeMergeIndex( h, pCache, pCabac, pBS, uiAbsPartIdx );
        return;
    }		

	//codePredMode
	codePredMode( h, pCache, pCabac, pBS, uiAbsPartIdx );

	//codePartSize
	codePartSize( h, pCache, pCabac, pBS, uiAbsPartIdx, uiDepth );

	//codePredInfo ( Intra : direction mode, Inter : Mv, reference idx )
	codePredInfo( h, pCache, pCabac, pBS, uiAbsPartIdx, uiDepth );

    assert( h->ucQuadtreeTUMaxDepthIntra == 1 );

    const UInt8 bFirstChromaCbf = TRUE;

	if( h->isIntra )
	{
        // Cbf
        xCabacEncodeBin( pCabac, pBS, CbfU[uiAbsPartIdx], OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX );
        xCabacEncodeBin( pCabac, pBS, CbfV[uiAbsPartIdx], OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX );
        
		if( eSize == SIZE_NxN )
		{
		    for( UInt uiSubdivIdx=0; uiSubdivIdx<uiNumSubdiv; uiSubdivIdx++ )
		    {
                xCabacEncodeBin( pCabac, pBS, CbfY[uiAbsPartIdx], OFF_QT_CBF_CTX + 0*NUM_QT_CBF_CTX );

                // Coeff
                if(CbfY[uiAbsPartIdx])
				{
                    xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefY[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, TRUE,  pCache->TemppuhLumaIntraDir[uiAbsPartIdx] );
                }
                uiAbsPartIdx++;
		    }       
			uiAbsPartIdx -= uiNumSubdiv;
		}
		
		else
		{
			xCabacEncodeBin( pCabac, pBS, CbfY[uiAbsPartIdx], OFF_QT_CBF_CTX + 0*NUM_QT_CBF_CTX+ getCtxQtCbf( TRUE ) );

            // Coeff
            if(CbfY[uiAbsPartIdx]) 
			{
                xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefY[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]), uiDepth, TRUE,  pCache->TemppuhLumaIntraDir[uiAbsPartIdx] );
            }
		}
		
        if(CbfU[uiAbsPartIdx]) 
		{
            xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefU[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, FALSE, pCache->TemppuhChromaIntraDir[uiAbsPartIdx] );
        }
        if(CbfV[uiAbsPartIdx]) 
		{
            xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefV[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, FALSE, pCache->TemppuhChromaIntraDir[uiAbsPartIdx] );
        }
	}
	//for inter 
	else
	{
		//RootCbf
		UInt partOffset = ( h->NumPartition >> ( uiDepth << 1 ) ) >> 2;
		UInt8 bRootCbfY =  CbfY[uiAbsPartIdx] || CbfY[uiAbsPartIdx+partOffset] || CbfY[uiAbsPartIdx+partOffset*2] || CbfY[uiAbsPartIdx+partOffset*3];
		UInt8 bRootCbfU =  CbfU[uiAbsPartIdx] || CbfU[uiAbsPartIdx+partOffset] || CbfU[uiAbsPartIdx+partOffset*2] || CbfU[uiAbsPartIdx+partOffset*3];
		UInt8 bRootCbfV =  CbfV[uiAbsPartIdx] || CbfV[uiAbsPartIdx+partOffset] || CbfV[uiAbsPartIdx+partOffset*2] || CbfV[uiAbsPartIdx+partOffset*3];
		UInt8 bRootCbf =  bRootCbfY || bRootCbfU || bRootCbfV;
		if( !(eSize == SIZE_2Nx2N && pCache->BestpuhMergeFlag[uiAbsPartIdx]) )
		{
			xCabacEncodeBin( pCabac, pBS, bRootCbf, OFF_QT_ROOT_CBF_CTX );
		}

		if( bRootCbf == 0 )
		{
			return;
		}

		if( eSize == SIZE_2Nx2N )
		{
			//Cbf
			xCabacEncodeBin( pCabac, pBS, CbfU[uiAbsPartIdx], OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX );
			xCabacEncodeBin( pCabac, pBS, CbfV[uiAbsPartIdx], OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX );
			if (!( !CbfU[uiAbsPartIdx] && !CbfV[uiAbsPartIdx]) )
			{
				xCabacEncodeBin( pCabac, pBS, CbfY[uiAbsPartIdx], OFF_QT_CBF_CTX + 0*NUM_QT_CBF_CTX + getCtxQtCbf( TRUE ) );
			}
			if( CbfY[uiAbsPartIdx] )
			{
				xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefY[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]), uiDepth, TRUE,  pCache->TemppuhLumaIntraDir[uiAbsPartIdx] );
			}
			if( CbfU[uiAbsPartIdx] )
			{
				xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefU[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, FALSE, pCache->TemppuhChromaIntraDir[uiAbsPartIdx] );
			}
			if( CbfV[uiAbsPartIdx] )
			{
				xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefV[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, FALSE, pCache->TemppuhChromaIntraDir[uiAbsPartIdx] );
			}
		}
		else
		{
			if( uiDepth == 2 )
			{
				xCabacEncodeBin( pCabac, pBS, CbfU[uiAbsPartIdx], OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX );
				xCabacEncodeBin( pCabac, pBS, CbfV[uiAbsPartIdx], OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX );
			}
			else
			{
				xCabacEncodeBin( pCabac, pBS, bRootCbfU, OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX  );
				xCabacEncodeBin( pCabac, pBS, bRootCbfV, OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX  );
			}
			for( UInt uiSubdivIdx=0; uiSubdivIdx<uiNumSubdiv; uiSubdivIdx++ )
			{
				if( uiDepth == 2 )
				{
					xCabacEncodeBin( pCabac, pBS, CbfY[uiAbsPartIdx], OFF_QT_CBF_CTX + 0*NUM_QT_CBF_CTX );
					if( CbfY[uiAbsPartIdx] )
					{
						xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefY[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, TRUE,  pCache->TemppuhLumaIntraDir[uiAbsPartIdx] );
					}
					uiAbsPartIdx += partOffset;
				}
				else
				{
					if(bRootCbfU)
					{
						xCabacEncodeBin( pCabac, pBS, CbfU[uiAbsPartIdx], OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX + getCtxQtCbf( TRUE ) );
					}
					if(bRootCbfV)
					{
						xCabacEncodeBin( pCabac, pBS, CbfV[uiAbsPartIdx], OFF_QT_CBF_CTX + 1*NUM_QT_CBF_CTX + getCtxQtCbf( TRUE ) );
					}					

					xCabacEncodeBin( pCabac, pBS, CbfY[uiAbsPartIdx], OFF_QT_CBF_CTX + 0*NUM_QT_CBF_CTX );
		
					if( CbfY[uiAbsPartIdx] )
					{
						xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefY[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, TRUE,  pCache->TemppuhLumaIntraDir[uiAbsPartIdx] );
					}
					if( CbfU[uiAbsPartIdx] )
					{
						xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefU[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+2), uiDepth, FALSE, pCache->TemppuhChromaIntraDir[uiAbsPartIdx] );
					}
					if( CbfV[uiAbsPartIdx] )
					{
						xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefV[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+2), uiDepth, FALSE, pCache->TemppuhChromaIntraDir[uiAbsPartIdx] );
					}
					uiAbsPartIdx += partOffset;
				}
			}
			uiAbsPartIdx -= uiNumSubdiv*partOffset;

			if( uiDepth == 2 )
			{
				if( CbfU[uiAbsPartIdx] )
				{
					xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefU[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, FALSE, pCache->TemppuhChromaIntraDir[uiAbsPartIdx] );
				}
				if( CbfV[uiAbsPartIdx] )
				{
					xEncodeCoeffNxN( h, pCache, pCabac, pBS, pCache->psTempCoefV[0], uiAbsPartIdx, nCUWidth>>(puhDepth[uiAbsPartIdx]+1), uiDepth, FALSE, pCache->TemppuhChromaIntraDir[uiAbsPartIdx] );
				}
			}
		}
	}
}

//estimate the number of bins for a coefficient block
UInt xEncodeCoeffNxNBinEst(X265_t *h, X265_Cache *pCache, Int16 *psCoef, UInt uiAbsPartIdx, UInt nSize, UInt8 bIsLuma, UInt nMode )
{
	UInt numBinEst = 0;

	const UInt		nStride 	 		= nSize;
	UInt		nLog2Size	 	= xLog2( nSize - 1 ); 
	UInt32		uiNumSig	 	= countNonZeroCoeffs( psCoef, nStride, nSize );
	UInt		nScanIdx			=SCAN_DIAG;
	const UInt16   	*scan		 	= NULL;
	const UInt16   	*scanCG		 	= NULL;
	const UInt		uiShift 	 		= MLS_CG_SIZE >> 1;
	const UInt		uiNumBlkSide 	= nSize >> uiShift;
	const Int			blockType	 	= nLog2Size;
	UInt8 		uiSigCoeffGroupFlag[MLS_GRP_NUM];
	Int idx;

	assert( nLog2Size <= 5 );
	scan   = g_ausScanIdx[ nScanIdx ][ nLog2Size - 1 ];
	scanCG = g_ausScanIdx[ nScanIdx ][ nLog2Size < 3 ? 0 : 1 ];
	if( nLog2Size == 3 ) {
		scanCG = g_sigLastScan8x8[ nScanIdx ];
	}
	else if( nLog2Size == 5 ) {
		scanCG = g_sigLastScanCG32x32;
	}

	memset( uiSigCoeffGroupFlag, 0, sizeof(uiSigCoeffGroupFlag) );

	// Find position of last coefficient
	Int scanPosLast = -1;
	Int iRealPos = -1;
	Int posLast;
	do {
		posLast = scan[ ++scanPosLast ];

		// get L1 sig map
		UInt nPosY	  = posLast >> nLog2Size;
		UInt nPosX	  = posLast - ( nPosY << nLog2Size );
		UInt nBlkIdx  = uiNumBlkSide * (nPosY >> uiShift) + (nPosX >> uiShift);

		iRealPos = nPosY * nStride + nPosX;

		if( psCoef[iRealPos] ) {
			uiSigCoeffGroupFlag[nBlkIdx] = 1;
		}

		uiNumSig -= ( psCoef[iRealPos] != 0 );
	} while( uiNumSig > 0 );

	// Code position of last coefficient
	UInt posLastY = posLast >> nLog2Size;
	UInt posLastX = posLast - ( posLastY << nLog2Size );
	numBinEst += codeLastSignificantXYBinEst( posLastX, posLastY, nSize, bIsLuma, nScanIdx );

	//===== code significance flag =====
	UInt nBaseCoeffGroupCtx = OFF_SIG_CG_FLAG_CTX + (bIsLuma ? 0 : NUM_SIG_CG_FLAG_CTX);
	UInt nBaseCtx = OFF_SIG_FLAG_CTX + (bIsLuma ? 0 : NUM_SIG_FLAG_CTX_LUMA);

	const Int  iLastScanSet 	 = scanPosLast >> LOG2_SCAN_SET_SIZE;
	UInt c1 = 1;
	UInt uiGoRiceParam			 = 0;
	Int  iScanPosSig			 = scanPosLast;
	Int  iSubSet;

	for( iSubSet = iLastScanSet; iSubSet >= 0; iSubSet-- ) {
		Int numNonZero = 0;
		Int  iSubPos	 = iSubSet << LOG2_SCAN_SET_SIZE;
		uiGoRiceParam	 = 0;
		Int16 absCoeff[16];
		UInt32 coeffSigns = 0;

		if( iScanPosSig == scanPosLast ) {
			absCoeff[0] = abs( psCoef[iRealPos] );
			coeffSigns	= ( psCoef[iRealPos] < 0 );
			numNonZero	= 1;
			iScanPosSig--;
		}

		// encode significant_coeffgroup_flag
		Int iCGBlkPos = scanCG[ iSubSet ];
		Int iCGPosY   = iCGBlkPos / uiNumBlkSide;
		Int iCGPosX   = iCGBlkPos - (iCGPosY * uiNumBlkSide);
		if( iSubSet == iLastScanSet || iSubSet == 0) {
			uiSigCoeffGroupFlag[ iCGBlkPos ] = 1;
		}
		else {
			UInt nSigCoeffGroup   = (uiSigCoeffGroupFlag[ iCGBlkPos ] != 0);
			numBinEst++;
		}

		// encode significant_coeff_flag
		if( uiSigCoeffGroupFlag[ iCGBlkPos ] ) {
			UInt nBlkPos, nPosY, nPosX, nSig;
			UInt nRealBlkPos;
			for( ; iScanPosSig >= iSubPos; iScanPosSig-- ) {
				nBlkPos 	= scan[ iScanPosSig ]; 
				nPosY		= nBlkPos >> nLog2Size;
				nPosX		= nBlkPos - ( nPosY << nLog2Size );
				nRealBlkPos = nPosY * nStride + nPosX;
				nSig		= (psCoef[ nRealBlkPos ] != 0);
				if( (iScanPosSig != iSubPos) || iSubSet == 0 || numNonZero ) {
					numBinEst++;
				}
				if( nSig ) {
					absCoeff[numNonZero] = abs( psCoef[nRealBlkPos] );
					coeffSigns = (coeffSigns << 1) + ( psCoef[nRealBlkPos] < 0 );
					numNonZero++;
				}
			}
		}
		else {
			iScanPosSig = iSubPos - 1;
		}

		if( numNonZero > 0 ) {
			UInt uiCtxSet = (iSubSet > 0 && bIsLuma) ? 2 : 0;
			if( c1 == 0 )
			{
				uiCtxSet++;
			}
			c1 = 1;

			UInt nBaseCtxMod = OFF_ONE_FLAG_CTX + 4 * uiCtxSet + ( bIsLuma ? 0 : NUM_ONE_FLAG_CTX_LUMA);

			Int numC1Flag = MIN(numNonZero, C1FLAG_NUMBER);
			Int firstC2FlagIdx = 16;
			for( idx = 0; idx < numC1Flag; idx++ ) {
				UInt uiSymbol = absCoeff[ idx ] > 1;
				numBinEst++;
				if( uiSymbol ) {
					c1 = 0;
					firstC2FlagIdx = MIN(firstC2FlagIdx, idx);
				}
				else if( c1 != 0 ) {
					c1 = MIN(c1+1, 3);
				}
			}

			if( c1 == 0 ) {
				nBaseCtxMod = OFF_ABS_FLAG_CTX + uiCtxSet + (bIsLuma ? 0 : NUM_ABS_FLAG_CTX_LUMA);

				//if( firstC2FlagIdx != 16 ) {
				if( firstC2FlagIdx != -1 ) {
					UInt symbol = absCoeff[ firstC2FlagIdx ] > 2;
					numBinEst++;
				}
			}

			numBinEst+=numNonZero;	

			Int iFirstCoeff2 = 1;	 
			if( c1 == 0 || numNonZero > C1FLAG_NUMBER ) {
				for( idx = 0; idx < numNonZero; idx++ ) {
					Int baseLevel = (idx < C1FLAG_NUMBER) ? (2 + iFirstCoeff2 ) : 1;

					if( absCoeff[ idx ] >= baseLevel ) {
						numBinEst +=xWriteCoefRemainExGolombBinEst( absCoeff[ idx ] - baseLevel, uiGoRiceParam );
						if(absCoeff[idx] > 3*(1<<uiGoRiceParam))
						{
							uiGoRiceParam = (uiGoRiceParam+ 1) < 4 ? (uiGoRiceParam+ 1) : 4;
						}
					}
					if( absCoeff[ idx ] >= 2 ) {
						iFirstCoeff2 = 0;
					}
				}		 
			}
		}
	}
	return numBinEst;
}

//for estmating the bin 
UInt codeLastSignificantXYBinEst( UInt nPosX, UInt nPosY, UInt nSize, UInt8 bIsLuma, UInt nScanIdx )
{
	UInt  numBinEst = 0; //number of Bins
	const UInt nLog2Size = xLog2( nSize - 1 );

	// swap
	if( nScanIdx == SCAN_VER ) {
		UInt tmp = nPosY;
		nPosY = nPosX;
		nPosX = tmp;
	}

	UInt nCtxLast;
	UInt uiGroupIdxX    = g_uiGroupIdx[ nPosX ];
	UInt uiGroupIdxY    = g_uiGroupIdx[ nPosY ];

	for( nCtxLast = 0; nCtxLast < uiGroupIdxX; nCtxLast++ ) {
		numBinEst++;
	}
	if( uiGroupIdxX < g_uiGroupIdx[nSize-1]) {
		numBinEst++;
	}

	for( nCtxLast = 0; nCtxLast < uiGroupIdxY; nCtxLast++ ) {
		numBinEst++;
	}
	if( uiGroupIdxY < g_uiGroupIdx[ nSize - 1 ]) {
		numBinEst++;
	}

	if( uiGroupIdxX > 3 ) {      
		UInt nCount = ( uiGroupIdxX - 2 ) >> 1;
		nPosX       = nPosX - g_uiMinInGroup[ uiGroupIdxX ];
		numBinEst+=nCount;
	}
	if( uiGroupIdxY > 3 ) {      
		UInt nCount = ( uiGroupIdxY - 2 ) >> 1;
		nPosY       = nPosY - g_uiMinInGroup[ uiGroupIdxY ];
		numBinEst+=nCount;
	}
	return numBinEst;
}

UInt xWriteCoefRemainExGolombBinEst (  UInt symbol, UInt &rParam )
{
	UInt numBinEst =0;

	Int codeNumber  = (Int)symbol;
	UInt length;
	if (codeNumber < (COEF_REMAIN_BIN_REDUCTION << rParam))
	{
		length = codeNumber>>rParam;
		numBinEst+=  length+1;
		numBinEst+=  rParam;
	}
	else
	{
		length = rParam;
		codeNumber  = codeNumber - ( COEF_REMAIN_BIN_REDUCTION << rParam);
		while (codeNumber >= (1<<length))
		{
			codeNumber -=  (1<<(length++));    
		}
		numBinEst+=  COEF_REMAIN_BIN_REDUCTION+length+1-rParam;
		numBinEst+=  length;
	}
	return numBinEst;
}