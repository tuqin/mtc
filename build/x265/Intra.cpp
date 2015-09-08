#include "MTC265.h"

extern UInt g_auiRasterToZscan [MAX_PU_XY*MAX_PU_XY]; 
extern UInt g_auiZscanToRaster [MAX_PU_XY*MAX_PU_XY]; 

void xPaddingRef( Pxl *pucRef, const bool bValid[5], const UInt nBlkOffset[6] )
{
    UInt    i, n;
    Pxl   ucPadding;
    const Int nValid = 5;

    // Padding from Right to Left
    for( n=0; n<nValid; n++ ) 
	{
        if( bValid[n] )
            break;
    }
    ucPadding = pucRef[nBlkOffset[n]];
    for( i=0; i<nBlkOffset[n]; i++ ) 
	{
        pucRef[i] = ucPadding;
    }

    // Padding from Left to Right
    for( ; n<nValid; n++ ) 
	{
        if( !bValid[n] ) 
		{
            assert( n > 0 );
            const UInt nBlkAddr = nBlkOffset[n];
            const UInt nBlkSize = nBlkOffset[n + 1] - nBlkOffset[n];
            ucPadding = pucRef[nBlkAddr - 1];
            for( i=0; i<nBlkSize; i++ ) 
			{
                pucRef[nBlkAddr + i] = ucPadding;
            }
        }
    }
}

void xPredIntraPlanar(
    Pxl   *pucRef,
    Pxl   *pucDst,
    Int      nDstStride,
    UInt     nSize
)
{
    UInt nLog2Size = xLog2(nSize - 1);
    Pxl *pucLeft = pucRef + 2 * nSize - 1;
    Pxl *pucTop  = pucRef + 2 * nSize + 1;
    Int i, j;
    Pxl bottomLeft, topRight;
    Int16 horPred;
    Int16 leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE+1], rightColumn[MAX_CU_SIZE+1];
    UInt offset2D = nSize;
    UInt shift1D = nLog2Size;
    UInt shift2D = shift1D + 1;

    // Get left and above reference column and row
    for( i=0; i<(Int)nSize+1; i++) 
	{
        topRow[i]     = pucTop[i];
        leftColumn[i] = pucLeft[-i];
    }

    // Prepare intermediate variables used in interpolation
    bottomLeft = pucLeft[-(Int)nSize];
    topRight   = pucTop[nSize];
    for( i=0; i<(Int)nSize; i++ ) 
	{
        bottomRow[i]   = bottomLeft - topRow[i];
        rightColumn[i] = topRight   - leftColumn[i];
        topRow[i]      <<= shift1D;
        leftColumn[i]  <<= shift1D;
    }

    // Generate prediction signal
    for( i=0; i<(Int)nSize; i++ ) 
	{
        horPred = leftColumn[i] + offset2D;
        for( j=0; j<(Int)nSize; j++ ) 
		{
            horPred += rightColumn[i];
            topRow[j] += bottomRow[j];
            pucDst[i*nDstStride+j] = ( (horPred + topRow[j]) >> shift2D );
        }
    }
}

Pxl xPredIntraGetDCVal(
    Pxl   *pucRef,
    UInt     nSize
)
{
    Pxl *pucLeft = pucRef + 2 * nSize - 1;
    Pxl *pucTop  = pucRef + 2 * nSize + 1;
    UInt32 uiSumTop = 0;
    UInt32 uiSumLeft = 0;
    UInt8 ucDcVal;
    int i;

    for( i=0; i<(Int)nSize; i++ ) 
	{
        uiSumTop  += pucTop [ i];
        uiSumLeft += pucLeft[-i];
    }
    ucDcVal = (uiSumTop + uiSumLeft + nSize) / (nSize + nSize);
    return ucDcVal;
}

void xPredIntraDc(
    Pxl   *pucRef,
    Pxl   *pucDst,
    Int      nDstStride,
    UInt     nSize,
    UInt     bLuma
)
{
    Pxl *pucLeft = pucRef + 2 * nSize - 1;
    Pxl *pucTop  = pucRef + 2 * nSize + 1;
    Pxl ucDcVal = xPredIntraGetDCVal( pucRef, nSize );
    int i;

    // Fill DC Val
    for( i=0; i<(Int)nSize; i++ )
	{
        memset( &pucDst[i * nDstStride], ucDcVal, nSize );
    }

    // DC Filtering ( 8.4.3.1.5 )
    if( bLuma ) 
	{
        pucDst[0] = ( pucTop[0] + pucLeft[0] + 2 * pucDst[0] + 2 ) >> 2;
        for( i=1; i<(Int)nSize; i++ ) 
		{
            pucDst[i           ] = ( pucTop [ i] + 3 * pucDst[i           ] + 2 ) >> 2;
            pucDst[i*nDstStride] = ( pucLeft[-i] + 3 * pucDst[i*nDstStride] + 2 ) >> 2;
        }
    }
}

void xPredIntraAng(
    Pxl   *pucRef,
    Pxl   *pucDst,
    Int      nDstStride,
     Int     nSize,
    UInt     nMode,
    UInt     bLuma
)
{
    UInt   bModeHor          = (nMode < 18);
    Int    nIntraPredAngle  = g_aucIntraPredAngle[nMode];
    Int    nInvAngle        = g_aucInvAngle[nMode];
    Pxl *pucLeft          = pucRef + 2 * nSize - 1;
    Pxl *pucTop           = pucRef + 2 * nSize + 1;
    Pxl *pucTopLeft       = pucRef + 2 * nSize;
    Pxl  ucRefBuf[2*MAX_CU_SIZE+1];
    Pxl *pucRefMain       = ucRefBuf + ( (nIntraPredAngle < 0) ? MAX_CU_SIZE : 0 );
    Int    x, k;

    // (8-47) and (8-50)
    for( x=0; x<nSize+1; x++ ) 
	{
        pucRefMain[x] = bModeHor ? pucTopLeft[-x] : pucTopLeft[x];
    }

    if( nIntraPredAngle < 0 ) 
	{
        Int iSum = 128;
        // (8-48) or (8-51)
        for( x=-1; x>(nSize*nIntraPredAngle)>>5; x-- ) 
		{
            iSum += nInvAngle;
            Int nOffset = bModeHor ? (iSum >> 8) : -(iSum >> 8);
            pucRefMain[x] = pucTopLeft[ nOffset ];
        }
    }
    else 
	{
        // (8-49) or (8-52)
        for( x=nSize+1; x<2*nSize+1; x++ ) 
		{
            pucRefMain[x] = bModeHor ? pucTopLeft[-x] : pucTopLeft[x];
        }
    }

    // 8.4.3.1.6
    Int deltaPos=0;
    Int refMainIndex;
    
    for( k=0; k<nSize; k++ ) 
	{
        deltaPos += nIntraPredAngle;
        Int iIdx  = deltaPos >> 5;  // (8-53)
        Int iFact = deltaPos & 31;  // (8-54)
        
        if( iFact )
		{
            // Do linear filtering
            for( x=0; x<nSize; x++ )
			{
                refMainIndex           = x+iIdx+1;
                pucDst[k*nDstStride+x] = ( ((32-iFact)*pucRefMain[refMainIndex]+iFact*pucRefMain[refMainIndex+1]+16) >> 5 );
            }
        }
        else 
		{
            // Just copy the integer samples
            for( x=0; x<nSize; x++) 
			{
                pucDst[k*nDstStride+x] = pucRefMain[iIdx+1+x];
            }
        }
    }

    // Filter if this is IntraPredAngle zero mode
    // see 8.4.3.1.3 and 8.4.3.1.4
    if( bLuma && (nIntraPredAngle == 0) )
    {
        Int offset = bModeHor ? 1 : -1;
        for( x=0; x<nSize; x++ ) 
		{
            pucDst[x*nDstStride] = Clip ( pucDst[x*nDstStride] + (( pucTopLeft[(x+1)*offset] - pucTopLeft[0] ) >> 1) );
        }
    }

	// Matrix Transpose if this is the horizontal mode
	if( bModeHor ) 
	{
		Pxl tmp;
		for (k=0;k<nSize-1;k++)
		{
			for (x=k+1;x<nSize;x++)
			{
				tmp                 = pucDst[k*nDstStride+x];
				pucDst[k*nDstStride+x] = pucDst[x*nDstStride+k];
				pucDst[x*nDstStride+k] = tmp;
			}
		}
	}
}

void xPredIntraLM(
    UInt8   *pucRefC,
    UInt8   *pucRefM_L,
    UInt8   *pucRefM_T,
    UInt8   *pucPredLM,
    UInt8   *pucDst,
    UInt     nSize
)
{
    // Table 8-7 – Specification of lmDiv
    static const UInt16 lmDiv[64] = 
	{
        32768, 16384, 10923, 8192, 6554, 5461, 4681, 4096,
         3641,  3277,  2979, 2731, 2521, 2341, 2185, 2048,
         1928,  1820,  1725, 1638, 1560, 1489, 1425, 1365,
         1311,  1260,  1214, 1170, 1130, 1092, 1057, 1024,
          993,   964,   936,  910,  886,  862,  840,  819,
          799,   780,   762,  745,  728,  712,  697,  683,
          669,   655,   643,  630,  618,  607,  596,  585,
          575,   565,   555,  546,  537,  529,  520,  512,

    };
    const UInt nLog2Size = xLog2( nSize - 1 );
    UInt8 *pucRefC_L = pucRefC + 2 * nSize - 1;
    UInt8 *pucRefC_T = pucRefC + 2 * nSize + 1;
    Int i;
    Int32 L=0, C=0, LL=0, LC=0;

    // (8-58) k3 = MAX(0, 8 + xLog2( nSize - 1 ) - 14) = 0;
    UInt k2 = xLog2(2*nSize - 1);   // (8-66)

    for( i=0; i<(Int)nSize; i++ ) 
	{
        UInt32 L0 = ( pucRefM_L[ i] + pucRefM_T[i] );
        UInt32 C0 = ( pucRefC_L[-i] + pucRefC_T[i] );
        L  += L0;   // (8-62)
        C  += C0;   // (8-63)
        LL += (pucRefM_L[i] * pucRefM_L[ i]) + (pucRefM_T[i] * pucRefM_T[i]);    // (8-64)
        LC += (pucRefM_L[i] * pucRefC_L[-i]) + (pucRefM_T[i] * pucRefC_T[i]);    // (8-65)
    }

    Int a1 = ( LC << k2 ) - L * C;
    Int a2 = ( LL << k2 ) - L * L;

    Int32 a1s = a1;
    Int32 a2s = a2;
    Int32 a, b;
    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    UInt nShift = 13;

    iScaleShiftA1 = MAX(0, xLog2( abs( a1 ) ) - 15);    
    iScaleShiftA2 = MAX(0, xLog2( abs( a2 ) ) -  6);    

    Int iScaleShiftA = iScaleShiftA2 + 15 - nShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;
    a1s = a1 >> iScaleShiftA1;

    a = (a2s < 1) ? 0 : a1s * lmDiv[a2s - 1];

    assert( iScaleShiftA >= 0 );
    a = Clip3( -32768, 32767, (a >> iScaleShiftA) );
    
    Int minA = -(1 << 6);
    Int maxA =  (1 << 6) - 1;
    if( a < minA || a > maxA ) 
	{
        UInt n = 15 - xLog2( (a >= 0 ? a : ~a) );
        a >>= (9-n);
        nShift -= (9-n);
    }
    b = (  C - ( ( a * L ) >> nShift ) + ( 1 << ( k2 - 1 ) ) ) >> k2;
    
    UInt x, y;
    for( y=0; y<nSize; y++ ) 
	{
        for( x=0; x<nSize; x++ ) 
		{
            Int32 T = ((pucPredLM[y*MAX_CU_SIZE/2+x] * a) >> nShift) + b;
            pucDst[y*MAX_CU_SIZE/2+x] = Clip(T);
        }
    }
}

void xEncIntraPredLuma( X265_t *h, UInt nMode, UInt nSize )
{
    X265_Cache  *pCache     = &h->cache;
    UInt        nLog2Size   = xLog2(nSize - 1);
    UInt        bFilter     = g_aucIntraFilterType[nLog2Size-2][nMode];
    Pxl       *pucRefY    = pCache->pucPixRef[bFilter];
    Pxl       *pucDstY    = pCache->pucPredY  + MAX_CU_SIZE*pCache->cuY + pCache->cuX;
    if( nMode == PLANAR_IDX ) 
	{
        xPredIntraPlanar(
            pucRefY,
            pucDstY,
            MAX_CU_SIZE,
            nSize
        );
    }
    else if( nMode == DC_IDX ) 
	{
        xPredIntraDc(
            pucRefY,
            pucDstY,
            MAX_CU_SIZE,
            nSize,
            TRUE
        );
    }
    else 
	{
        xPredIntraAng(
            pucRefY,
            pucDstY,
            MAX_CU_SIZE,
            nSize,
            nMode,
            TRUE
        );
    }
}

void xEncIntraPredChroma( X265_t *h, UInt nMode, UInt nSize )
{
    X265_Cache  *pCache     = &h->cache;

    if( nMode == PLANAR_IDX ) 
	{
        xPredIntraPlanar(
            pCache->pucPixRefC[0],
            pCache->pucPredC[0]+ (MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2,
            MAX_CU_SIZE / 2,
            nSize
        );
        xPredIntraPlanar(
            pCache->pucPixRefC[1],
            pCache->pucPredC[1]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2,
            MAX_CU_SIZE / 2,
            nSize
        );
    }
    else if( nMode == DC_IDX )
	{
        xPredIntraDc(
            pCache->pucPixRefC[0],
           pCache->pucPredC[0]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2,
            MAX_CU_SIZE / 2,
            nSize,
            FALSE
        );
        xPredIntraDc(
            pCache->pucPixRefC[1],
            pCache->pucPredC[1]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2,
            MAX_CU_SIZE / 2,
            nSize,
            FALSE
        );
    }

	else 
	{
		xPredIntraAng(
			pCache->pucPixRefC[0],
			pCache->pucPredC[0]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2,
			MAX_CU_SIZE / 2,
			nSize,
			nMode,
			FALSE
			);
		xPredIntraAng(
			pCache->pucPixRefC[1],
			pCache->pucPredC[1]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2,
			MAX_CU_SIZE / 2,
			nSize,
			nMode,
			FALSE
			);
	}
}

//for cu partition
void xEncGetCUPosition( X265_t *h,UInt uiCUWidth, UInt8  uiDepth, UInt uiCUX, UInt uiCUY, UInt* uisubCUX, UInt* uisubCUY)
{   
	X265_Cache *pCache = &h->cache;
	UInt8 uiMaxCUWidth = h->ucMaxCUWidth;
	* uisubCUX += h->uiCUX;
	* uisubCUY += h->uiCUY;
	Int * PartIdx = pCache->PartIdx;
    for(int i=0;i<=uiDepth;i++)
	{
		if(PartIdx[i]==0)
		{
			* uisubCUX += (uiMaxCUWidth>>i)*0 ;
			* uisubCUY += (uiMaxCUWidth>>i)*0 ;
		}
		else if(PartIdx[i]==1)
		{
			* uisubCUX += (uiMaxCUWidth>>i)*1 ;
			* uisubCUY += (uiMaxCUWidth>>i)*0 ;
		}
		else if(PartIdx[i]==2)
		{
			* uisubCUX += (uiMaxCUWidth>>i)*0 ;
			* uisubCUY += (uiMaxCUWidth>>i)*1 ;
		}
		else if(PartIdx[i]==3)
		{
			* uisubCUX += (uiMaxCUWidth>>i)*1 ;
			* uisubCUY += (uiMaxCUWidth>>i)*1 ;
		}		
	}
}

void xEncIntraCompressCU(X265_t *h,UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY)
{
	X265_Cache  *CU_Cache     = &h->cache;
	UInt32 iLCUX              = uiCUX; //当前CU的X坐标
	UInt32 iLCUY              = uiCUY; //当前CU的Y坐标
	Int * PartIdx             = CU_Cache->PartIdx;
	Int * RelIdx              = CU_Cache-> RelativeIdx;
	UInt nCUSize              = (h->ucMaxCUWidth)>>uiDepth;
	bool SplitFlag            = true;
	{
		xEncCheckRDCost(h,SIZE_2Nx2N,uiDepth, uiCUX,  uiCUY);
		if(uiDepth == h->ucMaxCUDepth-1)
		{   		  
			xEncCheckRDCost(h,SIZE_NxN,uiDepth, uiCUX,  uiCUY); 	
		}
	}
	if( SplitFlag == true&&uiDepth<h->ucMaxCUDepth-1)
	{ 
		UInt32 nextDepth   = uiDepth+1;
		UInt32 subCUWidth  = h->ucMaxCUWidth>>nextDepth;
		UInt32 subCUHeight = h->ucMaxCUWidth>>nextDepth;
		UInt32 subCUX;UInt32 subCUY;
		for(int partindex=0; partindex<4; partindex++)
		{   
		    subCUX=0;
		    subCUY=0;
			
			PartIdx[nextDepth]=partindex;

			UInt32 RelativIdx=0;
			for(Int32 num=0;num<=Int32(nextDepth);num++)
			{
				int ipow  = int(pow(double(4),num)); 
				RelativIdx +=(PartIdx[num])*(CU_Cache->NumPartition)/ipow;
			}
			
			RelIdx[nextDepth]=RelativIdx;
			xEncGetCUPosition(h,subCUWidth, nextDepth,iLCUX, iLCUY, &subCUX, &subCUY);//找CU位置	
			xEncIntraLoadRef(h, subCUX, subCUY, subCUWidth, nextDepth,RelativIdx,RelIdx[nextDepth-1]);                   //加载参考像素
			xEncIntraCompressCU(h,nextDepth, subCUX, subCUY);
		}
		//  计算当前深度的失真、比特数、总cost
		subCUX=0;
	  	subCUY=0;
		xEncGetCUPosition(h,nCUSize,uiDepth,h->uiCUX,h->uiCUY, &subCUX, &subCUY);
		CU_Cache->cuX = subCUX-h->uiCUX;   //在LCU中的相对位置
	    CU_Cache->cuY = subCUY-h->uiCUY;   //在LCU中的相对位置
		xEncCheckBestMode(h,nextDepth,nCUSize );
		xEncCacheStoreCU(h,subCUX,subCUY,nCUSize,uiDepth);
		//   将最佳CU中数据更新重建图像等信息;   
	}
}

void xEncCheckRDCost(X265_t *h, X265_PartSize ePartSize, UInt8 uiDepth,UInt32 uiCUX, UInt32 uiCUY)
{
	 X265_Cache  *CU_Cache     = &h->cache;
	 UInt32 nCUSize = h->ucMaxCUWidth>>uiDepth;
	 Int *   RelIdx = CU_Cache->RelativeIdx;
	 Int *  PartIdx = CU_Cache->PartIdx;
	 if(ePartSize == 0)
	 {
		 setPartSizeSubParts(CU_Cache,ePartSize, CU_Cache->RelativeIdx[uiDepth], uiDepth );
		 setPredModeSubParts(CU_Cache, MODE_INTRA, CU_Cache->RelativeIdx[uiDepth], uiDepth );
		 estIntraPredQT(h,nCUSize,uiDepth,ePartSize);
		 setCUPameters( CU_Cache, uiDepth );
		 xEncCacheStoreCU( h, uiCUX, uiCUY, nCUSize, uiDepth );
	 }
	 else if(ePartSize == 3)
	 {		
		 UInt32 subPUWidth  = h-> ucMaxCUWidth>>(uiDepth+1);
		 UInt32 subPUHeight = h-> ucMaxCUWidth>>(uiDepth+1);
		 UInt32 subPUX;UInt32 subPUY;
		 
		 for(int i=0;i<4;i++)
		 {
			subPUX = 0;
	  	    subPUY = 0;			
			PartIdx[uiDepth+1] = i;
			UInt32 RelativIdx  = 0;			
			for(Int32 num=0;num<=Int32(uiDepth+1);num++)
			{
				int ipow    = int(pow(double(4),num)); 
				RelativIdx += (PartIdx[num])*(CU_Cache->NumPartition)/ipow;
			}
			RelIdx[uiDepth+1] = RelativIdx;
			setPartSizeSubParts(CU_Cache,ePartSize,RelativIdx, uiDepth+1 );
            setPredModeSubParts(CU_Cache, MODE_INTRA, RelativIdx, uiDepth+1);
			xEncGetCUPosition(h,subPUWidth, uiDepth+1,h->uiCUX,h->uiCUY, &subPUX, &subPUY);//找CU位置                                           		
			xEncIntraLoadRef(h, subPUX, subPUY, subPUWidth,uiDepth+1,RelativIdx,RelIdx[uiDepth]);
			estIntraPredQT(h,subPUWidth,uiDepth+1,ePartSize);
		    setCUPameters(CU_Cache,uiDepth+1 );
			xEncCacheStoreCU( h,subPUX,subPUY,subPUWidth,uiDepth+1);
		 }
		 subPUX = 0;
	  	 subPUY = 0;
		 xEncGetCUPosition(h,nCUSize, uiDepth,h->uiCUX,h->uiCUY, &subPUX, &subPUY);    //重新定位到上一深度的CU位置
		 CU_Cache->cuX = subPUX-h->uiCUX;   //在LCU中的相对位置
	     CU_Cache->cuY = subPUY-h->uiCUY;   //在LCU中的相对位置
		 xEncCheckBestMode(h,uiDepth+1,nCUSize);		 
		 xEncCacheStoreCU(h,subPUX,subPUY,subPUWidth<<1, uiDepth );
	 }
}

void xEncCheckBestMode(X265_t *h, UInt8  Depth, UInt32  uiCUWidth)
{
	if(Depth==0)
	{
		return;
	}
	
	X265_Cache  *pCache = &h->cache;
	Int     *  PartIdx              = pCache->PartIdx;
	Int     *  RelativeIdx          = pCache->RelativeIdx;
	UInt8   * cTempbCbfY            = pCache->TempbCbfY; 
	UInt8   * cTempbCbfU            = pCache->TempbCbfU; 
	UInt8   * cTempbCbfV            = pCache->TempbCbfV; 
	UInt8   * cTemppuhWidth         = pCache->TemppuhWidth;              
	UInt8   * cTemppuhHeight        = pCache->TemppuhHeight;             
	UInt8   * cTemppuhDepth         = pCache->TemppuhDepth;              
	UInt8   * cTemppePartSize       = pCache->TemppePartSize;           
	UInt8   * cTemppuhLumaIntraDir  = pCache->TemppuhLumaIntraDir;       
	UInt8   * cTemppuhChromaIntraDir = pCache->TemppuhChromaIntraDir;   
	double  * cTempTotalCost        = pCache->TempTotalCost;           
	double  * cTempTotalDistortion  = pCache->TempTotalDistortion;                     
	double   CurrTotalCost   = 0;
	double   LDepthTotalCost = 0;
	double   temp = 0;double   temp1 = 0;double   temp2 = 0;double   temp3 = 0;
	UInt32   uichromeWidth = uiCUWidth>>1;
	int      ipow    = int(pow(double(4),Depth));
	int      cTcount = ((pCache->NumPartition)/ipow)<<2;
	UInt     cT      = pCache->NumPartition/ipow;
	UInt32   stride  = (Depth-1)*pCache->NumPartition + RelativeIdx[Depth-1] , stride1 = (Depth*pCache->NumPartition)+ RelativeIdx[Depth-1];
  
	for(int i=0;i<4;i++)
	{	  
		temp          += cTempTotalCost[stride1+i*(cT)];
		temp1         += cTempTotalDistortion[stride1+i*(cT)];
	}
	CurrTotalCost   = temp;
	LDepthTotalCost = cTempTotalCost[stride];
	temp2           = cTempTotalCost[stride];
	temp3           = cTempTotalDistortion[stride];
	if(CurrTotalCost <LDepthTotalCost || cTemppePartSize[stride]== SIZE_NONE)
	{
		for(int i=0;i<cTcount;i++)
		{				
			cTempbCbfY[stride+i]            = cTempbCbfY[stride1+i];
			cTempbCbfU[stride+i]            = cTempbCbfU[stride1+i];
			cTempbCbfV[stride+i]            = cTempbCbfV[stride1+i];
			cTemppuhHeight[stride+i]        = cTemppuhHeight[stride1+i];
			cTemppuhWidth[stride+i]         = cTemppuhWidth[stride1+i];
			cTemppePartSize[stride+i]       = cTemppePartSize[stride1+i];
			cTemppuhDepth[stride+i]         = cTemppuhDepth[stride1+i];
			cTemppePartSize[stride+i]       = cTemppePartSize[stride1+i];
			cTemppuhLumaIntraDir[stride+i]  = cTemppuhLumaIntraDir[stride1+i];
			cTemppuhChromaIntraDir[stride+i] = cTemppuhChromaIntraDir[stride1+i];
			cTempTotalCost[stride+i]        = temp;
			cTempTotalDistortion[stride+i]  = temp1;
			
		}
		stride = (MAX_CU_SIZE)*(pCache->cuY)+ pCache->cuX;
		for(UInt32 i=0;i<uiCUWidth;i++)
		{
			memcpy(pCache->psTempCoefY[ Depth-1]+ stride + i*MAX_CU_SIZE, pCache->psTempCoefY[ Depth]+ stride +i*MAX_CU_SIZE,sizeof(Int16)*uiCUWidth);
			memcpy(pCache->pucTempRecY[ Depth-1]+ stride + i*MAX_CU_SIZE, pCache->pucTempRecY[ Depth]+ stride +i*MAX_CU_SIZE,sizeof(Pxl)*uiCUWidth);
		   
		}	
		stride = (MAX_CU_SIZE)*(pCache->cuY)/4+ pCache->cuX/2;
		for(UInt32 i=0;i<uichromeWidth;i++)
		{		
			memcpy(pCache->psTempCoefU[ Depth-1]+ stride + i*MAX_CU_SIZE/2, pCache->psTempCoefU[ Depth]+ stride +i*MAX_CU_SIZE/2,sizeof(Int16)*uichromeWidth);
			memcpy(pCache->psTempCoefV[ Depth-1]+ stride + i*MAX_CU_SIZE/2, pCache->psTempCoefV[ Depth]+ stride +i*MAX_CU_SIZE/2,sizeof(Int16)*uichromeWidth);
		    memcpy(pCache->pucTempRecU[ Depth-1]+ stride + i*MAX_CU_SIZE/2, pCache->pucTempRecU[ Depth]+ stride +i*MAX_CU_SIZE/2,sizeof(Pxl)*uichromeWidth);
		    memcpy(pCache->pucTempRecV[ Depth-1]+ stride + i*MAX_CU_SIZE/2, pCache->pucTempRecV[ Depth]+ stride +i*MAX_CU_SIZE/2,sizeof(Pxl)*uichromeWidth);
		}			
	}
	else
	{
		stride = ((Depth-1)*pCache->NumPartition)+RelativeIdx[Depth-1];
		int j=0;
		for(j=0;j<4-Depth;j++)
		{
			for(int i=0;i<cTcount;i++)
			{
				cTemppuhLumaIntraDir[((Depth+j)*pCache->NumPartition)+ RelativeIdx[Depth-1]+i]   = cTemppuhLumaIntraDir[(Depth-1)*pCache->NumPartition + RelativeIdx[Depth-1]+i];
				cTemppuhChromaIntraDir[((Depth+j)*pCache->NumPartition)+ RelativeIdx[Depth-1]+i] = cTemppuhChromaIntraDir[(Depth-1)*pCache->NumPartition + RelativeIdx[Depth-1]+i];
			}
		}
	}		 
}

void estIntraPredQT(X265_t *h,UInt32 nCUSize,UInt8 uiDepth ,X265_PartSize ePartSize)
{
	X265_Cache  *pCache  = &h->cache;
	const UInt nLog2CUSize = xLog2(nCUSize-1);
	Int  nQP              = h->iQP;
	Int  nQPC             = g_aucChromaScale[nQP];
	double   lambda       = pCache->Lambda;//nQP ;pCache->Lambda
	UInt8    *pucMostModeY= pCache->ucMostModeY;
	UInt8    *pCbfY       = &pCache->bCbfY;
	UInt8    *pCbfU       = &pCache->bCbfU;
	UInt8    *pCbfV       = &pCache->bCbfV;
	Pxl    *pucPixY     = pCache->pucPixY +(MAX_CU_SIZE)*(pCache->cuY) + pCache->cuX;
	Pxl    *pucRecY     = pCache->pucTempRecY[ uiDepth]+(MAX_CU_SIZE)*(pCache->cuY) + pCache->cuX;
	Pxl    *pucPredY    = pCache->pucPredY +(MAX_CU_SIZE)*(pCache->cuY) + pCache->cuX;
	Pxl    *pucPixC[2]  = { pCache->pucPixU+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2, pCache->pucPixV+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2 };
	Pxl    *pucRecC[2]  = { pCache->pucTempRecU[uiDepth]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2, pCache->pucTempRecV[uiDepth]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2 };
	Pxl    *pucPredC[2] = { pCache->pucPredC[0]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2, pCache->pucPredC[1]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2 };
	Int16    *piTmp0      = pCache->piTmp[0];
	Int16    *piTmp1      = pCache->piTmp[1];
	Int16    *piCoefY     = pCache->psTempCoefY[uiDepth]+(MAX_CU_SIZE)*(pCache->cuY) + pCache->cuX;
	Int16    *piCoefC[2]  = { pCache->psTempCoefU[uiDepth]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2, pCache->psTempCoefV[uiDepth]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2 };
	UInt8    *pucMostModeC= pCache->ucMostModeC;
	UInt8    *puhWidth    = &pCache->puhWidth;             
	UInt8    *puhHeight   = &pCache->puhHeight;             
	UInt8    *puhDepth    = &pCache->puhDepth;              
	UInt8    *puhLumaIntraDir = &pCache-> puhLumaIntraDir ;       
	UInt8    *puhChromaIntraDir = &pCache->puhChromaIntraDir;      
	double   *TotalCost   = &pCache->TotalCost;
	double   *BestSadC    = &pCache->SadC;//临时加的变量
	UInt     realModeC;
	UInt32   uiSumY, uiSumC[2];
	double   uiBestSadY, uiBestSadC;
	bool     flagC = false;
	double   distY = 0,distC = 0;
	UInt     nBestModeY, nBestModeC;
	UInt     nMode,nrealMode;
	uiBestSadY = MAX_SAD;
	nBestModeY = 0;
	UInt     CandMode[4] = {0,1,10,26};

	for( nrealMode=0; nrealMode<4; nrealMode++ ) 
	{
	    nMode = CandMode[nrealMode];
		double uiSad;
		xEncIntraPredLuma( h, nMode, nCUSize );

		if( nMode == pucMostModeY[0] )
		{   
			uiSad = 1 * lambda;
		}
		else if( nMode == pucMostModeY[1] || nMode == pucMostModeY[2] )
		{	
			uiSad = 2 * lambda;
		}
		else
		{	
			uiSad = lambda * 3;
		}
		uiSad += xSad_new(nCUSize, nCUSize, pucPixY,MAX_CU_SIZE, pucPredY,MAX_CU_SIZE );

		if( uiSad < uiBestSadY ) 
		{
			uiBestSadY = uiSad ;
			nBestModeY = nMode;		
		}
	}

	xEncIntraPredLuma( h, nBestModeY, nCUSize );
	xSubDct( piTmp0,
		pucPixY,
		pucPredY, MAX_CU_SIZE,
		piTmp0, piTmp1,
		nCUSize, nCUSize, nBestModeY );
	uiSumY = xQuant( piCoefY, piTmp0, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_I );

	uiBestSadY = 0;
	for(int i =0; i<nCUSize;i++)
	{
		for(int j =0;j<nCUSize;j++)
		{
	      uiBestSadY += abs(piCoefY[i * MAX_CU_SIZE + j]);
		}
	}

	if( uiSumY ) 
	{
		xDeQuant( piTmp0, piCoefY, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_I );
		
		xIDctAdd( pucRecY,
			piTmp0,
			pucPredY, MAX_CU_SIZE,
			piTmp1, piTmp0,
			nCUSize, nCUSize, nBestModeY );
	}
	
	else 
	{
		for(UInt32 i=0; i<nCUSize; i++ ) 
		{
			memcpy( &pucRecY[i*MAX_CU_SIZE], &pucPredY[i*MAX_CU_SIZE], sizeof(Pxl)*nCUSize );
		}
	}
	//求失真
	distY  = CalDistortion(pucPixY, pucRecY, MAX_CU_SIZE, nCUSize);
	*pCbfY = (uiSumY != 0);

	if((nCUSize>4)||(ePartSize==3 && pCache->PartIdx[uiDepth]== 0))//if((nCUSize>4)||(nCUSize==4&&pCache->PartIdx[uiDepth]==0))
	{		
		if(ePartSize == 3)
		{
			nCUSize = nCUSize<<1;
			flagC   =  true;
			UInt32  uisubCUX = 0;UInt32 uisubCUY = 0;
			xEncGetCUPosition(h,nCUSize,uiDepth-1, h->uiCUX, h->uiCUY,&uisubCUX, &uisubCUY);						
			Pxl    *pucPixC[2]  = { pCache->pucPixU+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2, pCache->pucPixV+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2 };
			Pxl    *pucRecC[2]  = { pCache->pucTempRecU[uiDepth]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2, pCache->pucTempRecV[uiDepth]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2 };
			Pxl    *pucPredC[2] = { pCache->pucPredC[0]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2, pCache->pucPredC[1]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2 };
			Int16    *piCoefC[2]  = { pCache->psTempCoefU[uiDepth]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2, pCache->psTempCoefV[uiDepth]+(MAX_CU_SIZE/2)*(pCache->cuY/2) + pCache->cuX/2 };						
		}
		pucMostModeC[0] = PLANAR_IDX;
		pucMostModeC[1] = VER_IDX;
		pucMostModeC[2] = HOR_IDX;
		pucMostModeC[3] = DC_IDX;
		pucMostModeC[4] = LM_CHROMA_IDX;
		pucMostModeC[5] = nBestModeY;

		for(int i=0;i<4; i++ ) 
		{
			if( pucMostModeC[i] == nBestModeY ) 
			{
				pucMostModeC[i] = 34;
				break;
			}
		}
		uiBestSadC = MAX_SAD;
		for( nMode=0; nMode<NUM_CHROMA_MODE; nMode++ ) 
		{
			UInt32 uiSumSad = 0;
			UInt32 uiSad[2] = {0};
			realModeC = pucMostModeC[nMode];

			if ( !h->bUseLMChroma && (realModeC == LM_CHROMA_IDX) )
				continue;

			xEncIntraPredChroma( h, realModeC, nCUSize >> 1);

			uiSad[0] = xSad_new(nCUSize / 2, nCUSize / 2, pucPixC[0],MAX_CU_SIZE/2, pucPredC[0],MAX_CU_SIZE/2 );
			uiSad[1] = xSad_new(nCUSize / 2, nCUSize / 2, pucPixC[1],MAX_CU_SIZE/2, pucPredC[1],MAX_CU_SIZE/2 );

			uiSumSad += uiSad[0] + uiSad[1];
			if( uiSumSad < uiBestSadC ) 
			{
				uiBestSadC = uiSumSad;
				nBestModeC = nMode;
						
			}
		}
			
		realModeC = pucMostModeC[nBestModeC];

		pCache->nBestModeY = nBestModeY;
		pCache->nBestModeC = nBestModeC;
           
		// Cr and Cb
		xEncIntraPredChroma( h, realModeC, nCUSize >> 1 );
			
		for(int i=0; i<2; i++ ) 
		{
			xSubDct( piTmp0,
						pucPixC[i],
						pucPredC[i], MAX_CU_SIZE/2,
						piTmp0, piTmp1,
						nCUSize/2, nCUSize/2, realModeC );

			uiSumC[i] = xQuant( piCoefC[i], piTmp0, MAX_CU_SIZE/2, nQPC, nCUSize/2, nCUSize/2, SLICE_I );
		}
            
		*pCbfU = (uiSumC[0] != 0);
		*pCbfV = (uiSumC[1] != 0);

		uiBestSadC = 0;
		for(int i =0; i<nCUSize/2;i++)
		{
			for(int j =0;j<nCUSize/2;j++)
			{
				uiBestSadC += abs(piCoefC[0][i *  MAX_CU_SIZE/2 + j]);
				uiBestSadC += abs(piCoefC[0][i *  MAX_CU_SIZE/2 + j]);
			}
		}

		// Cr and Cb
		for(int i=0; i<2; i++ ) 
		{
			if( uiSumC[i] ) 
			{
				xDeQuant( piTmp0, piCoefC[i], MAX_CU_SIZE/2, nQPC, nCUSize/2, nCUSize/2, SLICE_I );
				xIDctAdd( pucRecC[i],
							piTmp0,
							pucPredC[i], MAX_CU_SIZE/2,
							piTmp1, piTmp0,
							nCUSize/2, nCUSize/2, realModeC );
			}
			else 
			{
				UInt k;
				for( k=0; k<nCUSize/2; k++ ) 
				{
					memcpy( &pucRecC[i][k*MAX_CU_SIZE/2], &pucPredC[i][k*MAX_CU_SIZE/2], sizeof(Pxl)*nCUSize/2 );
				}
			}
		}	
		distC = CalDistortion(pucPixC[0], pucRecC[0], MAX_CU_SIZE/2, nCUSize/2);
		distC += CalDistortion(pucPixC[1], pucRecC[1], MAX_CU_SIZE/2, nCUSize/2);
		pCache->TotalDistortion  = distY + distC;
		pCache->TotalDistortionC = distC;

	} 
	else if(nCUSize == 4 && ePartSize == 3 && pCache->PartIdx[uiDepth]!= 0)
	{
		nCUSize = nCUSize<<1;
		flagC   =  true;
		nBestModeC = pCache->TemppuhChromaIntraDir[(uiDepth)*pCache->NumPartition+pCache->RelativeIdx[uiDepth-1]];
		uiBestSadC =  0;
		pCache->TotalDistortion = distY;
		distC = 0;
	}	
	//存储划分和预测信息			
	if(pCache->TemppePartSize[uiDepth*pCache->NumPartition+pCache->RelativeIdx[uiDepth]] == SIZE_NxN)
	{
	    *puhDepth = uiDepth-1;              
	}
	else
	{
		*puhDepth = uiDepth;			
	}
	if(flagC == true)
	{	*puhWidth  = nCUSize>>1;              
	    *puhHeight = nCUSize>>1;             
	}
	else
	{	*puhWidth  = nCUSize;              
	    *puhHeight = nCUSize;             
	}
    *puhLumaIntraDir   = nBestModeY;       
	*puhChromaIntraDir = nBestModeC;      
	*TotalCost         =(uiBestSadY + uiBestSadC) + (distY + distC)*1 + nQP*eBit[uiDepth]; //uiBestSadY+uiBestSadC;  //
	*BestSadC          = uiBestSadC;//临时加的变量
}

void xEncIntraLoadRef( X265_t *h, UInt32 uiX, UInt32 uiY, UInt32 nSize ,UInt8 uiDepth,UInt32 uiCurrPartUnitIdx,UInt32 uiAbsIdxInLCU)
 {
	X265_Cache  *pCache         = &h->cache;
	X265_Frame  *pFrameRec      = h->pFrameRec;  
	UInt16 Stride               = h->usWidth;
	UInt16 StrideC              = Stride>>1;
	pCache->cuX                 = uiX-h->uiCUX;   //在LCU中的相对位置
	pCache->cuY                 = uiY-h->uiCUY;   //在LCU中的相对位置
	const UInt   nMinTUSize      =  (1 << h->ucQuadtreeTULog2MinSize);
	const UInt32 uiOffset        =  pCache->uiOffset;
    Pxl *pucRefY0        =  pCache->pucPixRef[0];
    Pxl *pucRefY1        =  pCache->pucPixRef[1];
    Pxl *pucRefU         =  pCache->pucPixRefC[0];
    Pxl *pucRefV         =  pCache->pucPixRefC[1];
	UInt8 *pucTopModeY     =  (pCache->pucTopModeY)+(h->uiCUX)*(pCache->NumPartition)/(h->ucMaxCUWidth);
	UInt8 *pucLeftModeY    =  pCache->pucLeftModeY;
	UInt  uiNumPartInCUWidth = pCache->NumPartitionInWidth;
    UInt  uiNumPartition   = pCache->NumPartition;
	UInt8 * TemppuhLumaIntraDir   = pCache->TemppuhLumaIntraDir+uiNumPartition*uiDepth;
	UInt8 * TemppuhChromaIntraDir = pCache->TemppuhChromaIntraDir+uiNumPartition*uiDepth;
	Int   * PartIdx = pCache->PartIdx;
	Int   iLeftIntraDir, iAboveIntraDir;
	Int   RelativIdxUpRight,RelativIdxLeftBottom; //RelativIdxUpRight,RelativIdxLeft,RelativIdxUp
	UInt8 *pucMostModeY    =  pCache->ucMostModeY;
    UInt8 *pucMostModeY0   =  pCache->TempucMostModeY[0];
    UInt8 *pucMostModeY1   =  pCache->TempucMostModeY[1];
	UInt8 *pucMostModeY2   =  pCache->TempucMostModeY[2];

    const UInt   nSizeC        = (nSize >> 1);	
	Pxl *pucReferenceLuma    = (pFrameRec->pucY) + (Stride*uiY + uiX);  //指向重建帧亮度  
	Pxl *pucReferenceChroma0 = (pFrameRec->pucU) + (Stride*uiY/4 + uiX/2);  //指向重建帧色度
	Pxl *pucReferenceChroma1 = (pFrameRec->pucV) + (Stride*uiY/4 + uiX/2);  //指向重建帧色度
	bool   bT ;bool   bL ;bool   bLT ;bool   bTR ;bool   bLB ;
	//求出当前块的左块和上块预测模式 确定左块上块是否可用
	if(uiDepth == 0)
	{
		UInt dir       = uiNumPartition+g_auiRasterToZscan[(uiNumPartInCUWidth)*(uiNumPartInCUWidth-1)];
		iLeftIntraDir  = pucLeftModeY[5];
		iAboveIntraDir = pucTopModeY[10];
		bT             = (iAboveIntraDir != MODE_INVALID);
		bL             = (iLeftIntraDir != MODE_INVALID);
		bLT            = bT && bL;		
		bTR            = pucTopModeY[dir] != MODE_INVALID;
		bLB            = 0;
	}
	else
	{
		if(uiCurrPartUnitIdx == 0)
		{
			UInt dir       = g_auiRasterToZscan[(uiNumPartInCUWidth)*(uiNumPartInCUWidth-1)+ (nSize>>2)];
			iLeftIntraDir  = pucLeftModeY[5];
		    iAboveIntraDir = pucTopModeY[10];		
			bTR            = (pucTopModeY[dir] != MODE_INVALID);
			bT             = (iAboveIntraDir != MODE_INVALID);
		    bL             = (iLeftIntraDir != MODE_INVALID);
		    bLT            = bT && bL;
			dir            = g_auiRasterToZscan[uiNumPartInCUWidth*(nSize>>2)+uiNumPartInCUWidth-1];
		    bLB            = (pucLeftModeY[dir] != MODE_INVALID);	
		}
		else if(uiCurrPartUnitIdx==1||uiCurrPartUnitIdx==4||uiCurrPartUnitIdx==5||uiCurrPartUnitIdx==16||uiCurrPartUnitIdx==17||uiCurrPartUnitIdx==20||uiCurrPartUnitIdx==21)
		{
			iLeftIntraDir  = TemppuhLumaIntraDir[g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]-1]];
		    iAboveIntraDir = pucTopModeY[g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]+uiNumPartInCUWidth*(uiNumPartInCUWidth-1)]];			
			RelativIdxUpRight = g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]+ uiNumPartInCUWidth*(uiNumPartInCUWidth-1)+ (nSize>>2)];
		    RelativIdxLeftBottom = g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]+uiNumPartInCUWidth*(nSize>>2)-1];		
			bLB            = (TemppuhLumaIntraDir[RelativIdxLeftBottom]!=MODE_INVALID);
			bT             = (iAboveIntraDir != MODE_INVALID);
		    bL             = (iLeftIntraDir != MODE_INVALID);
		    bLT            = bT && bL;
			bTR            = g_auiZscanToRaster[uiCurrPartUnitIdx]+ uiNumPartInCUWidth*(uiNumPartInCUWidth-1)+ (nSize>>2)<uiNumPartition?(pucTopModeY[RelativIdxUpRight] != MODE_INVALID):(pucTopModeY[uiNumPartition+uiNumPartInCUWidth*(uiNumPartInCUWidth-1)] != MODE_INVALID);
		}
		else if(uiCurrPartUnitIdx==2||uiCurrPartUnitIdx==8||uiCurrPartUnitIdx==10||uiCurrPartUnitIdx==32||uiCurrPartUnitIdx==34||uiCurrPartUnitIdx==40||uiCurrPartUnitIdx==42)
		{
			iLeftIntraDir  = pucLeftModeY[g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]+uiNumPartInCUWidth-1]];
		    iAboveIntraDir = TemppuhLumaIntraDir[g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]-uiNumPartInCUWidth]];	
			RelativIdxUpRight = g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]-uiNumPartInCUWidth+(nSize>>2)];
		    RelativIdxLeftBottom = g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]+ uiNumPartInCUWidth*(1+(nSize>>2))-1];		
			bLB            =  g_auiZscanToRaster[uiCurrPartUnitIdx]+ uiNumPartInCUWidth*(1+(nSize>>2))-1<uiNumPartition?(pucLeftModeY[RelativIdxLeftBottom] != MODE_INVALID):0;	
			bT             = (iAboveIntraDir != MODE_INVALID);
		    bL             = (iLeftIntraDir  != MODE_INVALID);
		    bLT            = bT && bL;
		    bTR            = ( TemppuhLumaIntraDir[RelativIdxUpRight] != MODE_INVALID);    
		}
		else
		{
			iLeftIntraDir  = TemppuhLumaIntraDir[g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]-1]];
		    iAboveIntraDir = TemppuhLumaIntraDir[g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]-uiNumPartInCUWidth]];
			bLB            = g_auiZscanToRaster[uiCurrPartUnitIdx]+uiNumPartInCUWidth*(nSize>>2)-1<uiNumPartition?(TemppuhLumaIntraDir[g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]+uiNumPartInCUWidth*(nSize>>2)-1]] != MODE_INVALID):0;
			bT             = (iAboveIntraDir != MODE_INVALID);
		    bL             = (iLeftIntraDir  != MODE_INVALID);
		    bLT            = bT && bL;
		    bTR            = (g_auiZscanToRaster[uiCurrPartUnitIdx]+(nSize>>2))%uiNumPartInCUWidth!=0 ? (TemppuhLumaIntraDir[g_auiRasterToZscan[g_auiZscanToRaster[uiCurrPartUnitIdx]-uiNumPartInCUWidth*(nSize>>2)+(nSize>>2)]] != MODE_INVALID):0;	  
		}
	}
	if(uiX + nSize >= (h->usWidth+h->PAD_YSIZEw))
	   bTR = false;  
	if(uiY + nSize >= (h->usHeight+h->PAD_YSIZEh))
	   bLB = false; 
   
	const bool  bValid[5]       = {bLB, bL, bLT, bT, bTR};
	const UInt   nBlkOffsetY[6] = {0, nSize,  2*nSize,  2*nSize +1, 3*nSize +1, 4*nSize +1};
	const UInt   nBlkOffsetC[6] = {0, nSizeC, 2*nSizeC, 2*nSizeC+1, 3*nSizeC+1, 4*nSizeC+1};
	UInt i;UInt   nSize1;UInt   nSizeC1;

    memcpy( pCache->bValid, bValid, sizeof(bValid) );

    // Default to DC when all reference invalid
	if(pCache->TemppePartSize[uiDepth*(pCache->NumPartition)+uiCurrPartUnitIdx] == SIZE_2Nx2N) //if(nSize>4)
	{
		if( (bT | bL | bLT | bTR | bLB) == false )
		{
			memset( pucRefY0, 0x80, nSize  * 4 + 1 );
			memset( pucRefY1, 0x80, nSize  * 4 + 1 );
			memset( pucRefU,  0x80, nSizeC * 4 + 1 );
			memset( pucRefV,  0x80, nSizeC * 4 + 1 );
		}
		else 
		{
			// Copy the reconst pixel when valid
			if( bLB ) 
			{
				nSize1 = nSize;
				nSizeC1 = nSize1>>1;
				for( i=0; i<nSize1; i++ ) 
				{
					pucRefY0[nSize-1-i + nBlkOffsetY[0]] = pucReferenceLuma[nSize*Stride+i*Stride-1];
				}
				for(i=nSize1;i<nSize;i++)
				{
					pucRefY0[nSize-1-i + nBlkOffsetY[0]] = pucRefY0[nSize-nSize1 + nBlkOffsetY[0]];
				}
				for( i=0; i<nSizeC1; i++ ) 
				{
					pucRefU[nSizeC-1-i + nBlkOffsetC[0]] = pucReferenceChroma0[nSizeC*StrideC+i*StrideC-1];
					pucRefV[nSizeC-1-i + nBlkOffsetC[0]] = pucReferenceChroma1[nSizeC*StrideC+i*StrideC-1];
				}
				for( i=nSizeC1; i<nSizeC; i++ ) 
				{
					pucRefU[nSizeC-1-i + nBlkOffsetC[0]] = pucRefU[nSizeC-nSizeC1 + nBlkOffsetC[0]];
					pucRefV[nSizeC-1-i + nBlkOffsetC[0]] = pucRefV[nSizeC-nSizeC1 + nBlkOffsetC[0]];
				}
			}
			if( bL ) 
			{
				nSize1 = nSize;
				nSizeC1 = nSize1>>1;
				for( i=0; i<nSize1; i++ ) 
				{
					pucRefY0[nBlkOffsetY[1]+nSize-1-i] = pucReferenceLuma[i*Stride-1];
				}
				for( i=0; i<nSizeC1; i++ ) 
				{
					pucRefU[nBlkOffsetC[1]+nSizeC-1-i] = pucReferenceChroma0[i*StrideC-1];
					pucRefV[nBlkOffsetC[1]+nSizeC-1-i] = pucReferenceChroma1[i*StrideC-1]; 
				}
			}
			if( bLT ) 
			{
				pucRefY0[nBlkOffsetY[2]] = pucReferenceLuma[-1-Stride];
				pucRefU[nBlkOffsetC[2]] = pucReferenceChroma0[-1 -StrideC ];
				pucRefV[nBlkOffsetC[2]] = pucReferenceChroma1[-1-StrideC];
			}
			if( bT ) 
			{
				for( i=0; i<nSize; i++ ) 
				{
					pucRefY0[i + nBlkOffsetY[3]] = pucReferenceLuma[i-Stride];
				}
				for( i=0; i<nSizeC; i++ ) 
				{
					pucRefU[i + nBlkOffsetC[3]] = pucReferenceChroma0[i-StrideC];
					pucRefV[i + nBlkOffsetC[3]] = pucReferenceChroma1[i-StrideC];
				}
			}
			if( bTR ) 
			{
				for( i=0; i<nSize; i++ ) 
				{
					pucRefY0[i + nBlkOffsetY[4]] = pucReferenceLuma[nSize+i-Stride];
				}
				for( i=0; i<nSizeC; i++ ) 
				{
					pucRefU[i + nBlkOffsetC[4]] = pucReferenceChroma0[nSizeC-StrideC+i];
					pucRefV[i + nBlkOffsetC[4]] = pucReferenceChroma1[nSizeC-StrideC+i];
				}
			}
	
			xPaddingRef( pucRefY0, bValid, nBlkOffsetY );
			xPaddingRef( pucRefU,  bValid, nBlkOffsetC );
			xPaddingRef( pucRefV,  bValid, nBlkOffsetC );

			// Filter with [1 2 1]
			pucRefY1[0      ] = pucRefY0[0];
			pucRefY1[4*nSize] = pucRefY0[4*nSize];
			for(UInt16 i=1; i<4*nSize; i++ ) 
			{
				pucRefY1[i] = (pucRefY0[i - 1] + 2 * pucRefY0[i] + pucRefY0[i + 1] + 2) >> 2;
			}
		}	
	}
	else
	{	
		if( (bT | bL | bLT | bTR | bLB) == 0 ) 
		{
			memset( pucRefY0, 0x80, nSize  * 4 + 1 );
			memset( pucRefY1, 0x80, nSize  * 4 + 1 );      
        }
		else 
		{
			// Copy the reconst pixel when valid
			if( bLB ) 
			{
				for( i=0; i<nSize; i++ ) 
				{
					pucRefY0[nSize-1-i + nBlkOffsetY[0]] = pucReferenceLuma[nSize*Stride+i*Stride-1];
				}
			}
			if( bL ) 
			{
				for( i=0; i<nSize; i++ ) 
				{
					pucRefY0[nBlkOffsetY[1]+nSize-1-i] = pucReferenceLuma[i*Stride-1];
				}
           
			}
			if( bLT ) 
			{
				pucRefY0[nBlkOffsetY[2]] = pucReferenceLuma[-1-Stride];
           
			}
			if( bT ) 
			{
				for( i=0; i<nSize; i++ ) 
				{
					pucRefY0[i + nBlkOffsetY[3]] = pucReferenceLuma[i-Stride];
				}
           
			}
			if( bTR ) 
			{
				for( i=0; i<nSize; i++ ) 
				{
					pucRefY0[i + nBlkOffsetY[4]] = pucReferenceLuma[nSize+i-Stride];
				}
          
			}
	
			xPaddingRef( pucRefY0, bValid, nBlkOffsetY );
      
			// Filter with [1 2 1]
			pucRefY1[0      ] = pucRefY0[0];
			pucRefY1[4*nSize] = pucRefY0[4*nSize];
			for(UInt16 i=1; i<4*nSize; i++ ) 
			{
				pucRefY1[i] = (pucRefY0[i - 1] + 2 * pucRefY0[i] + pucRefY0[i + 1] + 2) >> 2;
			}
		}	
	}

    // Most Mode
    UInt8 ucLeftMode = bL        ? iLeftIntraDir : DC_IDX;
    UInt8 ucTopMode  = bT && uiY ? iAboveIntraDir : DC_IDX;

    if( ucLeftMode == ucTopMode ) 
	{
        if( ucLeftMode > 1 ) 
		{
            // angular modes
			pucMostModeY[0] = ucLeftMode;
            pucMostModeY[1] = ((ucLeftMode + 29) % 32) + 2;
            pucMostModeY[2] = ((ucLeftMode -  1) % 32) + 2;
            pucMostModeY0[uiCurrPartUnitIdx] = pucMostModeY[0];
            pucMostModeY1[uiCurrPartUnitIdx] = pucMostModeY[1];
            pucMostModeY2[uiCurrPartUnitIdx] = pucMostModeY[2];
        }
        else 
		{
            // non angular modes
			pucMostModeY[0] = PLANAR_IDX;
            pucMostModeY[1] = DC_IDX;
            pucMostModeY[2] = VER_IDX; 
            pucMostModeY0[uiCurrPartUnitIdx] = pucMostModeY[0];
            pucMostModeY1[uiCurrPartUnitIdx] = pucMostModeY[1];
            pucMostModeY2[uiCurrPartUnitIdx] = pucMostModeY[2]; 
        }
    }
    else 
	{
		pucMostModeY[0] = ucLeftMode;
        pucMostModeY[1] = ucTopMode;
        pucMostModeY0[uiCurrPartUnitIdx] = pucMostModeY[0];
        pucMostModeY1[uiCurrPartUnitIdx] = pucMostModeY[1];
        if( ucLeftMode && ucTopMode )
		{
			pucMostModeY[2] = PLANAR_IDX;
            pucMostModeY2[uiCurrPartUnitIdx] = pucMostModeY[2];
		}
        else
		{
			pucMostModeY[2] = ( ucLeftMode + ucTopMode ) < 2 ? VER_IDX : DC_IDX;
            pucMostModeY2[uiCurrPartUnitIdx] = pucMostModeY[2];
		}
    }
}

void setPartSizeSubParts( X265_Cache *pCache, X265_PartSize eMode, UInt uiAbsPartIdx, UInt8 uiDepth )
{
	UInt8 * TemppePartSize=pCache->TemppePartSize  ;
	int ipow = int(pow(double(4),uiDepth));
	Int NumPartition = (pCache->NumPartition);
	Int stride = NumPartition /ipow;
	memset( TemppePartSize+(pCache->NumPartition)*uiDepth+uiAbsPartIdx, eMode,stride*sizeof(UInt8));
}

void setPredModeSubParts( X265_Cache *pCache, X265_PredMode PredMode, UInt uiAbsPartIdx, UInt8 uiDepth )
{
	int ipow = int(pow(double(4),uiDepth));
	Int NumPartition = (pCache->NumPartition);
	Int stride = NumPartition /ipow;
	UInt8 * TemppePredMode=pCache->TemppePredMode  ;
	memset(TemppePredMode+(pCache->NumPartition)*uiDepth+uiAbsPartIdx,PredMode,stride*sizeof(UInt8));
}

void setCUPameters( X265_Cache *pCache, UInt8 uiDepth )
{
	Int RelativeIdx = pCache->RelativeIdx[uiDepth];
	UInt8 * TempbCbfY = pCache->TempbCbfY; 
	UInt8 * TempbCbfU = pCache->TempbCbfU; 
	UInt8 * TempbCbfV = pCache->TempbCbfV; 
	UInt8 * TemppuhWidth = pCache->TemppuhWidth;              //临时存储划分块的宽度 初始化时分配大小为NumPartition*(MaxDepth+1)*sizeof(char)
	UInt8 * TemppuhHeight = pCache->TemppuhHeight;             //临时存储划分块的高度    如上
	UInt8 * TemppuhDepth = pCache->TemppuhDepth;              //临时存储划分块的深度    如上
	UInt8 * TemppuhLumaIntraDir = pCache->TemppuhLumaIntraDir;       //临时存储划分块的亮度预测模式   如上
	UInt8 * TemppuhChromaIntraDir = pCache->TemppuhChromaIntraDir;      //临时存储划分块的色度预测模式   如上
	double * TempTotalCost = pCache->TempTotalCost;           //临时存储每一深度的cu的总Cost   初始化时分配大小为MaxDepth*sizeof(Double)
	double  * TempTotalDistortion = pCache->TempTotalDistortion;      //临时存储每一深度的cu的总失真
	UInt8  puhWidth = pCache->puhWidth;              //存储划分块的宽度 初始化时分配大小为NumPartitionsizeof(char)
	UInt8  puhHeight = pCache->puhHeight;             //存储划分块的高度    如上
	UInt8  puhDepth = pCache->puhDepth;              //存储划分块的深度    如上
	UInt8  puhLumaIntraDir= pCache-> puhLumaIntraDir;       //存储划分块的亮度预测模式   如上
	UInt8  puhChromaIntraDir= pCache->puhChromaIntraDir;      //存储划分块的色度预测模式   如上
	UInt8  bCbfY = pCache->bCbfY;
    UInt8  bCbfU = pCache->bCbfU;
	UInt8  bCbfV = pCache->bCbfV;
	double  TotalCost = pCache->TotalCost;
	double  TotalDistortion= pCache->TotalDistortion;
	double  BestC = pCache->SadC;

	int ipow = int(pow(double(4),uiDepth));
	Int NumPartition = (pCache->NumPartition);
	Int stride = NumPartition /ipow;
	Int paetIdx = pCache->PartIdx[uiDepth];
	Int relativeIdx =pCache->RelativeIdx[uiDepth];
	memset( TempbCbfY+NumPartition*uiDepth+ relativeIdx, bCbfY , sizeof(UInt8)*stride );
	memset( TempbCbfU+NumPartition*uiDepth+ relativeIdx, bCbfU , sizeof(UInt8)*stride );
    memset( TempbCbfV+NumPartition*uiDepth+ relativeIdx, bCbfV, sizeof(UInt8)*stride );
	memset( TemppuhWidth+NumPartition*uiDepth+ relativeIdx,  puhWidth, sizeof(UInt8)*stride );
	memset( TemppuhHeight+NumPartition*uiDepth+ relativeIdx,  puhWidth, sizeof(UInt8)*stride );
	memset( TemppuhDepth+NumPartition*uiDepth+ relativeIdx,  puhDepth, sizeof(UInt8)*stride );
	memset( TemppuhLumaIntraDir+NumPartition*uiDepth+ relativeIdx,  puhLumaIntraDir, sizeof(UInt8)*stride );
	memset( TemppuhChromaIntraDir+NumPartition*uiDepth+ relativeIdx,  puhChromaIntraDir, sizeof(UInt8)*stride );
	for(int i=0;i<stride;i++)
	{   pCache->TempTotalCost[NumPartition*uiDepth+ relativeIdx+i]= TotalCost;
	    pCache->TempTotalDistortion[NumPartition*uiDepth+ relativeIdx+i]= TotalDistortion;
    	pCache->TempSadC[NumPartition*uiDepth+ relativeIdx+i]= BestC;
	}
}