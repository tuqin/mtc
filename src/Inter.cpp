#include "MTC265.h"
#include "InterpolationFilter.h"

extern UInt g_auiRasterToZscan [MAX_PU_XY*MAX_PU_XY]; 
extern UInt g_auiZscanToRaster [MAX_PU_XY*MAX_PU_XY]; 
extern UInt g_motionRefer   [ MAX_NUM_SPU_W*MAX_NUM_SPU_W ];

void xEncInterCheckRDCost(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, double& totalPUbestSAD, Int32* PURelIdxForCompare)
{
	MTC265_Cache* CU_Cache = &h->cache;
	UInt32 nCUSize = h->ucMaxCUWidth>>uiDepth;
	Int32* CURelIdx = CU_Cache->RelativeIdx;
	Int32* PUPartIdx = CU_Cache->PUPartIdx;
	Int32* PURelIdx = CU_Cache->PURelativeIdx;
	UInt32 subPUX = 0;
	UInt32 subPUY = 0;

	Interpara *StartTempInter = h->TempInter;
	Interpara *StartChange = 0;
	Int32 uiLCUIdx = h->uiCUX/h->ucMaxCUWidth + (((h->usWidth+h->PAD_YSIZEw)/h->ucMaxCUWidth) * (h->uiCUY/h->ucMaxCUWidth));
	Int32 TempInterToCur = 0;
	Int32 tempBinNumberCost = 0;
	Int32 totalBinNumber = 0;
	Int32 BinNumber_X = 0;
	Int32 BinNumber_Y = 0;
	UInt32 totalCoeffNxNBinEst = 0;
	double PUbestSAD = 0;
	double MEandMergeBestSAD = 0;
	double MECost = 0;
	totalPUbestSAD = 0;

	if(ePartSize == SIZE_2Nx2N)
	{
		PURelIdx[uiDepth] = CURelIdx[uiDepth];
		PURelIdxForCompare[0] = PURelIdx[uiDepth];//代表当前PU的RelativeIdx

		CU_Cache->subPUX = uiCUX;
		CU_Cache->subPUY = uiCUY;

		//得到正在计算的PU的offset
		TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*CU_Cache->NumPartition*h->PUnumber + uiDepth*CU_Cache->NumPartition*h->PUnumber + PURelIdx[uiDepth]*h->PUnumber + (Int32)ePartSize;
		//以下的变量由四元组(uiLCUIdx, uiDepth, PUrelIdx, ePartSize)决定
		StartChange = StartTempInter + TempInterToCur;
		InterPrediction( h, StartChange, SIZE_2Nx2N, uiDepth, uiCUX, uiCUY, PUbestSAD);//数据都存储在StartChange中

		calcMvdBinNumber(StartChange->iMvdBestY.m_x, BinNumber_X);
		calcMvdBinNumber(StartChange->iMvdBestY.m_y, BinNumber_Y);
		totalPUbestSAD = PUbestSAD + h->sqrt_Inter_lamada*(BinNumber_X + BinNumber_Y);;
	}
	else if (ePartSize == SIZE_2NxN)
	{
		for(int PUpartindex=0; PUpartindex < 2;PUpartindex++)
		{
			subPUX = 0;//当前PU左上角点的位置
			subPUY = 0;	
			PUPartIdx[uiDepth] = PUpartindex;
			PURelIdx[uiDepth] = CURelIdx[uiDepth] + PUpartindex*((CU_Cache->NumPartition>>(2*uiDepth))/2);//当前PU的RelativeIdx，此循环中会发生变化
			PURelIdxForCompare[PUpartindex] = PURelIdx[uiDepth];//代表当前PU的RelativeIdx

			//得到正在计算的PU的offset
			TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*CU_Cache->NumPartition*h->PUnumber + uiDepth*CU_Cache->NumPartition*h->PUnumber + PURelIdx[uiDepth]*h->PUnumber + (Int32)ePartSize;
			//以下的变量由四元组(uiLCUIdx, uiDepth, PUrelIdx, ePartSize)决定
			StartChange = StartTempInter + TempInterToCur;

			xEncGetPUPosition( h, SIZE_2NxN, uiDepth, PUpartindex, &subPUX, &subPUY );//得到subPUX和subPUY，确定当前PU左上角点的位置 
			CU_Cache->subPUX = subPUX;
			CU_Cache->subPUY = subPUY;
			//AMVP+Merge
			InterPrediction( h, StartChange, SIZE_2NxN, uiDepth, subPUX, subPUY, PUbestSAD);

			calcMvdBinNumber(StartChange->iMvdBestY.m_x, BinNumber_X);
			calcMvdBinNumber(StartChange->iMvdBestY.m_y, BinNumber_Y);
			MECost = PUbestSAD + h->sqrt_Inter_lamada*(BinNumber_X + BinNumber_Y);;
			//Merge
			xCheckRDCostMerge(h, SIZE_2NxN, uiDepth, subPUX, subPUY, MECost, MEandMergeBestSAD, tempBinNumberCost);
			//Copy到Ref中，进行变换R-D比较
			MotionCompensation( h, StartChange->iMvBestY, StartChange->iSrchCenBest.m_x, StartChange->iSrchCenBest.m_y, uiDepth, ePartSize, PUpartindex, nCUSize, nCUSize>>1);
			totalBinNumber += tempBinNumberCost;
		}
		//变换得到Distortion
		xEncInterDistortion( h, ePartSize, uiDepth, uiCUX, uiCUY, totalCoeffNxNBinEst, PUbestSAD);
		totalPUbestSAD = PUbestSAD + h->Inter_lamada * (totalCoeffNxNBinEst + totalBinNumber);
	}
	else if (ePartSize == SIZE_Nx2N)
	{
		for(int PUpartindex=0; PUpartindex < 2;PUpartindex++)
		{
			subPUX = 0;//当前PU左上角点的位置
			subPUY = 0;	
			PUPartIdx[uiDepth] = PUpartindex;
			PURelIdx[uiDepth] = CURelIdx[uiDepth] + PUpartindex*((CU_Cache->NumPartition>>(2*uiDepth))/4);//当前PU的RelativeIdx，此循环中会发生变化
			PURelIdxForCompare[PUpartindex] = PURelIdx[uiDepth];//代表当前PU的RelativeIdx

			//得到正在计算的PU的offset
			TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*CU_Cache->NumPartition*h->PUnumber + uiDepth*CU_Cache->NumPartition*h->PUnumber + PURelIdx[uiDepth]*h->PUnumber + (Int32)ePartSize;
			//以下的变量由四元组(uiLCUIdx, uiDepth, PUrelIdx, ePartSize)决定
			StartChange = StartTempInter + TempInterToCur;

			xEncGetPUPosition( h, SIZE_Nx2N, uiDepth, PUpartindex, &subPUX, &subPUY );//得到subPUX和subPUY，确定当前PU左上角点的位置 
			CU_Cache->subPUX = subPUX;
			CU_Cache->subPUY = subPUY;
			//AMVP+ME
			InterPrediction( h, StartChange, SIZE_Nx2N, uiDepth, subPUX, subPUY, PUbestSAD);

			calcMvdBinNumber(StartChange->iMvdBestY.m_x, BinNumber_X);
			calcMvdBinNumber(StartChange->iMvdBestY.m_y, BinNumber_Y);
			MECost = PUbestSAD + h->sqrt_Inter_lamada*(BinNumber_X + BinNumber_Y);;
			//Merge
			xCheckRDCostMerge(h, SIZE_Nx2N, uiDepth, subPUX, subPUY, MECost, MEandMergeBestSAD, tempBinNumberCost);
			//Copy到Ref中，进行变换R-D比较
			MotionCompensation( h, StartChange->iMvBestY, StartChange->iSrchCenBest.m_x, StartChange->iSrchCenBest.m_y, uiDepth, ePartSize, PUpartindex, nCUSize>>1, nCUSize);
			totalBinNumber += tempBinNumberCost;
		}
		//变换得到Distortion
		xEncInterDistortion( h, ePartSize, uiDepth, uiCUX, uiCUY, totalCoeffNxNBinEst, PUbestSAD);
		totalPUbestSAD = PUbestSAD + h->Inter_lamada * (totalCoeffNxNBinEst + totalBinNumber);
	}
}

void xEncInterDistortion(MTC265_t* h, MTC265_PartSize ePartSize, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, UInt32& totalCoeffNxNBinEst, double& InterDistortion)
{
	MTC265_Cache* pCache = &h->cache;
	pCache->subPUX = uiCUX;
	pCache->subPUY = uiCUY;
	UInt32 subTranCUX = 0;
	UInt32 subTranCUY = 0;
	UInt8 nextDepth = uiDepth + 1;
	UInt32 nCUSize = h->ucMaxCUWidth>>uiDepth;
	Int* RelIdx = pCache->RelativeIdx;
	Int32 CURelIdx = RelIdx[uiDepth];
	UInt32 CoeffNxNBinEst = 0;
	double InterDistortionLuma = 0;
	double InterDistortionChroma_U = 0;
	double InterDistortionChroma_V = 0;

	totalCoeffNxNBinEst = 0;
	if ((ePartSize == SIZE_2NxN)||(ePartSize == SIZE_Nx2N))
	{
		for(int i = 0; i<4 ; i++)//针对将非方形PU拆分成2部分
		{
			subTranCUX = uiCUX;
			subTranCUY = uiCUY;
			if(i==0)//得到4个变换块左上角点的坐标
			{
				subTranCUX += (nCUSize>>2)*0 ;
				subTranCUY += (nCUSize>>2)*0 ;
				pCache->RelativeIdx[nextDepth] = CURelIdx + i*(int)pow(double(4),2-uiDepth); 
			}
			else if(i==1)
			{
				subTranCUX += (nCUSize>>2)*1 ;
				subTranCUY += (nCUSize>>2)*0 ;
				pCache->RelativeIdx[nextDepth] = CURelIdx + i*(int)pow(double(4),2-uiDepth); 
			}
			else if(i==2)
			{
				subTranCUX += (nCUSize>>2)*0 ;
				subTranCUY += (nCUSize>>2)*1 ;
				pCache->RelativeIdx[nextDepth] = CURelIdx + i*(int)pow(double(4),2-uiDepth); 
			}
			else if(i==3)
			{
				subTranCUX += (nCUSize>>2)*1 ;
				subTranCUY += (nCUSize>>2)*1 ;
				pCache->RelativeIdx[nextDepth] = CURelIdx + i*(int)pow(double(4),2-uiDepth); 
			}
			if (uiDepth == 2)
			{
				//只能做亮度的拆分，色度需要组合起来，因此变换量化只能对亮度进行
				estInterPredQTLumaWithCoeffBin(h, nCUSize>>1, uiDepth, pCache->RelativeIdx[nextDepth], subTranCUX, subTranCUY, CoeffNxNBinEst );
				totalCoeffNxNBinEst += CoeffNxNBinEst;
			}
			else
			{
				estInterPredQTWithCoeffBin(h, nCUSize>>1, uiDepth, pCache->RelativeIdx[nextDepth], subTranCUX, subTranCUY, CoeffNxNBinEst );
				totalCoeffNxNBinEst += CoeffNxNBinEst;
			}				
		}
		if (uiDepth == 2)
		{
			//nextDepth层色度的变换、量化
			estInterPredQTChromaWithCoeffBin(h, nCUSize, uiDepth, CURelIdx, uiCUX, uiCUY, CoeffNxNBinEst );
			totalCoeffNxNBinEst += CoeffNxNBinEst;
		}
	}
	else
	{
		estInterPredQTWithCoeffBin(h, nCUSize, uiDepth, CURelIdx, uiCUX, uiCUY, CoeffNxNBinEst );
		totalCoeffNxNBinEst += CoeffNxNBinEst;
	}
	InterDistortionLuma = CalInterDistortion(pCache->pucPixY + uiCUY * MAX_CU_SIZE + uiCUX, pCache->pucTempBinRecY[ uiDepth]+MAX_CU_SIZE*uiCUY + uiCUX, MAX_CU_SIZE, MAX_CU_SIZE, nCUSize, ePartSize );
	InterDistortionChroma_U = CalInterDistortion(pCache->pucPixU + uiCUY/2 * MAX_CU_SIZE/2 + uiCUX/2, pCache->pucTempBinRecU[ uiDepth]+MAX_CU_SIZE/2*uiCUY/2 + uiCUX/2, MAX_CU_SIZE/2, MAX_CU_SIZE/2, nCUSize/2, ePartSize );
	InterDistortionChroma_V = CalInterDistortion(pCache->pucPixV + uiCUY/2 * MAX_CU_SIZE/2 + uiCUX/2, pCache->pucTempBinRecV[ uiDepth]+MAX_CU_SIZE/2*uiCUY/2 + uiCUX/2, MAX_CU_SIZE/2, MAX_CU_SIZE/2, nCUSize/2, ePartSize );
	InterDistortion = InterDistortionLuma + InterDistortionChroma_U + InterDistortionChroma_V;
}

void xEncGetPUPosition( MTC265_t* h, MTC265_PartSize ePartSize, UInt8 uiDepth, Int32 PUpartIdx, UInt32* uisubPUX, UInt32* uisubPUY)
{  
	MTC265_Cache *pCache = &h->cache;
	if(ePartSize == SIZE_2Nx2N)
	{
		pCache->subPUWidth  = h->ucMaxCUWidth>>uiDepth;
		pCache->subPUHeight = h->ucMaxCUWidth>>uiDepth;
		*uisubPUX = pCache->cuX;
		*uisubPUY = pCache->cuY;
	}
	else if(ePartSize == SIZE_2NxN)
	{
		pCache->subPUWidth  = h->ucMaxCUWidth>>uiDepth;
		pCache->subPUHeight = h->ucMaxCUWidth>>(uiDepth+1);
		*uisubPUX = pCache->cuX + 0 * pCache->subPUWidth;
		*uisubPUY = pCache->cuY + PUpartIdx * pCache->subPUHeight;
	}
	else if(ePartSize == SIZE_Nx2N)
	{
		pCache->subPUWidth  = h->ucMaxCUWidth>>(uiDepth+1);
		pCache->subPUHeight = h->ucMaxCUWidth>>uiDepth;
		*uisubPUX = pCache->cuX + PUpartIdx * pCache->subPUWidth;
		*uisubPUY = pCache->cuY + 0 * pCache->subPUHeight;
	}
}

void estInterPredQTChroma( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY)
{
	MTC265_Cache *pCache = &h->cache;
	Int nQP = h->iQP;
	Int nQPC = g_aucChromaScale[nQP];
	UInt32 uiSumC[2];
	double distC=0;
	UInt wbitC=0;
	UInt8* pCbfU = pCache->TempbCbfU + uiDepth* pCache->NumPartition + PURelIdx;
	UInt8* pCbfV = pCache->TempbCbfV + uiDepth* pCache->NumPartition + PURelIdx;
	Pxl* pucInterPredC[2] = { pCache->TemppuhRefU + uiDepth * MAX_CU_SIZE/2 * MAX_CU_SIZE/2 + subCUY/2 * MAX_CU_SIZE/2 + subCUX/2, pCache->TemppuhRefV + uiDepth * MAX_CU_SIZE/2 * MAX_CU_SIZE/2 + subCUY/2 * MAX_CU_SIZE/2 + subCUX/2};
	Pxl* pucInterPixC[2]  = { pCache->pucPixU + MAX_CU_SIZE/2 *(subCUY/2) + subCUX/2, pCache->pucPixV + MAX_CU_SIZE/2 *(subCUY/2) + subCUX/2 };
	Pxl* pucInterRecC[2]  = { pCache->pucTempRecU[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2, pCache->pucTempRecV[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2 };
	Int16* piInterTmp0      = pCache->piTmp[0];
	Int16* piInterTmp1      = pCache->piTmp[1];
	Int16* piInterCoefC[2]  = { pCache->psTempCoefU[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2, pCache->psTempCoefV[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2 };

	Int32 copyCURelIdx = 0;
	Int32 copyFromCUOffset = 0;
	Int32 copyToCUOffset = 0;
	for(int i=0; i<2; i++ ) 
	{
		xSubDct( piInterTmp0, pucInterPixC[i], pucInterPredC[i], MAX_CU_SIZE/2, piInterTmp0, piInterTmp1, nCUSize/2, nCUSize/2, MODE_INVALID );
		for (UInt32 i1=0; i1<nCUSize/2;i1++)
		{
			for (UInt32 j=0; j<nCUSize/2;j++)
			{
				wbitC += abs(piInterCoefC[i][i1*(MAX_CU_SIZE/2)+j]);
			}
		}
		uiSumC[i] = xQuant( piInterCoefC[i], piInterTmp0, MAX_CU_SIZE/2, nQPC, nCUSize/2, nCUSize/2, SLICE_P);
	}
	*pCbfU = (uiSumC[0] != 0);
	*pCbfV = (uiSumC[1] != 0);

	// Cr and Cb
	for(int i=0; i<2; i++ ) 
	{
		if( uiSumC[i] ) 
		{
			xDeQuant( piInterTmp0, piInterCoefC[i], (MAX_CU_SIZE/2), nQPC, nCUSize/2, nCUSize/2, SLICE_P );
			xIDctAdd( pucInterRecC[i], piInterTmp0, pucInterPredC[i], MAX_CU_SIZE/2 ,piInterTmp1, piInterTmp0, nCUSize/2, nCUSize/2, MODE_INVALID);
		}
		else 
		{
			UInt k;
			for( k=0; k<nCUSize/2; k++ ) 
			{
				memcpy( &pucInterRecC[i][k*(MAX_CU_SIZE/2)], &pucInterPredC[i][k*(MAX_CU_SIZE/2)], sizeof(Pxl)*nCUSize/2);
			}
		}
	}

	//将PU内的全部StartChange变量均赋值为同样的值
	for (UInt i = 0; i < nCUSize/4;i++)
	{
		for (UInt j = 0; j < nCUSize/4;j++)
		{
			copyCURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx] + i * (h->ucMaxCUWidth/4) + j];
			copyFromCUOffset = uiDepth*pCache->NumPartition + PURelIdx;
			copyToCUOffset = uiDepth*pCache->NumPartition + copyCURelIdx;

			pCache->TempbCbfU[copyToCUOffset] = pCache->TempbCbfU[copyFromCUOffset];
			pCache->TempbCbfV[copyToCUOffset] = pCache->TempbCbfV[copyFromCUOffset];
		}
	}
}

void estInterPredQTWithCoeffBin( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY, UInt32& CoeffNxNBinEst)
{
	UInt32 CoeffNxNBinEstLuma = 0;
	UInt32 CoeffNxNBinEstChroma = 0;
	estInterPredQTLumaWithCoeffBin( h, nCUSize, uiDepth, PURelIdx, subCUX, subCUY, CoeffNxNBinEstLuma);
	estInterPredQTChromaWithCoeffBin( h, nCUSize, uiDepth, PURelIdx, subCUX, subCUY, CoeffNxNBinEstChroma);
	CoeffNxNBinEst = CoeffNxNBinEstLuma + CoeffNxNBinEstChroma;
}

void estInterPredQTChromaWithCoeffBin( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY, UInt32& CoeffNxNBinEstChroma)
{
	MTC265_Cache *pCache = &h->cache;
	Int nQP = h->iQP;
	Int nQPC = g_aucChromaScale[nQP];
	UInt32 uiSumC[2];
	double distC=0;
	UInt wbitC=0;
	Pxl* pucInterPredC[2] = { pCache->TemppuhRefU + uiDepth * MAX_CU_SIZE/2 * MAX_CU_SIZE/2 + subCUY/2 * MAX_CU_SIZE/2 + subCUX/2, pCache->TemppuhRefV + uiDepth * MAX_CU_SIZE/2 * MAX_CU_SIZE/2 + subCUY/2 * MAX_CU_SIZE/2 + subCUX/2};
	Pxl* pucInterPixC[2]  = { pCache->pucPixU + MAX_CU_SIZE/2 *(subCUY/2) + subCUX/2, pCache->pucPixV + MAX_CU_SIZE/2 *(subCUY/2) + subCUX/2 };
	Pxl* pucInterRecC[2]  = { pCache->pucTempBinRecU[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2, pCache->pucTempBinRecV[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2 };
	Int16* piInterTmp0      = pCache->piTmp[0];
	Int16* piInterTmp1      = pCache->piTmp[1];
	Int16* piInterCoefC[2]  = { pCache->psTempBinCoefU[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2, pCache->psTempBinCoefV[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2 };

	UInt32 CoeffNxNBinEstChroma_U = 0;
	UInt32 CoeffNxNBinEstChroma_V = 0;
	for(int i=0; i<2; i++ ) 
	{
		xSubDct( piInterTmp0, pucInterPixC[i], pucInterPredC[i], MAX_CU_SIZE/2, piInterTmp0, piInterTmp1, nCUSize/2, nCUSize/2, MODE_INVALID );
		for (UInt32 i1=0; i1<nCUSize/2;i1++)
		{
			for (UInt32 j=0; j<nCUSize/2;j++)
			{
				wbitC += abs(piInterCoefC[i][i1*(MAX_CU_SIZE/2)+j]);
			}
		}
		uiSumC[i] = xQuant( piInterCoefC[i], piInterTmp0, MAX_CU_SIZE/2, nQPC, nCUSize/2, nCUSize/2, SLICE_P);
	}

	// Cr and Cb
	for(int i=0; i<2; i++ ) 
	{
		if( uiSumC[i] ) 
		{
			xDeQuant( piInterTmp0, piInterCoefC[i], (MAX_CU_SIZE/2), nQPC, nCUSize/2, nCUSize/2, SLICE_P );
			xIDctAdd( pucInterRecC[i], piInterTmp0, pucInterPredC[i], MAX_CU_SIZE/2 ,piInterTmp1, piInterTmp0, nCUSize/2, nCUSize/2, MODE_INVALID);
		}
		else 
		{
			UInt k;
			for( k=0; k<nCUSize/2; k++ ) 
			{
				memcpy( &pucInterRecC[i][k*(MAX_CU_SIZE/2)], &pucInterPredC[i][k*(MAX_CU_SIZE/2)], sizeof(Pxl)*nCUSize/2);
			}
		}
	}
	CoeffNxNBinEstChroma_U = xEncodeCoeffNxNBinEst(h, pCache, piInterCoefC[0], PURelIdx, nCUSize/2, FALSE, MODE_INVALID );
	CoeffNxNBinEstChroma_V = xEncodeCoeffNxNBinEst(h, pCache, piInterCoefC[1], PURelIdx, nCUSize/2, FALSE, MODE_INVALID );
	CoeffNxNBinEstChroma = CoeffNxNBinEstChroma_U + CoeffNxNBinEstChroma_V;
}

void estInterPredQTLumaWithCoeffBin( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY, UInt32& CoeffNxNBinEst)
{
	MTC265_Cache *pCache = &h->cache;
	Int nQP = h->iQP;
	UInt32 uiSumY;
	double distY=0;
	UInt wbitY=0;
	Pxl* pucInterPredY = pCache->TemppuhRefY + uiDepth * MAX_CU_SIZE * MAX_CU_SIZE + subCUY * MAX_CU_SIZE + subCUX;
	Pxl* pucInterPixY = pCache->pucPixY + MAX_CU_SIZE *(subCUY) + subCUX;
	Pxl* pucInterRecY = pCache->pucTempBinRecY[ uiDepth]+(MAX_CU_SIZE)*(subCUY) + subCUX;
	Int16* piInterTmp0 = pCache->piTmp[0];
	Int16* piInterTmp1 = pCache->piTmp[1];
	Int16* piInterCoefY = pCache->psTempBinCoefY[uiDepth]+(MAX_CU_SIZE)*(subCUY) + subCUX;

	xSubDct( piInterTmp0, pucInterPixY , pucInterPredY, MAX_CU_SIZE, piInterTmp0, piInterTmp1, nCUSize, nCUSize, MODE_INVALID );
	uiSumY = xQuant( piInterCoefY, piInterTmp0, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_P );

	if( uiSumY ) 
	{
		xDeQuant( piInterTmp0, piInterCoefY, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_P );
		for (UInt32 i=0; i<nCUSize;i++)
		{
			for (UInt32 j=0; j<nCUSize;j++)
			{
				wbitY += abs(piInterCoefY[i*MAX_CU_SIZE+j]);
			}	
		}
		xIDctAdd( pucInterRecY, piInterTmp0, pucInterPredY, MAX_CU_SIZE, piInterTmp1, piInterTmp0, nCUSize, nCUSize, MODE_INVALID  );
	}
	else 
	{
		for(UInt32 i=0; i<nCUSize; i++ ) 
		{
			memcpy( &pucInterRecY[i*MAX_CU_SIZE], &pucInterPredY[i*MAX_CU_SIZE], sizeof(Pxl)*nCUSize );
		}
	}
	CoeffNxNBinEst = xEncodeCoeffNxNBinEst(h, pCache, piInterCoefY, PURelIdx, nCUSize, TRUE, MODE_INVALID );
}

void estInterPredQTLuma( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY)
{
	MTC265_Cache *pCache = &h->cache;
	Int nQP = h->iQP;
	UInt32 uiSumY;
	double distY=0;
	UInt wbitY=0;
	UInt8* pCbfY = pCache->TempbCbfY + uiDepth* pCache->NumPartition + PURelIdx;
	Pxl* pucInterPredY = pCache->TemppuhRefY + uiDepth * MAX_CU_SIZE * MAX_CU_SIZE + subCUY * MAX_CU_SIZE + subCUX;
	Pxl* pucInterPixY = pCache->pucPixY + MAX_CU_SIZE *(subCUY) + subCUX;
	Pxl* pucInterRecY = pCache->pucTempRecY[ uiDepth]+(MAX_CU_SIZE)*(subCUY) + subCUX;
	Int16* piInterTmp0 = pCache->piTmp[0];
	Int16* piInterTmp1 = pCache->piTmp[1];
	Int16* piInterCoefY = pCache->psTempCoefY[uiDepth]+(MAX_CU_SIZE)*(subCUY) + subCUX;

	Int32 copyCURelIdx = 0;
	Int32 copyFromCUOffset = 0;
	Int32 copyToCUOffset = 0;

	xSubDct( piInterTmp0, pucInterPixY , pucInterPredY, MAX_CU_SIZE, piInterTmp0, piInterTmp1, nCUSize, nCUSize, MODE_INVALID );
	uiSumY = xQuant( piInterCoefY, piInterTmp0, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_P );

	if( uiSumY ) 
	{
		xDeQuant( piInterTmp0, piInterCoefY, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_P );
		for (UInt32 i=0; i<nCUSize;i++)
		{
			for (UInt32 j=0; j<nCUSize;j++)
			{
				wbitY += abs(piInterCoefY[i*MAX_CU_SIZE+j]);
			}	
		}
		xIDctAdd( pucInterRecY, piInterTmp0, pucInterPredY, MAX_CU_SIZE, piInterTmp1, piInterTmp0, nCUSize, nCUSize, MODE_INVALID  );
	}
	else 
	{
		for(UInt32 i=0; i<nCUSize; i++ ) 
		{
			memcpy( &pucInterRecY[i*MAX_CU_SIZE], &pucInterPredY[i*MAX_CU_SIZE], sizeof(Pxl)*nCUSize );
		}
	}
	*pCbfY = (uiSumY != 0);

	//将PU内的全部StartChange变量均赋值为同样的值
	for (UInt i = 0; i < nCUSize/4;i++)
	{
		for (UInt j = 0; j < nCUSize/4;j++)
		{
			copyCURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx] + i * (h->ucMaxCUWidth/4) + j];
			copyFromCUOffset = uiDepth*pCache->NumPartition + PURelIdx;
			copyToCUOffset = uiDepth*pCache->NumPartition + copyCURelIdx;

			pCache->TempbCbfY[copyToCUOffset] = pCache->TempbCbfY[copyFromCUOffset];
		}
	}
}

void estInterPredQT( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY)
{
	MTC265_Cache* pCache = &h->cache;

	Int nQP = h->iQP;
	Int nQPC = g_aucChromaScale[nQP];
	UInt32 uiSumY, uiSumC[2];
	double distY=0,distC=0;
	UInt wbitY=0,wbitC=0;

	UInt8* pCbfY = pCache->TempbCbfY + uiDepth* pCache->NumPartition + PURelIdx;
	UInt8* pCbfU = pCache->TempbCbfU + uiDepth* pCache->NumPartition + PURelIdx;
	UInt8* pCbfV = pCache->TempbCbfV + uiDepth* pCache->NumPartition + PURelIdx;
	Pxl* pucInterPredY = pCache->TemppuhRefY + uiDepth * MAX_CU_SIZE * MAX_CU_SIZE + subCUY * MAX_CU_SIZE + subCUX;
	Pxl* pucInterPredC[2] = { pCache->TemppuhRefU + uiDepth * MAX_CU_SIZE/2 * MAX_CU_SIZE/2 + subCUY/2 * MAX_CU_SIZE/2 + subCUX/2,pCache->TemppuhRefV + uiDepth * MAX_CU_SIZE/2 * MAX_CU_SIZE/2 + subCUY/2 * MAX_CU_SIZE/2 + subCUX/2};
	Pxl* pucInterPixY = pCache->pucPixY + MAX_CU_SIZE *(subCUY) + subCUX;
	Pxl* pucInterRecY = pCache->pucTempRecY[ uiDepth]+(MAX_CU_SIZE)*(subCUY) + subCUX;
	Pxl* pucInterPixC[2]  = { pCache->pucPixU + MAX_CU_SIZE/2 *(subCUY/2) + subCUX/2, pCache->pucPixV + MAX_CU_SIZE/2 *(subCUY/2) + subCUX/2 };
	Pxl* pucInterRecC[2]  = { pCache->pucTempRecU[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2, pCache->pucTempRecV[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2 };
	Int16* piInterTmp0      = pCache->piTmp[0];
	Int16* piInterTmp1      = pCache->piTmp[1];
	Int16* piInterCoefY     = pCache->psTempCoefY[ uiDepth]+(MAX_CU_SIZE)*(subCUY) + subCUX;
	Int16* piInterCoefC[2]  = { pCache->psTempCoefU[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2, pCache->psTempCoefV[ uiDepth]+(MAX_CU_SIZE/2)*(subCUY/2) + subCUX/2 };

	xSubDct( piInterTmp0, pucInterPixY , pucInterPredY, MAX_CU_SIZE, piInterTmp0, piInterTmp1, nCUSize, nCUSize, MODE_INVALID );
	uiSumY = xQuant( piInterCoefY, piInterTmp0, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_P );

	if( uiSumY ) 
	{
		xDeQuant( piInterTmp0, piInterCoefY, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_P );

		for (UInt32 i=0; i<nCUSize;i++)
		{
			for (UInt32 j=0; j<nCUSize;j++)
			{
				wbitY += abs(piInterCoefY[i*MAX_CU_SIZE+j]);
			}	
		}
		xIDctAdd( pucInterRecY, piInterTmp0, pucInterPredY, MAX_CU_SIZE, piInterTmp1, piInterTmp0, nCUSize, nCUSize, MODE_INVALID  );
	}
	else 
	{
		for(UInt32 i=0; i<nCUSize; i++ ) 
		{
			memcpy( &pucInterRecY[i*MAX_CU_SIZE], &pucInterPredY[i*MAX_CU_SIZE], sizeof(Pxl)*nCUSize );
		}
	}
	//求失真
	distY = CalDistortion(pucInterPixY, pucInterRecY, MAX_CU_SIZE, nCUSize);

	if(nCUSize>=4)
	{
		for(int i=0; i<2; i++ ) 
		{
			xSubDct( piInterTmp0, pucInterPixC[i], pucInterPredC[i], MAX_CU_SIZE/2, piInterTmp0, piInterTmp1, nCUSize/2, nCUSize/2, MODE_INVALID );
			for (UInt32 i1=0; i1<nCUSize/2;i1++)
			{
				for (UInt32 j=0; j<nCUSize/2;j++)
				{
					wbitC += abs(piInterCoefC[i][i1*(MAX_CU_SIZE/2)+j]);
				}
			}
			uiSumC[i] = xQuant( piInterCoefC[i], piInterTmp0, MAX_CU_SIZE/2, nQPC, nCUSize/2, nCUSize/2, SLICE_P);
		}
		*pCbfY = (uiSumY    != 0);
		*pCbfU = (uiSumC[0] != 0);
		*pCbfV = (uiSumC[1] != 0);

		// Cr and Cb
		for(int i=0; i<2; i++ ) 
		{
			if( uiSumC[i] ) 
			{
				xDeQuant( piInterTmp0, piInterCoefC[i], (MAX_CU_SIZE/2), nQPC, nCUSize/2, nCUSize/2, SLICE_P );
				xIDctAdd( pucInterRecC[i], piInterTmp0, pucInterPredC[i], MAX_CU_SIZE/2 ,piInterTmp1, piInterTmp0, nCUSize/2, nCUSize/2, MODE_INVALID);
			}
			else 
			{
				UInt k;
				for( k=0; k<nCUSize/2; k++ ) 
				{
					memcpy( &pucInterRecC[i][k*(MAX_CU_SIZE/2)], &pucInterPredC[i][k*(MAX_CU_SIZE/2)], sizeof(Pxl)*nCUSize/2 );
				}
			}
		}
		//求色度的失真
		distC = CalDistortion(pucInterPixC[0], pucInterRecC[0], MAX_CU_SIZE/2, nCUSize/2);
		distC += CalDistortion(pucInterPixC[1], pucInterRecC[1], MAX_CU_SIZE/2, nCUSize/2);
		pCache->TotalDistortion = distY+distC;
	}

	Int32 copyCURelIdx = 0;
	Int32 copyFromCUOffset = 0;
	Int32 copyToCUOffset = 0;
	//将PU内的全部StartChange变量均赋值为同样的值
	for (UInt i = 0; i < nCUSize/4;i++)
	{
		for (UInt j = 0; j < nCUSize/4;j++)
		{
			copyCURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx] + i * (h->ucMaxCUWidth/4) + j];
			copyFromCUOffset = uiDepth*pCache->NumPartition + PURelIdx;
			copyToCUOffset = uiDepth*pCache->NumPartition + copyCURelIdx;

			pCache->TempbCbfY[copyToCUOffset] = pCache->TempbCbfY[copyFromCUOffset];
			pCache->TempbCbfU[copyToCUOffset] = pCache->TempbCbfU[copyFromCUOffset];
			pCache->TempbCbfV[copyToCUOffset] = pCache->TempbCbfV[copyFromCUOffset];
		}
	}
}

void PreLargeInter(MTC265_t *h)
{
	MVpara LeftPix;//左边角点的相对位置
	MVpara TopPix;//上边角点的相对位置
	MVpara RightPix;//右边角点的相对位置
	MVpara BottomPix;//下边角点的相对位置

	LeftPix.m_x = 0;//左边角点的相对位置
	LeftPix.m_y = h->usAddRefLength;//左边角点的相对位置
	TopPix.m_x = 0;//上边角点的相对位置
	TopPix.m_y = 0;//上边角点的相对位置
	RightPix.m_x = h->usAddRefLength + (h->usWidth + h->PAD_YSIZEw) ;//右边角点的相对位置
	RightPix.m_y = h->usAddRefLength;//右边角点的相对位置
	BottomPix.m_x = 0;//下边角点的相对位置
	BottomPix.m_y = h->usAddRefLength +(h->usHeight + h->PAD_YSIZEh) ;//下边角点的相对位置

	Pxl* StartLargeInterY = h->LargeInter.pucY;
	Pxl* StartRefnY = h->refn[0].pucY;
	Pxl* StartLargeInterU = h->LargeInter.pucU;
	Pxl* StartRefnU = h->refn[0].pucU;
	Pxl* StartLargeInterV = h->LargeInter.pucV;
	Pxl* StartRefnV = h->refn[0].pucV;

	//Y分量的
	for (int x = 0; x<(h->usWidth + h->PAD_YSIZEw); x++)
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterY[(y+h->usAddRefLength)*h->usLargeWidth+(x+h->usAddRefLength)] = StartRefnY [y*(h->usWidth + h->PAD_YSIZEw)+x];

	for (int x = 0; x<h->usAddRefLength; x++)//A
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterY[(y+LeftPix.m_y)*h->usLargeWidth+(x+LeftPix.m_x)] = StartRefnY [y*(h->usWidth + h->PAD_YSIZEw)];

	for (int x = 0; x<h->usAddRefLength; x++)//C
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterY[(y+RightPix.m_y)*h->usLargeWidth+(x+RightPix.m_x)] = StartRefnY [y*(h->usWidth + h->PAD_YSIZEw)+(h->usWidth + h->PAD_YSIZEw)-1];

	for (int x = 0; x< (h->usWidth + h->PAD_YSIZEw) + 2*h->usAddRefLength; x++)//B
		for (int y =0; y< h->usAddRefLength; y++)
			StartLargeInterY[(y+TopPix.m_y)*h->usLargeWidth+(x+TopPix.m_x)] = StartLargeInterY [LeftPix.m_y*h->usLargeWidth + x];

	for (int x = 0; x< (h->usWidth + h->PAD_YSIZEw) + 2*h->usAddRefLength; x++)//D
		for (int y =0; y< h->usAddRefLength; y++)
			StartLargeInterY[(y+BottomPix.m_y)*h->usLargeWidth+(x+BottomPix.m_x)] = StartLargeInterY [(BottomPix.m_y-1)*h->usLargeWidth+x];

	//U分量的
	for (int x = 0; x<(h->usWidth + h->PAD_YSIZEw); x++)
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterU[(y/2+h->usAddRefLength/2)*h->usLargeWidth/2+(x/2+h->usAddRefLength/2)] = StartRefnU [y/2*(h->usWidth + h->PAD_YSIZEw)/2+x/2];

	for (int x = 0; x<h->usAddRefLength; x++)//A
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterU[(y/2+LeftPix.m_y/2)*h->usLargeWidth/2+(x/2+LeftPix.m_x/2)] = StartRefnU [(y/2)*((h->usWidth + h->PAD_YSIZEw)/2)];

	for (int x = 0; x<h->usAddRefLength; x++)//C
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterU[(y/2+RightPix.m_y/2)*h->usLargeWidth/2+(x/2+RightPix.m_x/2)] = StartRefnU [(y/2)*((h->usWidth + h->PAD_YSIZEw)/2)+((h->usWidth + h->PAD_YSIZEw)-1)/2];

	for (int x = 0; x< (h->usWidth + h->PAD_YSIZEw) + 2*h->usAddRefLength; x++)//B
		for (int y =0; y< h->usAddRefLength; y++)
			StartLargeInterU[(y/2+TopPix.m_y/2)*h->usLargeWidth/2+(x/2+TopPix.m_x/2)] = StartLargeInterU [(LeftPix.m_y/2)*(h->usLargeWidth/2) + (x/2)];

	for (int x = 0; x< (h->usWidth + h->PAD_YSIZEw) + 2*h->usAddRefLength; x++)//D
		for (int y =0; y< h->usAddRefLength; y++)
			StartLargeInterU[(y/2+BottomPix.m_y/2)*h->usLargeWidth/2+(x/2+BottomPix.m_x/2)] = StartLargeInterU [(BottomPix.m_y-1)/2*h->usLargeWidth/2+x/2];

	//V分量的
	for (int x = 0; x<(h->usWidth + h->PAD_YSIZEw); x++)
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterV[(y/2+h->usAddRefLength/2)*h->usLargeWidth/2+(x/2+h->usAddRefLength/2)] = StartRefnV[y/2*(h->usWidth + h->PAD_YSIZEw)/2+x/2];

	for (int x = 0; x<h->usAddRefLength; x++)//A
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterV[(y/2+LeftPix.m_y/2)*h->usLargeWidth/2+(x/2+LeftPix.m_x/2)] = StartRefnV [y/2*(h->usWidth + h->PAD_YSIZEw)/2];

	for (int x = 0; x<h->usAddRefLength; x++)//C
		for (int y =0; y< (h->usHeight + h->PAD_YSIZEh); y++)
			StartLargeInterV[(y/2+RightPix.m_y/2)*h->usLargeWidth/2+(x/2+RightPix.m_x/2)] = StartRefnV[y/2*(h->usWidth + h->PAD_YSIZEw)/2+((h->usWidth + h->PAD_YSIZEw)-1)/2];

	for (int x = 0; x< (h->usWidth + h->PAD_YSIZEw) + 2*h->usAddRefLength; x++)//B
		for (int y =0; y< h->usAddRefLength; y++)
			StartLargeInterV[(y/2+TopPix.m_y/2)*h->usLargeWidth/2+(x/2+TopPix.m_x/2)] = StartLargeInterV[LeftPix.m_y/2*h->usLargeWidth/2 + x/2];

	for (int x = 0; x< (h->usWidth + h->PAD_YSIZEw) + 2*h->usAddRefLength; x++)//D
		for (int y =0; y< h->usAddRefLength; y++)
			StartLargeInterV[(y/2+BottomPix.m_y/2)*h->usLargeWidth/2+(x/2+BottomPix.m_x/2)] = StartLargeInterV[(BottomPix.m_y-1)/2*h->usLargeWidth/2+x/2];
}

void InterPrediction(MTC265_t *h, Interpara * StartInter, MTC265_PartSize ePartSize, UInt8 uiDepth, UInt32 subPUX, UInt32 subPUY, double& PUBestSAD)
{
	//初始化
	MTC265_Cache* pCache = &h->cache;
	pCache->subPUX = subPUX;
	pCache->subPUY = subPUY;
	Int32 iSearchRounds = 3;
	Int32 iDist = 2;
	Int32 SrchWindowRange = 64; 
	UInt32 nCUSize = h->ucMaxCUWidth >> uiDepth;
	UInt32 subPUWidth = 0; 
	UInt32 subPUHeight = 0;
	UInt32 uiLCUIdx = h->uiCUX/h->ucMaxCUWidth + (((h->usWidth+h->PAD_YSIZEw)/h->ucMaxCUWidth) * (h->uiCUY/h->ucMaxCUWidth));
	UInt32 CURelIdx = pCache->RelativeIdx[uiDepth];
	UInt32 PUPartIdx = pCache->PUPartIdx[uiDepth];//会发生变化，仅代表当前PU的情况
	UInt32 PURelIdx = pCache->PURelativeIdx[uiDepth];//会发生变化，仅代表当前PU的情况
	Interpara *StartChange = 0;
	Interpara *StartTempInter = h->TempInter;
	Int32 copyPURelIdx = 0;
	//将本PU内的全部位置，均赋同样的值
	Int32 TempInterToCur = 0;
	MVpara tempMvp_0 ={0,0};
	MVpara tempMvp_1 ={0,0};
	MVpara tempMvd = {0,0};
	MVpara PreCenter ={0,0};
	Int32 BinNumber_X = 0;
	Int32 BinNumber_Y = 0;

	UInt SadMvpY0 = MAX_UINT;
	UInt SadMvpY1 = MAX_UINT;
	UInt tempMVPidx = 0;

	UInt ruiPartIdxLT = 0;
	UInt ruiPartIdxRT = 0;
	UInt ruiPartIdxLB = 0;
	UInt ruiPartIdxRB = 0;
	UInt ruiPartIdxCenter = 0;
	UInt ruiLeftLCUIdx = 0;//左LCUindex
	UInt ruiAboveLeftLCUIdx = 0;//左上LCUindex

	UInt ruiAboveLCUIdx = 0;//上LCUindex
	UInt ruiAboveRightLCUIdx = 0;//右上LCUindex
	UInt ruiBelowLeftLCUIdx = 0;//左下LCUindex

	UInt ruiLeftPURelIdx = 0;//左PUindex
	UInt ruiAboveLeftPURelIdx = 0;//左上PUindex
	UInt ruiAbovePURelIdx = 0; //上PUindex
	UInt ruiAboveRightPURelIdx = 0;//右上PUindex
	UInt ruiBelowLeftPURelIdx = 0;//左下PUindex
	UInt ruiBelowRightLCUIdx = 0;
	UInt ruiBelowRightPURelIdx = 0;

	//AMVP
	deriveLeftTopIdxGeneral ( h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PUPartIdx, ruiPartIdxLT );//最后1个值为返回值, 返回当前PU内部左下角
	bool AboveLeftValid = getPUAboveLeft	( h, uiLCUIdx, ruiPartIdxLT, ruiAboveLeftLCUIdx, ruiAboveLeftPURelIdx );//最后2个值为返回值, 返回左上PU的LCUIdx和RelativeIdx
	deriveRightTopIdxGeneral( h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PUPartIdx, ruiPartIdxRT );//最后1个值为返回值
	bool AboveValid = getPUAbove( h, uiLCUIdx, ruiPartIdxRT, ruiAboveLCUIdx, ruiAbovePURelIdx );//最后2个值为返回值, 返回上PU的LCUIdx和RelativeIdx
	bool AboveRightValid = getPUAboveRight( h, uiLCUIdx, ruiPartIdxRT, ruiAboveRightLCUIdx, ruiAboveRightPURelIdx );//最后2个值为返回值, 返回右上PU的LCUIdx和RelativeIdx
	deriveLeftBottomIdxGeneral ( h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PUPartIdx, ruiPartIdxLB );//最后1个值为返回值
	bool LeftValid = getPULeft ( h, uiLCUIdx, ruiPartIdxLB, ruiLeftLCUIdx, ruiLeftPURelIdx ); //最后2个值为返回值, 返回左PU的LCUIdx和RelativeIdx
	bool BelowLeftValid = getPUBelowLeft	( h, uiLCUIdx, ruiPartIdxLB, ruiBelowLeftLCUIdx, ruiBelowLeftPURelIdx );//最后2个值为返回值, 返回左下PU的LCUIdx和RelativeIdx
	deriveRightBottomIdxGeneral ( h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PUPartIdx, ruiPartIdxRB );//最后1个值为返回值
	bool BelowRightValid = getPUBelowRight ( h, uiLCUIdx, ruiPartIdxRB, ruiBelowRightLCUIdx, ruiBelowRightPURelIdx );//最后2个值为返回值, 返回右下PU的LCUIdx和RelativeIdx
	deriveCenterIdxGeneral( h, PURelIdx, uiDepth, ePartSize, ruiPartIdxCenter);//最后1个值为返回值

	if ( BelowLeftValid )
	{
		FindMv( h, MODE_AMVP, 0, uiDepth, uiLCUIdx, ruiBelowLeftLCUIdx, ruiBelowLeftPURelIdx, 0, ePartSize, VALID_LB, PUPartIdx, tempMvp_0, tempMvd );
		StartInter->Mvp_0.m_x = tempMvp_0.m_x;
		StartInter->Mvp_0.m_y = tempMvp_0.m_y;
		tempMVPidx ++;
	}
	else if ( LeftValid )
	{
		FindMv( h, MODE_AMVP, 0, uiDepth, uiLCUIdx, ruiLeftLCUIdx, ruiLeftPURelIdx, 0, ePartSize, VALID_L, PUPartIdx, tempMvp_0, tempMvd );
		StartInter->Mvp_0.m_x = tempMvp_0.m_x;
		StartInter->Mvp_0.m_y = tempMvp_0.m_y;		
		tempMVPidx ++;
	}
	if (AboveRightValid )
	{
		FindMv( h, MODE_AMVP, 0, uiDepth, uiLCUIdx, ruiAboveRightLCUIdx, ruiAboveRightPURelIdx, 0, ePartSize, VALID_RT, PUPartIdx, tempMvp_0, tempMvd );
		if(tempMVPidx == 0)
		{
			StartInter->Mvp_0.m_x = tempMvp_0.m_x;
			StartInter->Mvp_0.m_y = tempMvp_0.m_y;	
		}
		else//只会添加1个Mvp，因此此时tempMVPidx == 1
		{
			StartInter->Mvp_1.m_x = tempMvp_0.m_x;
			StartInter->Mvp_1.m_y = tempMvp_0.m_y;	
		}
		tempMVPidx ++;
	}
	else if (AboveValid )
	{
		FindMv( h, MODE_AMVP, 0, uiDepth, uiLCUIdx, ruiAboveLCUIdx, ruiAbovePURelIdx, 0, ePartSize, VALID_T, PUPartIdx, tempMvp_0, tempMvd );
		if(tempMVPidx == 0)
		{
			StartInter->Mvp_0.m_x = tempMvp_0.m_x;
			StartInter->Mvp_0.m_y = tempMvp_0.m_y;	
		}
		else//只会添加1个Mvp，因此此时tempMVPidx == 1
		{
			StartInter->Mvp_1.m_x = tempMvp_0.m_x;
			StartInter->Mvp_1.m_y = tempMvp_0.m_y;	
		}
		tempMVPidx ++;
	}
	else if (AboveLeftValid )
	{
		FindMv( h, MODE_AMVP, 0, uiDepth, uiLCUIdx, ruiAboveLeftLCUIdx, ruiAboveLeftPURelIdx, 0, ePartSize, VALID_LT, PUPartIdx, tempMvp_0, tempMvd );
		if(tempMVPidx == 0)
		{
			StartInter->Mvp_0.m_x = tempMvp_0.m_x;
			StartInter->Mvp_0.m_y = tempMvp_0.m_y;	
		}
		else//只会添加1个Mvp，因此此时tempMVPidx == 1
		{
			StartInter->Mvp_1.m_x = tempMvp_0.m_x;
			StartInter->Mvp_1.m_y = tempMvp_0.m_y;	
		}
		tempMVPidx ++;
	}

	if (tempMVPidx <2)//B位置取不到
	{
		if (BelowRightValid)
		{
			FindMv( h, MODE_AMVP,-1, uiDepth, uiLCUIdx, ruiBelowRightLCUIdx, ruiBelowRightPURelIdx, 0, ePartSize, VALID_T, 0, tempMvp_0, tempMvd );//TMVP
		}
		else
		{
			FindMv( h, MODE_AMVP, -1, uiDepth, uiLCUIdx, uiLCUIdx, ruiPartIdxCenter, 0, ePartSize, VALID_T, PUPartIdx, tempMvp_0, tempMvd );
		}
		//else
		if(tempMVPidx == 0)
		{
			StartInter->Mvp_0.m_x = tempMvp_0.m_x;
			StartInter->Mvp_0.m_y = tempMvp_0.m_y;	
		}
		else//只会添加1个Mvp，因此此时tempMVPidx == 1
		{
			StartInter->Mvp_1.m_x = tempMvp_0.m_x;
			StartInter->Mvp_1.m_y = tempMvp_0.m_y;	
		}
		tempMVPidx ++;
	}

	MvpComparison ( h, StartInter, ePartSize, nCUSize, subPUX, subPUY);

	//Motion Estimation
	PreCenter.m_x = StartInter->iSrchCenBest.m_x;
	PreCenter.m_y = StartInter->iSrchCenBest.m_y;
	StartInter->iSrchCenBest.m_x = Clip3(0, h->usLargeWidth-h->ucMaxCUWidth, StartInter->iSrchCenBest.m_x);
	StartInter->iSrchCenBest.m_y = Clip3(0, h->usLargeHeight-h->ucMaxCUWidth, StartInter->iSrchCenBest.m_y);

	for ( Int32 i = 0; i < iSearchRounds; i++ )//iSearchRounds 为搜索递归次数，目前为3
	{
		InterSearch( h, StartInter, uiDepth, ePartSize, 2* (i+1), &SrchWindowRange );
	}
	InterSearch( h, StartInter, uiDepth, ePartSize, 8, &SrchWindowRange );
	InterSearch( h, StartInter, uiDepth, ePartSize, 8, &SrchWindowRange );

	xTZ2PointSearch( h, StartInter, uiDepth, ePartSize);

	StartInter->iSrchCenBest.m_x = Clip3(0, h->usLargeWidth-h->ucMaxCUWidth, StartInter->iSrchCenBest.m_x);
	StartInter->iSrchCenBest.m_y = Clip3(0, h->usLargeHeight-h->ucMaxCUWidth, StartInter->iSrchCenBest.m_y);

	//计算MVd
	StartInter->iMvdBestY.m_x = StartInter->iSrchCenBest.m_x-PreCenter.m_x; //三角形矩阵运算
	StartInter->iMvdBestY.m_y = StartInter->iSrchCenBest.m_y-PreCenter.m_y;

	xCheckBestMVP( h, StartInter, tempMVPidx);

	//计算MV
	StartInter->iMvBestY.m_x = (StartInter->iMvpBestY.m_x>>2) + StartInter->iMvdBestY.m_x;
	StartInter->iMvBestY.m_y = (StartInter->iMvpBestY.m_y>>2) + StartInter->iMvdBestY.m_y;
	MVpara pcMvInt;
	MVpara rcMvHalf;
	MVpara rcMvQter;
	pcMvInt.m_x = StartInter->iMvBestY.m_x;
	pcMvInt.m_y = StartInter->iMvBestY.m_y;

	xPatternSearchFracDIF(h, ePartSize, uiDepth, StartInter->iSrchCenBest.m_x, StartInter->iSrchCenBest.m_y, &pcMvInt, rcMvHalf, rcMvQter, PUBestSAD);
	pcMvInt.m_x <<= 2;
	pcMvInt.m_y <<= 2;
	pcMvInt.m_x += (rcMvHalf.m_x <<= 1);
	pcMvInt.m_y += (rcMvHalf.m_y <<= 1);
	pcMvInt.m_x += rcMvQter.m_x;
	pcMvInt.m_y += rcMvQter.m_y;

	calcMvdBinNumber(pcMvInt.m_x, BinNumber_X);
	calcMvdBinNumber(pcMvInt.m_y, BinNumber_Y);	
	PUBestSAD -= h->sqrt_Inter_lamada *(BinNumber_X + BinNumber_Y);

	StartInter->iMvBestY.m_x = pcMvInt.m_x;
	StartInter->iMvBestY.m_y = pcMvInt.m_y;
	StartInter->iMvdBestY.m_x = StartInter->iMvBestY.m_x - StartInter->iMvpBestY.m_x;
	StartInter->iMvdBestY.m_y = StartInter->iMvBestY.m_y - StartInter->iMvpBestY.m_y;
	
	StartInter->iSrchCenBest.m_x = subPUX + h->uiCUX + h->usAddRefLength + (pcMvInt.m_x>>2);
	StartInter->iSrchCenBest.m_y = subPUY + h->uiCUY + h->usAddRefLength + (pcMvInt.m_y>>2);

	//GetPUSize
	if(ePartSize == SIZE_2Nx2N)
	{
		subPUWidth = h->ucMaxCUWidth >> uiDepth; 
		subPUHeight = h->ucMaxCUWidth >> uiDepth; 
	}
	else if(ePartSize == SIZE_2NxN)
	{
		subPUWidth = h->ucMaxCUWidth >> uiDepth; 
		subPUHeight = h->ucMaxCUWidth >> (uiDepth+1); 
	}
	else if(ePartSize == SIZE_Nx2N)
	{
		subPUWidth = h->ucMaxCUWidth >> (uiDepth+1); 
		subPUHeight = h->ucMaxCUWidth >> uiDepth; 
	}

	//将PU内的全部StartChange变量均赋值为同样的值
	for (UInt i = 0; i < subPUHeight/4;i++)
	{
		for (UInt j = 0; j < subPUWidth/4;j++)
		{
			copyPURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx] + i * (h->ucMaxCUWidth/4) + j];
			TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*pCache->NumPartition*h->PUnumber + uiDepth*pCache->NumPartition*h->PUnumber + copyPURelIdx*h->PUnumber + (Int32)ePartSize;
			//以下的变量由四元组(iLCUNum, uiDepth, copyPUrelIdx, ePartSize)决定
			StartChange = StartTempInter + TempInterToCur;

			StartChange->iMvBestY.m_x = StartInter->iMvBestY.m_x;
			StartChange->iMvBestY.m_y = StartInter->iMvBestY.m_y;
			StartChange->iMvdBestY.m_x = StartInter->iMvdBestY.m_x;
			StartChange->iMvdBestY.m_y = StartInter->iMvdBestY.m_y;
			StartChange->iMvpBestY.m_x = StartInter->iMvpBestY.m_x;
			StartChange->iMvpBestY.m_y = StartInter->iMvpBestY.m_y;
			StartChange->iMvp_index = StartInter->iMvp_index;
			StartChange->iSadBest = StartInter->iSadBest;
			StartChange->iSrchCenBest.m_x = StartInter->iSrchCenBest.m_x;
			StartChange->iSrchCenBest.m_y = StartInter->iSrchCenBest.m_y;
			StartChange->Mvp_0.m_x = StartInter->Mvp_0.m_x;
			StartChange->Mvp_0.m_y = StartInter->Mvp_0.m_y;
			StartChange->Mvp_1.m_x = StartInter->Mvp_1.m_x;
			StartChange->Mvp_1.m_y = StartInter->Mvp_1.m_y;	
		}
	}
}

void xTZ2PointSearch(MTC265_t *h, Interpara* StartInter, UInt8 uiDepth, MTC265_PartSize ePartSize)
{
	Int   iSrchRngHorLeft   = 0;
	Int   iSrchRngHorRight  = h->usLargeWidth;
	Int   iSrchRngVerTop    = 0;
	Int   iSrchRngVerBottom = h->usLargeHeight;

	// 2 point search,                   //   1 2 3
	// check only the 2 untested points  //   4 0 5
	// around the start point            //   6 7 8
	Int iStartX = StartInter->iSrchCenBest.m_x;
	Int iStartY = StartInter->iSrchCenBest.m_y;
	Int8 ResultTmp = 0;
	switch( StartInter->ucPointNr )
	{
	case 1:
		{
			if ( (iStartX - 1) >= iSrchRngHorLeft )
			{
				ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX - 1, iStartY, 1);
				if (ResultTmp)
				{
					StartInter->iSrchCenBest.m_x = iStartX - 1;
					StartInter->iSrchCenBest.m_y = iStartY;
				}
			}
			if ( (iStartY - 1) >= iSrchRngVerTop )
			{
				ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX, iStartY - 1, 1);
				if (ResultTmp)
				{
					StartInter->iSrchCenBest.m_x = iStartX;
					StartInter->iSrchCenBest.m_y = iStartY - 1;
				}
			}
		}
		break;
	case 2:
		{
			if ( (iStartY - 1) >= iSrchRngVerTop )
			{
				if ( (iStartX - 1) >= iSrchRngHorLeft )
				{
					ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX - 1, iStartY - 1, 1);
					if (ResultTmp)
					{
						StartInter->iSrchCenBest.m_x = iStartX - 1;
						StartInter->iSrchCenBest.m_y = iStartY - 1;
					}
				}
				if ( (iStartX + 1) <= iSrchRngHorRight )
				{
					ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX + 1, iStartY - 1, 1);
					if (ResultTmp)
					{
						StartInter->iSrchCenBest.m_x = iStartX + 1;
						StartInter->iSrchCenBest.m_y = iStartY - 1;
					}
				}
			}
		}
		break;
	case 3:
		{
			if ( (iStartY - 1) >= iSrchRngVerTop )
			{
				ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX, iStartY - 1, 1);
				if (ResultTmp)
				{
					StartInter->iSrchCenBest.m_x = iStartX;
					StartInter->iSrchCenBest.m_y = iStartY - 1;
				}
			}
			if ( (iStartX + 1) <= iSrchRngHorRight )
			{
				ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX + 1, iStartY, 1);
				if (ResultTmp)
				{
					StartInter->iSrchCenBest.m_x = iStartX + 1;
					StartInter->iSrchCenBest.m_y = iStartY;
				}
			}
		}
		break;
	case 4:
		{
			if ( (iStartX - 1) >= iSrchRngHorLeft )
			{
				if ( (iStartY + 1) <= iSrchRngVerBottom )
				{
					ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX - 1, iStartY + 1, 1);
					if (ResultTmp)
					{
						StartInter->iSrchCenBest.m_x = iStartX - 1;
						StartInter->iSrchCenBest.m_y = iStartY + 1;
					}
				}
				if ( (iStartY - 1) >= iSrchRngVerTop )
				{
					ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX - 1, iStartY - 1, 1);
					if (ResultTmp)
					{
						StartInter->iSrchCenBest.m_x = iStartX - 1;
						StartInter->iSrchCenBest.m_y = iStartY - 1;
					}
				}
			}
		}
		break;
	case 5:
		{
			if ( (iStartX + 1) <= iSrchRngHorRight )
			{
				if ( (iStartY - 1) >= iSrchRngVerTop )
				{
					ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX + 1, iStartY - 1, 1);
					if (ResultTmp)
					{
						StartInter->iSrchCenBest.m_x = iStartX + 1;
						StartInter->iSrchCenBest.m_y = iStartY - 1;
					}
				}
				if ( (iStartY + 1) <= iSrchRngVerBottom )
				{
					ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX + 1, iStartY + 1, 1);
					if (ResultTmp)
					{
						StartInter->iSrchCenBest.m_x = iStartX + 1;
						StartInter->iSrchCenBest.m_y = iStartY + 1;
					}
				}
			}
		}
		break;
	case 6:
		{
			if ( (iStartX - 1) >= iSrchRngHorLeft )
			{
				ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX - 1, iStartY, 1);
				if (ResultTmp)
				{
					StartInter->iSrchCenBest.m_x = iStartX - 1;
					StartInter->iSrchCenBest.m_y = iStartY;
				}
			}
			if ( (iStartY + 1) <= iSrchRngVerBottom )
			{
				ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX, iStartY + 1, 1);
				if (ResultTmp)
				{
					StartInter->iSrchCenBest.m_x = iStartX;
					StartInter->iSrchCenBest.m_y = iStartY + 1;
				}
			}
		}
		break;
	case 7:
		{
			if ( (iStartY + 1) <= iSrchRngVerBottom )
			{
				if ( (iStartX - 1) >= iSrchRngHorLeft )
				{
					ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX - 1, iStartY + 1, 1);
					if (ResultTmp)
					{
						StartInter->iSrchCenBest.m_x = iStartX - 1;
						StartInter->iSrchCenBest.m_y = iStartY + 1;
					}
				}
				if ( (iStartX + 1) <= iSrchRngHorRight )
				{
					ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX + 1, iStartY + 1, 1);
					if (ResultTmp)
					{
						StartInter->iSrchCenBest.m_x = iStartX + 1;
						StartInter->iSrchCenBest.m_y = iStartY + 1;
					}
				}
			}
		}
		break;
	case 8:
		{
			if ( (iStartX + 1) <= iSrchRngHorRight )
			{
				ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX + 1, iStartY, 1);
				if (ResultTmp)
				{
					StartInter->iSrchCenBest.m_x = iStartX + 1;
					StartInter->iSrchCenBest.m_y = iStartY ;
				}
			}
			if ( (iStartY + 1) <= iSrchRngVerBottom )
			{
				ResultTmp = xDiamondSearch ( h, StartInter, uiDepth, ePartSize, iStartX, iStartY + 1, 1);
				if (ResultTmp)
				{
					StartInter->iSrchCenBest.m_x = iStartX;
					StartInter->iSrchCenBest.m_y = iStartY + 1;
				}
			}
		}
		break;
	default:
		break;
	} // switch( rcStruct.ucPointNr )
}

void xPatternSearchFracDIF(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth, Int32 iCopyStart_x, Int32 iCopyStart_y, MVpara* pcMvInt, MVpara& rcMvHalf, MVpara& rcMvQter, double& ruiCost)
{
	xExtDIFUpSamplingH(h,ePartSize, uiDepth, iCopyStart_x, iCopyStart_y);
	rcMvHalf = *pcMvInt;   
	rcMvHalf.m_x <<= 1; 
	rcMvHalf.m_y <<= 1;   // for mv-cost

	MVpara baseRefMv;
	baseRefMv.m_x = 0;
	baseRefMv.m_y = 0;

	ruiCost = xPatternRefinement( h, uiDepth, ePartSize, baseRefMv, 2, rcMvHalf );

	xExtDIFUpSamplingQ( h, ePartSize, uiDepth, iCopyStart_x, iCopyStart_y, rcMvHalf);
	baseRefMv.m_x = rcMvHalf.m_x;
	baseRefMv.m_y = rcMvHalf.m_y;
	baseRefMv.m_x <<= 1;
	baseRefMv.m_y <<= 1;

	rcMvQter = *pcMvInt;   
	rcMvQter.m_x <<= 1;    // for mv-cost
	rcMvQter.m_y <<= 1;
	rcMvQter.m_x += rcMvHalf.m_x;  
	rcMvQter.m_y += rcMvHalf.m_y;  
	rcMvQter.m_x <<= 1;
	rcMvQter.m_y <<= 1;
	ruiCost = xPatternRefinement( h, uiDepth, ePartSize, baseRefMv, 1, rcMvQter );
}

void xExtDIFUpSamplingH(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth, Int32 iCopyStart_x, Int32 iCopyStart_y)
{
	MTC265_Cache* pCache = &h->cache;
	UInt32 subPUX = pCache->subPUX;
	UInt32 subPUY = pCache->subPUY;
	xEncGetPUPosition( h, ePartSize, uiDepth, 0, &subPUX, &subPUY );//得到subPUX和subPUY，确定当前PU左上角点的位置 
	Int width      = pCache->subPUWidth;
	Int height     = pCache->subPUHeight;
	Int srcStride  = h->usLargeWidth;

	Int intStride = MAX_CU_SIZE + 16;
	Int dstStride = MAX_CU_SIZE + 16;
	Short *intPtr;
	Short *dstPtr;
	Int filterSize = NTAPS_LUMA;
	Int halfFilterSize = (filterSize>>1);
	Pel *srcPtr = h->LargeInter.pucY + iCopyStart_y * h->usLargeWidth + iCopyStart_x - halfFilterSize*srcStride - 1;

	filterHorLuma(srcPtr, srcStride, pCache->filteredBlockTmp[0], intStride, width+1, height+filterSize, 0, false);
	filterHorLuma(srcPtr, srcStride, pCache->filteredBlockTmp[2], intStride, width+1, height+filterSize, 2, false);

	intPtr = pCache->filteredBlockTmp[0] + halfFilterSize * intStride + 1;  
	dstPtr = pCache->filteredBlock[0][0];
	filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+0, height+0, 0, false, true);

	intPtr = pCache->filteredBlockTmp[0] + (halfFilterSize-1) * intStride + 1;  
	dstPtr = pCache->filteredBlock[2][0];
	filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+0, height+1, 2, false, true);

	intPtr = pCache->filteredBlockTmp[2] + halfFilterSize * intStride;
	dstPtr = pCache->filteredBlock[0][2];
	filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+1, height+0, 0, false, true);

	intPtr = pCache->filteredBlockTmp[2] + (halfFilterSize-1) * intStride;
	dstPtr = pCache->filteredBlock[2][2];
	filterVerLuma(intPtr, intStride, dstPtr, dstStride, width+1, height+1, 2, false, true);
}

void xExtDIFUpSamplingQ(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth, Int32 iCopyStart_x, Int32 iCopyStart_y, MVpara halfPelRef )
{
	MTC265_Cache* pCache = &h->cache;
	UInt32 subPUX = pCache->subPUX;
	UInt32 subPUY = pCache->subPUY;
	xEncGetPUPosition( h, ePartSize, uiDepth, 0, &subPUX, &subPUY );//得到subPUX和subPUY，确定当前PU左上角点的位置 
	Int width      = pCache->subPUWidth;
	Int height     = pCache->subPUHeight;
	Int srcStride  = h->usLargeWidth;

	Pel *srcPtr;
	Int intStride = MAX_CU_SIZE + 16;
	Int dstStride = MAX_CU_SIZE + 16;
	Short *intPtr;
	Short *dstPtr;
	Int filterSize = NTAPS_LUMA;

	Int halfFilterSize = (filterSize>>1);

	Int extHeight = (halfPelRef.m_y == 0) ? height + filterSize : height + filterSize-1;

	// Horizontal filter 1/4
	srcPtr = h->LargeInter.pucY + iCopyStart_y * h->usLargeWidth + iCopyStart_x - halfFilterSize * srcStride - 1;
	intPtr = pCache->filteredBlockTmp[1];
	if (halfPelRef.m_y > 0)
	{
		srcPtr += srcStride;
	}
	if (halfPelRef.m_x >= 0)
	{
		srcPtr += 1;
	}
	filterHorLuma(srcPtr, srcStride, intPtr, intStride, width, extHeight, 1, false);

	// Horizontal filter 3/4
	srcPtr = h->LargeInter.pucY + iCopyStart_y * h->usLargeWidth + iCopyStart_x - halfFilterSize*srcStride - 1;
	intPtr = pCache->filteredBlockTmp[3];
	if (halfPelRef.m_y > 0)
	{
		srcPtr += srcStride;
	}
	if (halfPelRef.m_x > 0)
	{
		srcPtr += 1;
	}
	filterHorLuma(srcPtr, srcStride, intPtr, intStride, width, extHeight, 3, false);        

	// Generate @ 1,1
	intPtr = pCache->filteredBlockTmp[1] + (halfFilterSize-1) * intStride;
	dstPtr = pCache->filteredBlock[1][1];
	if (halfPelRef.m_y == 0)
	{
		intPtr += intStride;
	}
	filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true);

	// Generate @ 3,1
	intPtr = pCache->filteredBlockTmp[1] + (halfFilterSize-1) * intStride;
	dstPtr = pCache->filteredBlock[3][1];
	filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true);

	if (halfPelRef.m_y != 0)
	{
		// Generate @ 2,1
		intPtr = pCache->filteredBlockTmp[1] + (halfFilterSize-1) * intStride;
		dstPtr = pCache->filteredBlock[2][1];
		if (halfPelRef.m_y == 0)
		{
			intPtr += intStride;
		}
		filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true);

		// Generate @ 2,3
		intPtr = pCache->filteredBlockTmp[3] + (halfFilterSize-1) * intStride;
		dstPtr = pCache->filteredBlock[2][3];
		if (halfPelRef.m_y == 0)
		{
			intPtr += intStride;
		}
		filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true);
	}
	else
	{
		// Generate @ 0,1
		intPtr = pCache->filteredBlockTmp[1] + halfFilterSize * intStride;
		dstPtr = pCache->filteredBlock[0][1];
		filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true);

		// Generate @ 0,3
		intPtr = pCache->filteredBlockTmp[3] + halfFilterSize * intStride;
		dstPtr = pCache->filteredBlock[0][3];
		filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true);
	}

	if (halfPelRef.m_x != 0)
	{
		// Generate @ 1,2
		intPtr = pCache->filteredBlockTmp[2] + (halfFilterSize-1) * intStride;
		dstPtr = pCache->filteredBlock[1][2];
		if (halfPelRef.m_x > 0)
		{
			intPtr += 1;
		}
		if (halfPelRef.m_y >= 0)
		{
			intPtr += intStride;
		}
		filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true);

		// Generate @ 3,2
		intPtr = pCache->filteredBlockTmp[2] + (halfFilterSize-1) * intStride;
		dstPtr = pCache->filteredBlock[3][2];
		if (halfPelRef.m_x > 0)
		{
			intPtr += 1;
		}
		if (halfPelRef.m_y > 0)
		{
			intPtr += intStride;
		}
		filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true);  
	}
	else
	{
		// Generate @ 1,0
		intPtr = pCache->filteredBlockTmp[0] + (halfFilterSize-1) * intStride + 1;
		dstPtr = pCache->filteredBlock[1][0];
		if (halfPelRef.m_y >= 0)
		{
			intPtr += intStride;
		}
		filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true);

		// Generate @ 3,0
		intPtr = pCache->filteredBlockTmp[0] + (halfFilterSize-1) * intStride + 1;
		dstPtr = pCache->filteredBlock[3][0];
		if (halfPelRef.m_y > 0)
		{
			intPtr += intStride;
		}
		filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true);
	}

	// Generate @ 1,3
	intPtr = pCache->filteredBlockTmp[3] + (halfFilterSize-1) * intStride;
	dstPtr = pCache->filteredBlock[1][3];
	if (halfPelRef.m_y == 0)
	{
		intPtr += intStride;
	}
	filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true);

	// Generate @ 3,3
	intPtr = pCache->filteredBlockTmp[3] + (halfFilterSize-1) * intStride;
	dstPtr = pCache->filteredBlock[3][3];
	filterVerLuma(intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true);
}

MVpara s_acMvRefineH(Int32 RefinePara)
{
	MVpara acMvRefineH;
	acMvRefineH.m_x = 0;
	acMvRefineH.m_y = 0;
	switch(RefinePara)
	{
	case 0:
		{
			acMvRefineH.m_x = 0;
			acMvRefineH.m_y = 0;
			return acMvRefineH;
		}
	case 1:
		{
			acMvRefineH.m_x = 0;
			acMvRefineH.m_y = -1;
			return acMvRefineH;
		}
	case 2:
		{
			acMvRefineH.m_x = 0;
			acMvRefineH.m_y = 1;
			return acMvRefineH;
		}
	case 3:
		{
			acMvRefineH.m_x = -1;
			acMvRefineH.m_y = 0;
			return acMvRefineH;
		}
	case 4:
		{
			acMvRefineH.m_x = 1;
			acMvRefineH.m_y = 0;
			return acMvRefineH;
		}
	case 5:
		{
			acMvRefineH.m_x = -1;
			acMvRefineH.m_y = -1;
			return acMvRefineH;
		}
	case 6:
		{
			acMvRefineH.m_x = 1;
			acMvRefineH.m_y = -1;
			return acMvRefineH;
		}
	case 7:
		{
			acMvRefineH.m_x = -1;
			acMvRefineH.m_y = 1;
			return acMvRefineH;
		}
	case 8:
		{
			acMvRefineH.m_x = 1;
			acMvRefineH.m_y = 1;
			return acMvRefineH;
		}
	default:
		{
			acMvRefineH.m_x = 0;
			acMvRefineH.m_y = 0;
			return acMvRefineH;
		}
	}
}

MVpara s_acMvRefineQ(Int32 RefinePara)
{
	MVpara acMvRefineQ;
	acMvRefineQ.m_x = 0;
	acMvRefineQ.m_y = 0;
	switch(RefinePara)
	{
	case 0:
		{
			acMvRefineQ.m_x = 0;
			acMvRefineQ.m_y = 0;
			return acMvRefineQ;
		}
	case 1:
		{
			acMvRefineQ.m_x = 0;
			acMvRefineQ.m_y = -1;
			return acMvRefineQ;
		}
	case 2:
		{
			acMvRefineQ.m_x = 0;
			acMvRefineQ.m_y = 1;
			return acMvRefineQ;
		}
	case 3:
		{
			acMvRefineQ.m_x = -1;
			acMvRefineQ.m_y = 0;
			return acMvRefineQ;
		}
	case 4:
		{
			acMvRefineQ.m_x = 1;
			acMvRefineQ.m_y = 0;
			return acMvRefineQ;
		}
	case 5:
		{
			acMvRefineQ.m_x = -1;
			acMvRefineQ.m_y = -1;
			return acMvRefineQ;
		}
	case 6:
		{
			acMvRefineQ.m_x = 1;
			acMvRefineQ.m_y = -1;
			return acMvRefineQ;
		}
	case 7:
		{
			acMvRefineQ.m_x = -1;
			acMvRefineQ.m_y = 1;
			return acMvRefineQ;
		}
	case 8:
		{
			acMvRefineQ.m_x = 1;
			acMvRefineQ.m_y = 1;
			return acMvRefineQ;
		}
	default:
		{
			acMvRefineQ.m_x = 0;
			acMvRefineQ.m_y = 0;
			return acMvRefineQ;
		}
	}
}

pcMvRefine* pcMvRefineN[2] = {
	s_acMvRefineH,
	s_acMvRefineQ
};

double xPatternRefinement( MTC265_t *h, UInt8 uiDepth, MTC265_PartSize ePartSize, MVpara baseRefMv, Int iFrac, MVpara& rcMvFrac )
{
	MTC265_Cache* pCache = &h->cache;
	double uiDist;
	double uiDistBest  = MAX_DOUBLE;
	Int32 Cost_cMvTest_X;
	Int32 Cost_cMvTest_Y;
	UInt uiDirecBest = 0;

	Pel* piRefPos;
	Int iRefStride = MAX_CU_SIZE + 16;

	pcMvRefine* pcMvRefineTmp = (iFrac == 2 ? pcMvRefineN[0] : pcMvRefineN[1]);
	for (UInt i = 0; i < 9; i++)
	{
		MVpara cMvTest;
		cMvTest.m_x = pcMvRefineTmp(i).m_x;
		cMvTest.m_y = pcMvRefineTmp(i).m_y;
		cMvTest.m_x += baseRefMv.m_x;
		cMvTest.m_y += baseRefMv.m_y;

		Int horVal = cMvTest.m_x * iFrac;
		Int verVal = cMvTest.m_y * iFrac;
		piRefPos = pCache->filteredBlock[ verVal & 3 ][ horVal & 3 ];
		if ( horVal == 2 && ( verVal & 1 ) == 0 )
			piRefPos += 1;
		if ( ( horVal & 1 ) == 0 && verVal == 2 )
			piRefPos += iRefStride;
		cMvTest.m_x = pcMvRefineTmp(i).m_x;
		cMvTest.m_y = pcMvRefineTmp(i).m_y;
		cMvTest.m_x += rcMvFrac.m_x;
		cMvTest.m_y += rcMvFrac.m_y;

		uiDist = CalInterDistortion( pCache->pucPixY + pCache->subPUY * MAX_CU_SIZE + pCache->subPUX, piRefPos, MAX_CU_SIZE, iRefStride ,h->ucMaxCUWidth>>uiDepth, ePartSize);
		calcMvdBinNumber(cMvTest.m_x, Cost_cMvTest_X);
		calcMvdBinNumber(cMvTest.m_y, Cost_cMvTest_Y);
		
		uiDist += h->sqrt_Inter_lamada * (Cost_cMvTest_X + Cost_cMvTest_Y);

		if ( uiDist < uiDistBest )
		{
			uiDistBest  = uiDist;
			uiDirecBest = i;
		}
	}

	rcMvFrac.m_x = pcMvRefineTmp(uiDirecBest).m_x;
	rcMvFrac.m_y = pcMvRefineTmp(uiDirecBest).m_y;
	return uiDistBest;
}

void xCheckBestMVP( MTC265_t *h, Interpara * StartInter, UInt MVPidxNum)
{
	MTC265_Cache* pCache = &h->cache;
	UInt32 subPUX = pCache->subPUX;
	UInt32 subPUY = pCache->subPUY;
	Int32 BinNumberMvd0_X = 0;
	Int32 BinNumberMvd0_Y = 0;
	Int32 BinNumberMvd1_X = 0;
	Int32 BinNumberMvd1_Y = 0;
	Int32 tempMvp0_SrchX;
	Int32 tempMvp0_SrchY;
	Int32 tempMvp1_SrchX;
	Int32 tempMvp1_SrchY;
	Int32 tempMvd0_X;
	Int32 tempMvd0_Y;
	Int32 tempMvd1_X;
	Int32 tempMvd1_Y;
	Int32 tempCost0;
	Int32 tempCost1;

	if (MVPidxNum == 1)
		return;
	else//(MVPidxNum == 2)
	{
		if (StartInter->iMvp_index == 0)
		{
			calcMvdBinNumber(StartInter->iMvdBestY.m_x, BinNumberMvd0_X);
			calcMvdBinNumber(StartInter->iMvdBestY.m_y, BinNumberMvd0_Y);
			tempCost0 = BinNumberMvd0_X + BinNumberMvd0_Y;

			tempMvp1_SrchX = (StartInter->Mvp_1.m_x >> 2) + subPUX + h->uiCUX + h->usAddRefLength;
			tempMvp1_SrchY = (StartInter->Mvp_1.m_y >> 2) + subPUY + h->uiCUY + h->usAddRefLength;
			tempMvd1_X = StartInter->iSrchCenBest.m_x - tempMvp1_SrchX;
			tempMvd1_Y = StartInter->iSrchCenBest.m_y - tempMvp1_SrchY;
			calcMvdBinNumber(tempMvd1_X, BinNumberMvd1_X);
			calcMvdBinNumber(tempMvd1_Y, BinNumberMvd1_Y);
			tempCost1 = BinNumberMvd1_X + BinNumberMvd1_Y;
			if(tempCost1 < tempCost0)
			{
				StartInter->iMvp_index = 1;
				StartInter->iMvpBestY.m_x = StartInter->Mvp_1.m_x;
				StartInter->iMvpBestY.m_y = StartInter->Mvp_1.m_y;
				StartInter->iMvdBestY.m_x = tempMvd1_X;
				StartInter->iMvdBestY.m_y = tempMvd1_Y;
			}
		}
		else
		{
			calcMvdBinNumber(StartInter->iMvdBestY.m_x, BinNumberMvd1_X);
			calcMvdBinNumber(StartInter->iMvdBestY.m_y, BinNumberMvd1_Y);
			tempCost1 = BinNumberMvd1_X + BinNumberMvd1_Y;

			tempMvp0_SrchX = (StartInter->Mvp_0.m_x >> 2) + subPUX + h->uiCUX + h->usAddRefLength;
			tempMvp0_SrchY = (StartInter->Mvp_0.m_y >> 2) + subPUY + h->uiCUY + h->usAddRefLength;
			tempMvd0_X = StartInter->iSrchCenBest.m_x - tempMvp0_SrchX;
			tempMvd0_Y = StartInter->iSrchCenBest.m_y - tempMvp0_SrchY;
			calcMvdBinNumber(tempMvd0_X, BinNumberMvd0_X);
			calcMvdBinNumber(tempMvd0_Y, BinNumberMvd0_Y);
			tempCost0 = BinNumberMvd0_X + BinNumberMvd0_Y;
			if(tempCost0 < tempCost1)
			{
				StartInter->iMvp_index = 0;
				StartInter->iMvpBestY.m_x = StartInter->Mvp_0.m_x;
				StartInter->iMvpBestY.m_y = StartInter->Mvp_0.m_y;
				StartInter->iMvdBestY.m_x = tempMvd0_X;
				StartInter->iMvdBestY.m_y = tempMvd0_Y;
			}
		}
	}
}

void deriveLeftTopIdxGeneral( MTC265_t *h, UInt uiLCUIdx, UInt uiCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt uiPUIdx, UInt &ruiPartIdxLT )
{
	UInt uiHorOffset, uiVerOffset;
	if( !uiPUIdx )
	{
		ruiPartIdxLT = uiCUIdx;
		return;
	}
	switch( eCUMode )
	{
	case SIZE_2Nx2N:
		ruiPartIdxLT = uiCUIdx;
		return;
	case SIZE_2NxN:
		uiHorOffset = 0;
		uiVerOffset = 1 << ( h->ucMaxCUDepth - uiDepth -1 );
		break;
	case SIZE_Nx2N:
		uiHorOffset = 1 << ( h->ucMaxCUDepth - uiDepth -1 );
		uiVerOffset = 0;
		break;
	case SIZE_NxN:
		if( uiPUIdx & 0x1 )
			uiHorOffset = 1 << ( h->ucMaxCUDepth - uiDepth -1 );
		else
			uiHorOffset = 0;

		if( uiPUIdx & 0x2 )
			uiVerOffset = 1 << ( h->ucMaxCUDepth - uiDepth -1 );
		else
			uiVerOffset = 0;
		break;
	}
	ruiPartIdxLT = g_auiRasterToZscan[g_auiZscanToRaster[uiCUIdx] + uiHorOffset + uiVerOffset * (1 << h->ucMaxCUDepth)];
}

void deriveRightTopIdxGeneral( MTC265_t *h, UInt uiLCUIdx, UInt uiCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt uiPUIdx, UInt &ruiPartIdxRT )
{
	UInt uiHorOffset, uiVerOffset;
	switch( eCUMode )
	{
	case SIZE_2Nx2N:
		uiHorOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		uiVerOffset = 0;
		break;
	case SIZE_2NxN:
		uiHorOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		uiVerOffset = uiPUIdx * ( 1 << ( h->ucMaxCUDepth - uiDepth -1 ));
		break;
	case SIZE_Nx2N:
		uiHorOffset = ( uiPUIdx + 1 ) * ( 1 << ( h->ucMaxCUDepth - uiDepth -1 )) - 1;
		uiVerOffset = 0;
		break;
	case SIZE_NxN:
		if( uiPUIdx & 0x1 )
			uiHorOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		else
			uiHorOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth -1 )) - 1;

		if( uiPUIdx & 0x2 )
			uiVerOffset = 1 << ( h->ucMaxCUDepth - uiDepth -1 );
		else
			uiVerOffset = 0;
		break;
	}
	ruiPartIdxRT = g_auiRasterToZscan[g_auiZscanToRaster[uiCUIdx] + uiHorOffset + uiVerOffset * (1 << h->ucMaxCUDepth)];
}

void deriveLeftBottomIdxGeneral( MTC265_t *h, UInt uiLCUIdx, UInt uiCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt uiPUIdx, UInt &ruiPartIdxLB )
{
	UInt uiHorOffset, uiVerOffset;
	switch( eCUMode )
	{
	case SIZE_2Nx2N:
		uiHorOffset = 0;
		uiVerOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		break;
	case SIZE_2NxN:
		uiHorOffset = 0;
		uiVerOffset = ( uiPUIdx + 1 ) * ( 1 << ( h->ucMaxCUDepth - uiDepth -1 )) - 1;
		break;
	case SIZE_Nx2N:
		uiHorOffset = uiPUIdx * ( 1 << ( h->ucMaxCUDepth - uiDepth -1 ));
		uiVerOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		break;
	case SIZE_NxN:
		if( uiPUIdx & 0x1 )
			uiHorOffset = 1 << ( h->ucMaxCUDepth - uiDepth -1 );
		else
			uiHorOffset = 0;
		if( uiPUIdx & 0x2 )
			uiVerOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		else
			uiVerOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth -1 )) - 1;
		break;
	}
	ruiPartIdxLB = g_auiRasterToZscan[g_auiZscanToRaster[uiCUIdx] + uiHorOffset + uiVerOffset * (1 << h->ucMaxCUDepth)];
}

void deriveCenterIdxGeneral(MTC265_t *h, UInt uiPUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt &ruiPartIdxCenter)
{
	UInt nCUSize = h->ucMaxCUWidth >> uiDepth;
	UInt iPartWidth = 0;
	UInt iPartHeight = 0;
	switch( eCUMode )
	{
	case SIZE_2Nx2N:
		iPartWidth = nCUSize;
		iPartHeight = nCUSize;
		break;
	case SIZE_2NxN:
		iPartWidth = nCUSize;
		iPartHeight = nCUSize >> 1;
		break;
	case SIZE_Nx2N:
		iPartWidth = nCUSize >> 1;
		iPartHeight = nCUSize;
		break;
	case SIZE_NxN:
		iPartWidth = nCUSize >> 1;
		iPartHeight = nCUSize >> 1;
		break;
	}
	ruiPartIdxCenter = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPUIdx ] + ( iPartHeight/4  )/2*(h->ucMaxCUWidth/4) + ( iPartWidth/4 )/2];
}

void deriveRightBottomIdxGeneral( MTC265_t *h, UInt uiLCUIdx, UInt uiCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt uiPUIdx, UInt &ruiPartIdxRB )
{
	UInt uiHorOffset, uiVerOffset;
	switch( eCUMode )
	{
	case SIZE_2Nx2N:
		uiHorOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		uiVerOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		break;
	case SIZE_2NxN:
		uiHorOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		uiVerOffset = ( uiPUIdx + 1 ) * ( 1 << ( h->ucMaxCUDepth - uiDepth -1 )) - 1;
		break;
	case SIZE_Nx2N:
		uiHorOffset = ( uiPUIdx + 1 ) * ( 1 << ( h->ucMaxCUDepth - uiDepth -1 )) - 1;
		uiVerOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		break;
	case SIZE_NxN:
		if( uiPUIdx & 0x1 )
			uiHorOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		else
			uiHorOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth -1 )) - 1;
		if( uiPUIdx & 0x2 )
			uiVerOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth )) - 1;
		else
			uiVerOffset = ( 1 << ( h->ucMaxCUDepth - uiDepth -1 )) - 1;
		break;
	}
	ruiPartIdxRB = g_auiRasterToZscan[g_auiZscanToRaster[uiCUIdx] + uiHorOffset + uiVerOffset * (1 << h->ucMaxCUDepth)];
}

bool getPULeft( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPartIdx, UInt &ruiLeftLCUIdx, UInt &ruiLeftPartIdx )
{
	UInt uiNumPartInLCUWidth, uiNumLCUInPicWidth, uiCurPartRasterIdx;
	uiNumPartInLCUWidth = 1 << h->ucMaxCUDepth;
	uiNumLCUInPicWidth = ( h->usWidth + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;
	uiCurPartRasterIdx = g_auiZscanToRaster[uiCurPartIdx];

	if( uiCurPartRasterIdx % uiNumPartInLCUWidth == 0 )
	{
		if( uiCurLCUIdx % uiNumLCUInPicWidth == 0 )
			return false;
		ruiLeftPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiNumPartInLCUWidth - 1];
		ruiLeftLCUIdx = uiCurLCUIdx - 1;
	}
	else
	{
		ruiLeftPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx - 1];
		ruiLeftLCUIdx = uiCurLCUIdx;
	}
	return true;
}

bool getPUAbove( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPartIdx, UInt &ruiAboveLCUIdx, UInt &ruiAbovePartIdx )
{
	UInt uiNumPartInLCUWidth, uiNumLCUInPicWidth, uiCurPartRasterIdx;
	uiNumPartInLCUWidth = 1 << h->ucMaxCUDepth;
	uiNumLCUInPicWidth = ( h->usWidth + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;
	uiCurPartRasterIdx = g_auiZscanToRaster[uiCurPartIdx];

	if( uiCurPartRasterIdx < uiNumPartInLCUWidth )
	{
		if( uiCurLCUIdx < uiNumLCUInPicWidth )
			return false;
		ruiAbovePartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiNumPartInLCUWidth * (uiNumPartInLCUWidth - 1)];
		ruiAboveLCUIdx = uiCurLCUIdx - uiNumLCUInPicWidth;
	}
	else
	{
		ruiAbovePartIdx = g_auiRasterToZscan[uiCurPartRasterIdx - uiNumPartInLCUWidth];
		ruiAboveLCUIdx = uiCurLCUIdx;
	}

	ruiAbovePartIdx = g_motionRefer[ruiAbovePartIdx];
	return true;
}

bool getPUAboveLeft( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPartIdx, UInt &ruiAboveLeftLCUIdx, UInt &ruiAboveLeftPartIdx )
{
	UInt uiNumPartInLCUWidth, uiNumLCUInPicWidth, uiCurPartRasterIdx;
	Int uiHorOffset, uiVerOffset;

	uiNumPartInLCUWidth = 1 << h->ucMaxCUDepth;
	uiNumLCUInPicWidth = ( h->usWidth + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;
	uiCurPartRasterIdx = g_auiZscanToRaster[uiCurPartIdx];
	uiHorOffset = 0;
	uiVerOffset = 0;
	ruiAboveLeftLCUIdx = uiCurLCUIdx;

	if( uiCurPartRasterIdx % uiNumPartInLCUWidth == 0 )
	{
		if( uiCurLCUIdx % uiNumLCUInPicWidth == 0 )
			return false;
		uiHorOffset += uiNumPartInLCUWidth - 1;
		ruiAboveLeftLCUIdx -= 1;
	}
	else
		uiHorOffset -= 1;

	if( uiCurPartRasterIdx < uiNumPartInLCUWidth )
	{
		if( uiCurLCUIdx < uiNumLCUInPicWidth )
			return false;
		uiVerOffset += uiNumPartInLCUWidth * (uiNumPartInLCUWidth - 1);
		ruiAboveLeftLCUIdx -= uiNumLCUInPicWidth;
		ruiAboveLeftPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiHorOffset + uiVerOffset];
	}
	else
	{
		uiVerOffset -= 1;
		ruiAboveLeftPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiHorOffset + uiVerOffset*uiNumPartInLCUWidth];
	}
	

	ruiAboveLeftPartIdx = g_motionRefer[ruiAboveLeftPartIdx];
	return true;
}

bool getPUAboveRight( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPartIdx, UInt &ruiAboveRightLCUIdx, UInt &ruiAboveRightPartIdx )
{
	UInt uiNumPartInLCUWidth, uiNumLCUInPicWidth, uiCurPartRasterIdx;
	Int uiHorOffset, uiVerOffset;

	uiNumPartInLCUWidth = 1 << h->ucMaxCUDepth;
	uiNumLCUInPicWidth = ( h->usWidth + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;
	uiCurPartRasterIdx = g_auiZscanToRaster[uiCurPartIdx];
	uiHorOffset = 0;
	uiVerOffset = 0;
	ruiAboveRightLCUIdx = uiCurLCUIdx;

	if( uiCurPartRasterIdx % uiNumPartInLCUWidth == uiNumPartInLCUWidth - 1 )
	{
		if( uiCurLCUIdx % uiNumLCUInPicWidth == uiNumLCUInPicWidth - 1 )
			return false;
		uiHorOffset -= uiNumPartInLCUWidth - 1;
		ruiAboveRightLCUIdx += 1;
	}
	else
		uiHorOffset += 1;

	if( uiCurPartRasterIdx < uiNumPartInLCUWidth )
	{
		if( uiCurLCUIdx < uiNumLCUInPicWidth )
			return false;
		uiVerOffset += uiNumPartInLCUWidth * (uiNumPartInLCUWidth - 1);
		ruiAboveRightLCUIdx -= uiNumLCUInPicWidth;
		ruiAboveRightPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiHorOffset + uiVerOffset];
	}
	else
	{
		uiVerOffset -= 1;
	    ruiAboveRightPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiHorOffset + uiVerOffset*uiNumPartInLCUWidth];
	}

	if( ruiAboveRightLCUIdx > uiCurLCUIdx )
		return false;
	if( ruiAboveRightLCUIdx == uiCurLCUIdx && ruiAboveRightPartIdx > uiCurPartIdx )
		return false;

	ruiAboveRightPartIdx = g_motionRefer[ruiAboveRightPartIdx];
	return true;
}

bool getPUBelowLeft( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPartIdx, UInt &ruiBelowLeftLCUIdx, UInt &ruiBelowLeftPartIdx )
{
	UInt uiNumPartInLCUWidth, uiNumLCUInPicWidth, uiNumLCUInPicHeight, uiCurPartRasterIdx;
	Int uiHorOffset, uiVerOffset;

	uiNumPartInLCUWidth = 1 << h->ucMaxCUDepth;
	uiNumLCUInPicWidth = ( h->usWidth + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;
	uiNumLCUInPicHeight = ( h->usHeight + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;
	uiCurPartRasterIdx = g_auiZscanToRaster[uiCurPartIdx];
	uiHorOffset = 0;
	uiVerOffset = 0;
	ruiBelowLeftLCUIdx = uiCurLCUIdx;

	if( uiCurPartRasterIdx % uiNumPartInLCUWidth == 0 )
	{
		if( uiCurLCUIdx % uiNumLCUInPicWidth == 0 )
			return false;
		uiHorOffset += uiNumPartInLCUWidth - 1;
		ruiBelowLeftLCUIdx -= 1;
	}
	else
		uiHorOffset -= 1;

	if( uiCurPartRasterIdx / uiNumPartInLCUWidth == uiNumPartInLCUWidth - 1 )
	{
		if( uiCurLCUIdx / uiNumLCUInPicWidth == uiNumLCUInPicHeight - 1 )
			return false;
		uiVerOffset -= uiNumPartInLCUWidth * (uiNumPartInLCUWidth - 1);
		ruiBelowLeftLCUIdx += uiNumLCUInPicWidth;
		ruiBelowLeftPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiHorOffset + uiVerOffset];
	}
	else
	{
		uiVerOffset += 1;
	    ruiBelowLeftPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiHorOffset + uiVerOffset * uiNumPartInLCUWidth];
	}

	if( ruiBelowLeftLCUIdx > uiCurLCUIdx )
		return false;
	if( ruiBelowLeftLCUIdx == uiCurLCUIdx && ruiBelowLeftPartIdx > uiCurPartIdx )
		return false;

	return true;
}

bool getPUBelowRight( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPartIdx, UInt &ruiBelowRightLCUIdx, UInt &ruiBelowRightPartIdx )
{
	UInt uiNumPartInLCUWidth, uiNumLCUInPicWidth, uiNumLCUInPicHeight, uiCurPartRasterIdx;
	Int uiHorOffset, uiVerOffset;

	uiNumPartInLCUWidth = 1 << h->ucMaxCUDepth;
	uiNumLCUInPicWidth = ( h->usWidth + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;
	uiNumLCUInPicHeight = ( h->usHeight + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;
	uiCurPartRasterIdx = g_auiZscanToRaster[uiCurPartIdx];
	uiHorOffset = 0;
	uiVerOffset = 0;
	ruiBelowRightLCUIdx = uiCurLCUIdx;

	if( uiCurPartRasterIdx % uiNumPartInLCUWidth == uiNumPartInLCUWidth - 1 )
	{
		if( uiCurLCUIdx % uiNumLCUInPicWidth == uiNumLCUInPicWidth - 1 )
			return false;
		uiHorOffset -= uiNumPartInLCUWidth - 1;
		ruiBelowRightLCUIdx += 1;
	}
	else
		uiHorOffset += 1;

	if( uiCurPartRasterIdx / uiNumPartInLCUWidth == uiNumPartInLCUWidth - 1 )
	{
		return false;
	}
	else
	{
		uiVerOffset += 1;
	    ruiBelowRightPartIdx = g_auiRasterToZscan[uiCurPartRasterIdx + uiHorOffset + uiVerOffset * uiNumPartInLCUWidth];
	}

	return true;
}

void FindMv( MTC265_t *h, FindMv_Mode eFindMv, Int iRefidx, UInt8 uiDepth, UInt uiCurLCUIdx, UInt uiMvpLCUIdx, UInt uiMvpPURelIdx, UInt uiCurPURelIdx, MTC265_PartSize ePartSize, FindMv_Position MvPosition, Int32 PUPartIdx, MVpara& iMv, MVpara& iMvd )//前n参考帧，赋值为-n
{
	MTC265_Cache* pCache = &h->cache;
	Interpara* StartInter = h->TempInter;
	Interpara* StartChange = 0;
	Int32 TempInterToCur = 0;
	UInt32 PURelIdx = pCache->PURelativeIdx[uiDepth];//当前PU的RelativeIdx
	MVpara* BestMvStart = h->FrameBestMv;//跨越LCU时所用的Mv
	MVpara* BestMvCur = BestMvStart + uiMvpLCUIdx*h->NumPartition + uiMvpPURelIdx;
	MVpara* PrevBestMvStart = h->PrevFrameBestMv[abs(iRefidx)-1];//TMVP，LCU时所用的Mv
	MVpara* PrevBestMvCur = PrevBestMvStart + uiMvpLCUIdx*h->NumPartition + uiMvpPURelIdx;
	MVpara* BestMvdStart = h->FrameBestMvd;//跨越LCU时所用的Mv
	MVpara* BestMvdCur = BestMvdStart + uiMvpLCUIdx*h->NumPartition + uiMvpPURelIdx;
	MVpara* PrevBestMvdStart = h->PrevFrameBestMvd[abs(iRefidx)-1];//TMVP，LCU时所用的Mv
	MVpara* PrevBestMvdCur = PrevBestMvdStart + uiMvpLCUIdx*h->NumPartition + uiMvpPURelIdx;

	MVpara* CacheMvStart = pCache->TemppuhMv;
	MVpara* CacheMvCur = pCache->TemppuhMv + pCache->BestpuhInterDepth[uiMvpPURelIdx] * pCache->NumPartition + uiMvpPURelIdx;
	MVpara* CacheMvdStart = pCache->TemppuhMvd;
	MVpara* CacheMvdCur = pCache->TemppuhMvd + pCache->BestpuhInterDepth[uiMvpPURelIdx] * pCache->NumPartition + uiMvpPURelIdx;
	if (iRefidx == 0)//使用当前帧
	{
		if( uiCurLCUIdx == uiMvpLCUIdx )
		{
			if ((ePartSize == SIZE_2NxN)&&(PUPartIdx == 1)&&(MvPosition == VALID_T))//此时的MV没有存储到pCache的最优MV中（for,PUpartindex的循环未结束）
			{
				TempInterToCur = uiCurLCUIdx*h->ucMaxCUDepth*pCache->NumPartition*h->PUnumber + uiDepth*pCache->NumPartition*h->PUnumber + uiMvpPURelIdx*h->PUnumber + (Int32)SIZE_2NxN;
				//以上的变量由四元组(iLCUNum, uiDepth, PUrelIdx, ePartSize)决定			
				StartChange = StartInter + TempInterToCur;
				iMv.m_x = StartChange->iMvBestY.m_x;
				iMv.m_y = StartChange->iMvBestY.m_y;
				iMvd.m_x = StartChange->iMvdBestY.m_x;
				iMvd.m_y = StartChange->iMvdBestY.m_y;
			}
			else if ((ePartSize == SIZE_Nx2N)&&(PUPartIdx == 1)&&(MvPosition == VALID_L)) 
			{
				TempInterToCur = uiCurLCUIdx*h->ucMaxCUDepth*pCache->NumPartition*h->PUnumber + uiDepth*pCache->NumPartition*h->PUnumber + uiMvpPURelIdx*h->PUnumber + (Int32)SIZE_Nx2N;
				//以上的变量由四元组(iLCUNum, uiDepth, PUrelIdx, ePartSize)决定			
				StartChange = StartInter + TempInterToCur;
				iMv.m_x = StartChange->iMvBestY.m_x;
				iMv.m_y = StartChange->iMvBestY.m_y;
				iMvd.m_x = StartChange->iMvdBestY.m_x;
				iMvd.m_y = StartChange->iMvdBestY.m_y;
			}
			else
			{
				iMv.m_x = CacheMvCur->m_x;
				iMv.m_y = CacheMvCur->m_y;
				iMvd.m_x = CacheMvdCur->m_x;
				iMvd.m_y = CacheMvdCur->m_y;
			}
		}
		else//跨越LCU时，需要从h->FrameBestMv中读取MV
		{
			iMv.m_x = BestMvCur->m_x;
			iMv.m_y = BestMvCur->m_y;
			iMvd.m_x = BestMvdCur->m_x;
			iMvd.m_y = BestMvdCur->m_y;
		}
	}
	else//TMVP，参考前n帧
	{
		iMv.m_x = PrevBestMvCur->m_x;
		iMv.m_y = PrevBestMvCur->m_y;
		iMvd.m_x = PrevBestMvdCur->m_x;
		iMvd.m_y = PrevBestMvdCur->m_y;
	}
}

void PrevChangeMv( MTC265_t *h, MVpara * PrevFrameBestMv, MVpara * CurFrameBestMv)
{
	Int32 subPUHeight = 0;
	Int32 subPUWidth = 0;
	Int32 copyPURelIdx = 0;
	Int32 PURelIdx = 0;
	Int32 iLCUNum = ((h->usWidth+h->PAD_YSIZEw)/h->ucMaxCUWidth)*((h->usHeight+h->PAD_YSIZEh)/h->ucMaxCUWidth);
	for (int iLCUIdx = 0; iLCUIdx< iLCUNum ; iLCUIdx++)
	{
		PURelIdx = 0;
		for (int k = 0;k < 4; k++)
		{
			for (int i = 0; i < h->ucMaxCUWidth/8;i++)
			{
				for (int j = 0; j < h->ucMaxCUWidth/8;j++)
				{
					copyPURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx] + i * (h->ucMaxCUWidth/4) + j];
					PrevFrameBestMv[iLCUIdx*h->NumPartition + copyPURelIdx].m_x = CurFrameBestMv[iLCUIdx*h->NumPartition + PURelIdx].m_x;
					PrevFrameBestMv[iLCUIdx*h->NumPartition + copyPURelIdx].m_y = CurFrameBestMv[iLCUIdx*h->NumPartition + PURelIdx].m_y;
				}
			}
			PURelIdx += h->ucMaxCUWidth/2;
		}
	}
}

void InterSearch( MTC265_t *h, Interpara* StartIn, UInt8 uiDepth, MTC265_PartSize ePartSize, Int32 iDist, Int32* SrchWindowRange )
{
	bool Judge[9] = {0,0,0,0,0,0,0,0,0};
	const bool bAarrayLT[9] =  {1,0,0,0,0,1,0,1,1};
	const bool bAarrayLTB[9] =  {1,0,0,1,0,1,0,1,1};
	const bool bAarrayLM[9] =  {1,0,1,1,0,1,0,1,1};
	const bool bAarrayLBT[9] =  {1,0,1,1,0,1,0,0,1};
	const bool bAarrayLB[9] =  {1,0,1,1,0,1,0,0,0};
	const bool bAarrayLT2[9] =  {1,0,0,0,0,1,1,1,1};
	const bool bAarrayLTB2[9] =  {1,1,0,1,0,1,1,1,1};
	const bool bAarrayLM2[9] =  {1,1,1,1,0,1,1,1,1};
	const bool bAarrayLBT2[9] =  {1,1,1,1,0,1,1,0,1};
	const bool bAarrayLB2[9] =  {1,1,1,1,0,1,0,0,0};
	const bool bAarrayMT3[9] =  {1,0,0,0,1,1,1,1,1};
	const bool bAarrayMTB3[9] =  {1,1,0,1,1,1,1,1,1};
	const bool bAarrayMM3[9] =  {1,1,1,1,1,1,1,1,1};
	const bool bAarrayMBT3[9] =  {1,1,1,1,1,1,1,0,1};
	const bool bAarrayMB3[9] =  {1,1,1,1,1,1,0,0,0};
	const bool bAarrayRT4[9] =  {1,0,0,0,1,0,1,1,1};
	const bool bAarrayRTB4[9] =  {1,1,0,1,1,0,1,1,1};
	const bool bAarrayRM4[9] =  {1,1,1,1,1,0,1,1,1};
	const bool bAarrayRBT4[9] =  {1,1,1,1,1,0,1,0,1};
	const bool bAarrayRB4[9] =  {1,1,1,1,1,0,0,0,0};
	const bool bAarrayRT5[9] =  {1,0,0,0,1,0,1,1,0};
	const bool bAarrayRTB5[9] =  {1,1,0,0,1,0,1,1,0};
	const bool bAarrayRM5[9] =  {1,1,1,0,1,0,1,1,0};
	const bool bAarrayRBT5[9] =  {1,1,1,0,1,0,1,0,0};
	const bool bAarrayRB5[9] =  {1,1,1,0,1,0,0,0,0};

	//Search
	if ( StartIn->iSrchCenBest.m_x < (iDist >> 1) )
	{
		if ( StartIn->iSrchCenBest.m_y < (iDist >> 1) )
		{
			memcpy(Judge,bAarrayLT,sizeof(bAarrayLT));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y < iDist) && ( StartIn->iSrchCenBest.m_y >= (iDist >> 1) ))
		{
			memcpy(Judge,bAarrayLTB,sizeof(bAarrayLTB));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y >= iDist) && ( StartIn->iSrchCenBest.m_y <= ((h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist )))
		{
			memcpy(Judge,bAarrayLM,sizeof(bAarrayLM));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if ( StartIn->iSrchCenBest.m_y > ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist ) && ( StartIn->iSrchCenBest.m_y <= ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-(iDist >> 1) )))
		{
			memcpy(Judge,bAarrayLBT,sizeof(bAarrayLBT));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else //if ( TempInter.iSrchCenBest.m_x >= ( (h->usHeight + h->PAD_YSIZEh)-iDist/2 ))
		{
			memcpy(Judge,bAarrayLB,sizeof(bAarrayLB));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);						
		}
	}
	else if (( StartIn->iSrchCenBest.m_x >= (iDist >> 1) ) && ( StartIn->iSrchCenBest.m_x < iDist))
	{
		if ( StartIn->iSrchCenBest.m_y < (iDist >> 1) )
		{
			memcpy(Judge,bAarrayLT2,sizeof(bAarrayLT2));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y < iDist) && ( StartIn->iSrchCenBest.m_y >= (iDist >> 1) ))
		{
			memcpy(Judge,bAarrayLTB2,sizeof(bAarrayLTB2));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y >= iDist) && ( StartIn->iSrchCenBest.m_y <= ((h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist )))
		{
			memcpy(Judge,bAarrayLM2,sizeof(bAarrayLM2));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if ( StartIn->iSrchCenBest.m_y > ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist ) && ( StartIn->iSrchCenBest.m_y <= ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-(iDist >> 1) )))
		{
			memcpy(Judge,bAarrayLBT2,sizeof(bAarrayLBT2));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else //if ( TempInter.iSrchCenBest.m_x >= ( (h->usHeight + h->PAD_YSIZEh)-iDist/2 ))
		{
			memcpy(Judge,bAarrayLB2,sizeof(bAarrayLB2));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);						
		}					
	}
	else if (( StartIn->iSrchCenBest.m_x >= iDist ) && ( StartIn->iSrchCenBest.m_x <= (h->usWidth + h->PAD_YSIZEw) - iDist))
	{
		if ( StartIn->iSrchCenBest.m_y < (iDist >> 1) )
		{
			memcpy(Judge,bAarrayMT3,sizeof(bAarrayMT3));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y < iDist) && ( StartIn->iSrchCenBest.m_y >= (iDist >> 1) ))
		{
			memcpy(Judge,bAarrayMTB3,sizeof(bAarrayMTB3));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y >= iDist) && ( StartIn->iSrchCenBest.m_y <= ((h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist )))
		{
			memcpy(Judge,bAarrayMM3,sizeof(bAarrayMM3));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if ( StartIn->iSrchCenBest.m_y > ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist ) && ( StartIn->iSrchCenBest.m_y <= ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-(iDist >> 1) )))
		{
			memcpy(Judge,bAarrayMBT3,sizeof(bAarrayMBT3));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else// if ( TempInter.iSrchCenBest.m_x >= ( (h->usHeight + h->PAD_YSIZEh)-iDist/2 ))
		{
			memcpy(Judge,bAarrayMB3,sizeof(bAarrayMB3));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);						
		}					
	}
	else if (( StartIn->iSrchCenBest.m_x > (h->usWidth + h->PAD_YSIZEw) - iDist ) && ( StartIn->iSrchCenBest.m_x <= (h->usWidth + h->PAD_YSIZEw) - (iDist >> 1)))
	{
		if ( StartIn->iSrchCenBest.m_y < (iDist >> 1) )
		{
			memcpy(Judge,bAarrayRT4,sizeof(bAarrayRT4));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y < iDist) && ( StartIn->iSrchCenBest.m_y >= (iDist >> 1) ))
		{
			memcpy(Judge,bAarrayRTB4,sizeof(bAarrayRTB4));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y >= iDist) && ( StartIn->iSrchCenBest.m_y <= ((h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist )))
		{
			memcpy(Judge,bAarrayRM4,sizeof(bAarrayRM4));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if ( StartIn->iSrchCenBest.m_y > ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist ) && ( StartIn->iSrchCenBest.m_y <= ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-(iDist >> 1) )))
		{
			memcpy(Judge,bAarrayRBT4,sizeof(bAarrayRBT4));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else// if ( TempInter.iSrchCenBest.m_x >= ( (h->usHeight + h->PAD_YSIZEh)-iDist/2 ))
		{
			memcpy(Judge,bAarrayRB4,sizeof(bAarrayRB4));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);						
		}
	}
	else
	{
		if ( StartIn->iSrchCenBest.m_y < (iDist >> 1) )
		{
			memcpy(Judge,bAarrayRT5,sizeof(bAarrayRT5));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y < iDist) && ( StartIn->iSrchCenBest.m_y >= (iDist >> 1) ))
		{
			memcpy(Judge,bAarrayRTB5,sizeof(bAarrayRTB5));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if (( StartIn->iSrchCenBest.m_y >= iDist) && ( StartIn->iSrchCenBest.m_y <= ((h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist )))
		{
			memcpy(Judge,bAarrayRM5,sizeof(bAarrayRM5));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else if ( StartIn->iSrchCenBest.m_y > ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-iDist ) && ( StartIn->iSrchCenBest.m_y <= ( (h->usHeight + h->PAD_YSIZEh)-h->ucMaxCUWidth-(iDist >> 1) )))
		{
			memcpy(Judge,bAarrayRBT5,sizeof(bAarrayRBT5));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);
		}
		else //if ( TempInter.iSrchCenBest.m_x >= ( (h->usHeight + h->PAD_YSIZEh)-iDist/2 ))
		{
			memcpy(Judge,bAarrayRB5,sizeof(bAarrayRB5));
			DiamondSearch ( h, StartIn, uiDepth, ePartSize, StartIn->iSrchCenBest.m_x, StartIn->iSrchCenBest.m_y, iDist, SrchWindowRange, Judge);						
		}
	}
}


Int8 xDiamondSearch ( MTC265_t *h, Interpara *StartIn, UInt8 uiDepth, MTC265_PartSize ePartSize, Int32 iCenX, Int32 iCenY, bool t )
{
	MTC265_Cache* pCache = &h->cache;
	UInt32 nCUSize = h->ucMaxCUWidth >> uiDepth;
	if ( t == 1 )
	{
		Int32 x = iCenX;//重建点
		Int32 y = iCenY;
		Int32 tempMvd_X = x - ((h->uiCUX + pCache->subPUX + h->usAddRefLength) + (StartIn->iMvpBestY.m_x >> 2));
		Int32 tempMvd_Y = y - ((h->uiCUY + pCache->subPUY + h->usAddRefLength) + (StartIn->iMvpBestY.m_y >> 2));
		Int32 MvdBinNumber_X;
		Int32 MvdBinNumber_Y;
		double TotalCost = 0;
		double SadTempY ;

		SadTempY = CalInterDistortion( pCache->pucPixY + pCache->subPUY * MAX_CU_SIZE + pCache->subPUX, h->LargeInter.pucY + y * h->usLargeWidth + x, MAX_CU_SIZE, h->usLargeWidth, nCUSize, ePartSize);
		calcMvdBinNumber(tempMvd_X << 2, MvdBinNumber_X);
		calcMvdBinNumber(tempMvd_Y << 2, MvdBinNumber_Y);
		TotalCost = SadTempY + h->sqrt_Inter_lamada *(MvdBinNumber_X + MvdBinNumber_Y);
		if ( TotalCost < StartIn->iSadBest )
		{
			StartIn->iSadBest = TotalCost;
			return 1;
		}
		else
			return 0 ;
	}
	else
		return 0 ;
}

void DiamondSearch ( MTC265_t *h, Interpara * StartIn, UInt8 uiDepth, MTC265_PartSize ePartSize, Int32 iCenX, Int32 iCenY, Int32 iDist, Int32* SrchWindowSize, bool *t )
{
	MTC265_Cache  *pCache     = &h->cache;
	Int32 xx1= iCenX;//暂存搜索中心坐标
	Int32 yy1= iCenY;
	Int8 Sadindex = 0; 
	Int8 ucPointNr = 0;
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[0] ) != 0 ) ? Sadindex : ucPointNr;
	iCenX = xx1- (iDist >> 1);//8个点分别坐标
	iCenY = yy1- (iDist >> 1);
	Sadindex = 1; 
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[1]) != 0 ) ? Sadindex : ucPointNr;
	iCenX = xx1;
	iCenY = yy1- iDist;
	Sadindex = 2;
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[2]) != 0 ) ? Sadindex : ucPointNr;
	iCenX = xx1+ (iDist >> 1);
	iCenY = yy1- (iDist >> 1);
	Sadindex = 3;
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[3]) != 0 ) ? Sadindex : ucPointNr;
	iCenX = xx1- iDist;
	iCenY = yy1;
	Sadindex = 4;
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[4] ) != 0 ) ? Sadindex : ucPointNr;
	iCenX = xx1+ iDist;
	iCenY = yy1;
	Sadindex = 5;
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[5] ) != 0 ) ? Sadindex : ucPointNr;
	iCenX = xx1- (iDist >> 1);
	iCenY = yy1+ (iDist >> 1);
	Sadindex = 6;
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[6] ) != 0 ) ? Sadindex : ucPointNr;
	iCenX = xx1;
	iCenY = yy1+ iDist;
	Sadindex = 7;
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[7] ) != 0 ) ? Sadindex : ucPointNr;
	iCenX = xx1+ (iDist >> 1);
	iCenY = yy1+ (iDist >> 1);
	Sadindex = 8;
	ucPointNr = (xDiamondSearch ( h, StartIn, uiDepth, ePartSize, iCenX, iCenY, t[8] ) != 0 ) ? Sadindex : ucPointNr;
	switch (ucPointNr)
	{
	case 1: 
		{
			StartIn->iSrchCenBest.m_x = xx1- (iDist >> 1); 
			StartIn->iSrchCenBest.m_y = yy1- (iDist >> 1); 
			StartIn->ucPointNr = 1;
			break;
		}
	case 2: 
		{
			StartIn->iSrchCenBest.m_x = xx1; 
			StartIn->iSrchCenBest.m_y = yy1- iDist; 
			StartIn->ucPointNr = 2;
			break;
		}
	case 3: 
		{
			StartIn->iSrchCenBest.m_x = xx1+ (iDist >> 1); 
			StartIn->iSrchCenBest.m_y = yy1- (iDist >> 1);
			StartIn->ucPointNr = 3;
			break;
		}
	case 4 : 
		{
			StartIn->iSrchCenBest.m_x = xx1- iDist; 
			StartIn->iSrchCenBest.m_y = yy1; 
			StartIn->ucPointNr = 4;
			break;
		}
	case 5: 
		{
			StartIn->iSrchCenBest.m_x = xx1+ iDist; 
			StartIn->iSrchCenBest.m_y = yy1; 
			StartIn->ucPointNr = 5;
			break;
		}
	case 6: 
		{
			StartIn->iSrchCenBest.m_x = xx1- (iDist >> 1); 
			StartIn->iSrchCenBest.m_y = yy1+ (iDist >> 1); 
			StartIn->ucPointNr = 6;
			break;
		}
	case 7: 
		{
			StartIn->iSrchCenBest.m_x = xx1; 
			StartIn->iSrchCenBest.m_y = yy1+ iDist;
			StartIn->ucPointNr = 7;
			break;
		}
	case 8: 
		{
			StartIn->iSrchCenBest.m_x = xx1+ (iDist >> 1); 
			StartIn->iSrchCenBest.m_y = yy1+ (iDist >> 1);
			StartIn->ucPointNr = 8;
			break;
		}
	default:
		{
			StartIn->iSrchCenBest.m_x = xx1; 
			StartIn->iSrchCenBest.m_y = yy1; 
			StartIn->ucPointNr = 0;
			break;
		}
	}
}

double CalInterDistortion(Pxl * pSrc, Pxl * pPred, Int32 strideSrc, Int32 stridePred, UInt32 CUSize, MTC265_PartSize ePartSize)
{
	double dist=0;
	if (ePartSize == SIZE_2Nx2N)
	{
		for (UInt32 i=0; i<CUSize;i++)
		{
			for (UInt32 j=0; j<CUSize;j++)
			{
				dist = dist + double(abs(pSrc[i*strideSrc+j]-pPred[i*stridePred+j]));
			}
		}
		return dist;
	}
	else if (ePartSize == SIZE_2NxN)
	{
		for (UInt32 i=0; i<CUSize/2;i++)
		{
			for (UInt32 j=0; j<CUSize;j++)
			{
				dist = dist + double(abs(pSrc[i*strideSrc+j]-pPred[i*stridePred+j]));
			}
		}
		return dist;
	}
	else //ePartSize == SIZE_Nx2N
	{
		for (UInt32 i=0; i<CUSize;i++)
		{
			for (UInt32 j=0; j<CUSize/2;j++)
			{
				dist = dist + double(abs(pSrc[i*strideSrc+j]-pPred[i*stridePred+j]));
			}
		}
		return dist;
	}
}

void MvpComparison ( MTC265_t *h, Interpara *StartIn, MTC265_PartSize ePartSize, UInt32 nCUSize, UInt32 subPUX, UInt32 subPUY)
{
	MTC265_Cache* pCache = &h->cache;
	double Mvp0_SAD = MAX_DOUBLE;
	double Mvp1_SAD = MAX_DOUBLE;
	Int32 tempMvp0_SrchX = 0;
	Int32 tempMvp0_SrchY = 0;
	Int32 tempMvp1_SrchX = 0;
	Int32 tempMvp1_SrchY = 0;
	Int32 BinNumberMvp0_X = 0;
	Int32 BinNumberMvp0_Y = 0;
	Int32 BinNumberMvp1_X = 0;
	Int32 BinNumberMvp1_Y = 0;
	double tempCost_Mvp0 = 0;
	double tempCost_Mvp1 = 0;

	if ((StartIn->Mvp_0.m_x == StartIn->Mvp_1.m_x) && (StartIn->Mvp_0.m_y == StartIn->Mvp_1.m_y))
	{
		StartIn->iMvp_index = 0;
		StartIn->iSrchCenBest.m_x = (StartIn->Mvp_0.m_x >> 2) + subPUX + h->uiCUX + h->usAddRefLength;
		StartIn->iSrchCenBest.m_y = (StartIn->Mvp_0.m_y >> 2) + subPUY + h->uiCUY + h->usAddRefLength;
		//计算MVp
		StartIn->iMvpBestY.m_x = StartIn->Mvp_0.m_x;
		StartIn->iMvpBestY.m_y = StartIn->Mvp_0.m_y;
		return;
	}

	tempMvp0_SrchX = (StartIn->Mvp_0.m_x >> 2) + subPUX + h->uiCUX + h->usAddRefLength;
	tempMvp0_SrchY = (StartIn->Mvp_0.m_y >> 2) + subPUY + h->uiCUY + h->usAddRefLength;
	Mvp0_SAD = CalInterDistortion( pCache->pucPixY + subPUY * MAX_CU_SIZE + subPUX, h->LargeInter.pucY + tempMvp0_SrchY * h->usLargeWidth + tempMvp0_SrchX, MAX_CU_SIZE, h->usLargeWidth, nCUSize, ePartSize);
	calcMvdBinNumber(StartIn->Mvp_0.m_x, BinNumberMvp0_X);
	calcMvdBinNumber(StartIn->Mvp_0.m_y, BinNumberMvp0_Y);
	tempCost_Mvp0 = Mvp0_SAD + h->sqrt_Inter_lamada * (BinNumberMvp0_X + BinNumberMvp0_Y);

	tempMvp1_SrchX = (StartIn->Mvp_1.m_x >> 2) + subPUX + h->uiCUX + h->usAddRefLength;
	tempMvp1_SrchY = (StartIn->Mvp_1.m_y >> 2) + subPUY + h->uiCUY + h->usAddRefLength;
	Mvp1_SAD = CalInterDistortion( pCache->pucPixY + subPUY * MAX_CU_SIZE + subPUX, h->LargeInter.pucY + tempMvp1_SrchY * h->usLargeWidth + tempMvp1_SrchX, MAX_CU_SIZE, h->usLargeWidth, nCUSize, ePartSize);
	calcMvdBinNumber(StartIn->Mvp_1.m_x, BinNumberMvp1_X);
	calcMvdBinNumber(StartIn->Mvp_1.m_y, BinNumberMvp1_Y);
	tempCost_Mvp1 = Mvp1_SAD + h->sqrt_Inter_lamada * (BinNumberMvp1_X + BinNumberMvp1_Y);

	if (tempCost_Mvp0 <= tempCost_Mvp1)
	{
		StartIn->iMvp_index = 0;
		StartIn->iSrchCenBest.m_x = tempMvp0_SrchX;
		StartIn->iSrchCenBest.m_y = tempMvp0_SrchY;
		//计算MVp
		StartIn->iMvpBestY.m_x = StartIn->Mvp_0.m_x; //三角形矩阵运算
		StartIn->iMvpBestY.m_y = StartIn->Mvp_0.m_y;
	}
	else
	{
		StartIn->iMvp_index = 1;
		StartIn->iSrchCenBest.m_x = tempMvp1_SrchX;
		StartIn->iSrchCenBest.m_y = tempMvp1_SrchY;
		//计算MVp
		StartIn->iMvpBestY.m_x = StartIn->Mvp_1.m_x; //三角形矩阵运算
		StartIn->iMvpBestY.m_y = StartIn->Mvp_1.m_y;
	}
}

void xEncInterCompressCU(MTC265_t* h, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, double &SAD_BestCU)
{
	MTC265_Cache* CU_Cache = &h->cache;
	CU_Cache->cuX = uiCUX;//表示当前CU左上角的X坐标点
	CU_Cache->cuY = uiCUY;//表示当前CU左上角的Y坐标点
	UInt32 nCUSize = h->ucMaxCUWidth >> uiDepth;
	Int32* PartIdx = CU_Cache->PartIdx;
	Int32* RelIdx = CU_Cache->RelativeIdx;
	UInt32 RelativeIdx = 0;
	double SAD_2Nx2N = MAX_DOUBLE;
	double SAD_2NxN = MAX_DOUBLE;
	double SAD_Nx2N = MAX_DOUBLE;
	double SAD_2Nx2NMergeBest = 0;
	double SAD_totalCU = 0;
	Int32 PURelIdxCompare_SIZE_2Nx2N[2] = {0,0};
	Int32 PURelIdxCompare_SIZE_2NxN[2] = {0,0};
	Int32 PURelIdxCompare_SIZE_Nx2N[2] = {0,0};
	Int32 EarlySkipDetection = 0;

	setInterDepth(h, uiDepth);
	xEncInterCheckRDCost(h, SIZE_2Nx2N, uiDepth, uiCUX, uiCUY, SAD_2Nx2N, PURelIdxCompare_SIZE_2Nx2N);//计算2Nx2N的PU，得到SAD_2Nx2N
	xCheckRDCostMerge2Nx2N(h, uiDepth, uiCUX, uiCUY, SAD_2Nx2N, SAD_2Nx2NMergeBest, EarlySkipDetection);

	if (!EarlySkipDetection)
	{
		xEncInterCheckRDCost(h, SIZE_2NxN, uiDepth, uiCUX, uiCUY, SAD_2NxN, PURelIdxCompare_SIZE_2NxN);//计算2NxN的PU，得到两个PU：SAD之和SAD_2NxN
		xEncInterCheckRDCost(h, SIZE_Nx2N, uiDepth, uiCUX, uiCUY, SAD_Nx2N, PURelIdxCompare_SIZE_Nx2N);//计算Nx2N的PU，得到两个PU：SAD之和SAD_Nx2N
	}
	CheckPUBestMode(h, uiDepth, PURelIdxCompare_SIZE_2Nx2N, PURelIdxCompare_SIZE_2NxN, PURelIdxCompare_SIZE_Nx2N, SAD_2Nx2NMergeBest, SAD_2NxN, SAD_Nx2N, SAD_BestCU);//SAD_2Nx2N,SAD_2NxN,SAD_Nx2N,SAD_2Nx2NMergeBest比较，并替换最优的MV,MVd.MVpIndex,PartSize;对2NxN和Nx2N的Ref进行组合

	if( uiDepth < (h->ucMaxCUDepth - 1))
	{	
		if(!EarlySkipDetection)
		{
			UInt8 nextDepth = uiDepth + 1;
			UInt32 subCUSize = h->ucMaxCUWidth>>nextDepth;
			UInt32 subCUX = 0;
			UInt32 subCUY = 0;
			double SAD_BestPartCU = 0;
			for(int partindex=0; partindex<4; partindex++)
			{
				subCUX=0;
				subCUY=0;

				PartIdx[nextDepth]= partindex;
				RelativeIdx = 0;
				for(Int32 num=0;num<=Int32(nextDepth);num++)
				{
					int ipow  = int(pow(double(4),num)); 
					RelativeIdx +=(PartIdx[num])*(CU_Cache->NumPartition)/ipow;
				}
				RelIdx[nextDepth] = RelativeIdx;//RelativeIdx代表当前的CU，0-63的位置
				xEncGetCUPosition( h, subCUSize, nextDepth, uiCUX, uiCUY, &subCUX, &subCUY );//找CU位置，当前CU的左上角坐标为CU_Cache->cuX和CU_Cache->cuY	
				subCUX -= h->uiCUX;//上面的函数中多加了h->uiCUX和h->uiCUY
				subCUY -= h->uiCUY;
				xEncInterCompressCU( h, nextDepth, subCUX, subCUY, SAD_BestPartCU);
				SAD_totalCU += SAD_BestPartCU;
			}
		}
		else
		{
			SAD_totalCU = SAD_2Nx2NMergeBest;
		}
		//放在此处，由于只能在运算完for循环的4个CU之后回到上一层才能进行比较(此时uiDepth<=1)
		CU_Cache->cuX = uiCUX;
		CU_Cache->cuY = uiCUY;
		CheckCUBestMode(h, uiDepth, SAD_totalCU, SAD_BestCU);//由于比较本层uiDepth的SAD和uiDepth+1的4个块的SAD之和,替换最优的MV,MVd.MVpIndex,PartSize,残差  and 保存最优的Depth至BestDepth
	}
}

void calcMvdBinNumber(Int32 Mvd, Int32& BinNumber)
{
	Int32 MvdMinus2 = abs(Mvd)-2;
	if (Mvd == 0)
		BinNumber = 1;
	else if ((Mvd == 1)||(Mvd == -1))
		BinNumber = 3;
	else if((MvdMinus2 == 0)||(MvdMinus2 == 1))
		BinNumber = 3 + 2;
	else if((MvdMinus2 >=2 )&&(MvdMinus2 <= 5 ))
		BinNumber = 3 + 4;
	else if((MvdMinus2 >=6 )&&(MvdMinus2 <= 13 ))
		BinNumber = 3 + 6;
	else if((MvdMinus2 >=14 )&&(MvdMinus2 <= 29 ))
		BinNumber = 3 + 8;
	else if((MvdMinus2 >=30 )&&(MvdMinus2 <= 61 ))
		BinNumber = 3 + 10;
	else if((MvdMinus2 >=62 )&&(MvdMinus2 <= 125 ))
		BinNumber = 3 + 12;
	else if((MvdMinus2 >=126 )&&(MvdMinus2 <= 253 ))
		BinNumber = 3 + 14;
	else if((MvdMinus2 >=254 )&&(MvdMinus2 <= 508 ))
		BinNumber = 3 + 16;
	else if((MvdMinus2 >=509 )&&(MvdMinus2 <= 1018 ))
		BinNumber = 3 + 18;
	else if((MvdMinus2 >=1019 )&&(MvdMinus2 <= 2038 ))
		BinNumber = 3 + 20;
	else if((MvdMinus2 >=2039 )&&(MvdMinus2 <= 4078 ))
		BinNumber = 3 + 22;
	else if((MvdMinus2 >=4079 )&&(MvdMinus2 <= 8158 ))
		BinNumber = 3 + 24;
}

void calcMergeIdxBinNumber(Int32 MergeIdx, Int32& BinNumber)
{
	if (MergeIdx == 4)
		BinNumber = 4;
	else
		BinNumber = MergeIdx + 1;
}

void xCheckRDCostMerge(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, double SAD_ME, double& MEandMergeBestSAD,	Int32& tempBinNumberCost)
{
	MTC265_Cache* pCache = &h->cache;
	UInt32 nCUSize = h->ucMaxCUWidth >> uiDepth;
	Int32 subPUWidth = 0;
	Int32 subPUHeight = 0;
	if (ePartSize == SIZE_2NxN)
	{
		subPUWidth = h->ucMaxCUWidth >> uiDepth;
		subPUHeight = h->ucMaxCUWidth >> (uiDepth+1);
	}
	else if (ePartSize == SIZE_Nx2N)
	{
		subPUWidth = h->ucMaxCUWidth >> (uiDepth+1);
		subPUHeight = h->ucMaxCUWidth >> uiDepth;
	}

	Int32* CUrelIdx = pCache->RelativeIdx;
	UInt32 CURelIdx = CUrelIdx[uiDepth];
	UInt32 PUPartIdx = pCache->PUPartIdx[uiDepth];
	Int32* PURelIdx = pCache->PURelativeIdx;
	pCache->subPUX = uiCUX;
	pCache->subPUY = uiCUY;

	Interpara *StartTempInter = h->TempInter;
	Interpara *StartChange = 0;
	Interpara *StartInter = 0;
	Int32 copyPURelIdx = 0; 
	Int32 uiLCUIdx = h->uiCUX/h->ucMaxCUWidth + (((h->usWidth+h->PAD_YSIZEw)/h->ucMaxCUWidth) * (h->uiCUY/h->ucMaxCUWidth));
	Int32 TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*pCache->NumPartition*h->PUnumber + uiDepth*pCache->NumPartition*h->PUnumber + PURelIdx[uiDepth]*h->PUnumber + (Int32)ePartSize;
	StartChange = StartTempInter + TempInterToCur;	

	MVpara MergeCandidatesMv[MRG_MAX_NUM_CANDS] = {(0,0),(0,0),(0,0),(0,0),(0,0)};
	MVpara MergeCandidatesMvd[MRG_MAX_NUM_CANDS] = {(0,0),(0,0),(0,0),(0,0),(0,0)};
	MVpara BestMergeMv = {0,0};
	MVpara BestMergeMvd = {0,0};
	MVpara BestMergeSrchCenter = {0,0};
	UInt numValidMergeCand = 0;
	Int32 uiMergeIndex = 0;
	Int32 tempMergeIdxCost = 0;
	double MergeBestSAD = 0;
	getInterMergeCandidates(h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PURelIdx[uiDepth], MergeCandidatesMv, MergeCandidatesMvd, numValidMergeCand);

	double MergeMinCost = MAX_DOUBLE;
	double uiCostCand = 0;
	UInt32 totalCoeffNxNBinEst = 0;
	Int32 BinNumber_X = 0;
	Int32 BinNumber_Y = 0;
	double uitempCostCand = 0;
	Int32 tempMerge_SrchX = 0;
	Int32 tempMerge_SrchY = 0;
	for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
	{
		tempMerge_SrchX = (MergeCandidatesMv[uiMergeCand].m_x >> 2) + uiCUX + h->uiCUX + h->usAddRefLength;
		tempMerge_SrchY = (MergeCandidatesMv[uiMergeCand].m_y >> 2) + uiCUY + h->uiCUY + h->usAddRefLength;
		uiCostCand = CalInterDistortion(pCache->pucPixY + uiCUY * MAX_CU_SIZE + uiCUX, h->LargeInter.pucY + h->usLargeWidth * tempMerge_SrchY + tempMerge_SrchX, MAX_CU_SIZE, h->usLargeWidth, nCUSize, ePartSize );
		calcMergeIdxBinNumber( uiMergeCand, tempMergeIdxCost ); 
		uitempCostCand = uiCostCand + h->sqrt_Inter_lamada * tempMergeIdxCost;
		if ( uitempCostCand < MergeMinCost )
		{
			MergeMinCost = uitempCostCand;
			BestMergeMv.m_x = MergeCandidatesMv[uiMergeCand].m_x;
			BestMergeMv.m_y = MergeCandidatesMv[uiMergeCand].m_y;			
			BestMergeMvd.m_x = MergeCandidatesMvd[uiMergeCand].m_x;
			BestMergeMvd.m_y = MergeCandidatesMvd[uiMergeCand].m_y;
			BestMergeSrchCenter.m_x = tempMerge_SrchX;
			BestMergeSrchCenter.m_y = tempMerge_SrchY;
			uiMergeIndex = uiMergeCand;
		}
	}

	if ( MergeMinCost<= SAD_ME )
	{
		calcMergeIdxBinNumber( uiMergeIndex, tempBinNumberCost);
		MEandMergeBestSAD = MergeMinCost;
		StartChange->MergeFlag = 1;
		StartChange->MergeIdx = uiMergeIndex;
		StartChange->iMvBestY.m_x = BestMergeMv.m_x;
		StartChange->iMvBestY.m_y = BestMergeMv.m_y;
		StartChange->iMvdBestY.m_x = BestMergeMvd.m_x;
		StartChange->iMvdBestY.m_y = BestMergeMvd.m_y;
		StartChange->iSrchCenBest.m_x = BestMergeSrchCenter.m_x;
		StartChange->iSrchCenBest.m_y = BestMergeSrchCenter.m_y;
		StartChange->iSadBest = MergeMinCost;

		//将Merge的PU内的全部StartChange变量均赋值为同样的值
		for (int i = 0; i < subPUHeight/4;i++)
		{
			for (int j = 0; j < subPUWidth/4;j++)
			{
				copyPURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx[uiDepth]] + i * (h->ucMaxCUWidth/4) + j];
				TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*pCache->NumPartition*h->PUnumber + uiDepth*pCache->NumPartition*h->PUnumber + copyPURelIdx*h->PUnumber + (Int32)ePartSize;
				//以下的变量由四元组(iLCUNum, uiDepth, copyPUrelIdx, ePartSize)决定
				StartInter = StartTempInter + TempInterToCur;

				StartInter->MergeFlag = StartChange->MergeFlag;
				StartInter->MergeIdx = StartChange->MergeIdx;
				StartInter->iMvBestY.m_x = StartChange->iMvBestY.m_x;
				StartInter->iMvBestY.m_y = StartChange->iMvBestY.m_y;
				StartInter->iMvdBestY.m_x = StartChange->iMvdBestY.m_x;
				StartInter->iMvdBestY.m_y = StartChange->iMvdBestY.m_y;
				StartInter->iSrchCenBest.m_x = StartChange->iSrchCenBest.m_x;
				StartInter->iSrchCenBest.m_y = StartChange->iSrchCenBest.m_y;
				StartInter->iSadBest = StartChange->iSadBest;
			}
		}
	}
	else
	{
		MEandMergeBestSAD = SAD_ME;
		calcMvdBinNumber(StartChange->iMvdBestY.m_x, BinNumber_X);
		calcMvdBinNumber(StartChange->iMvdBestY.m_y, BinNumber_Y);
		tempBinNumberCost = BinNumber_X + BinNumber_Y;
	}
}

void xCheckRDCostMerge2Nx2N(MTC265_t *h, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, double SAD_2Nx2N, double &SAD_BestCU, Int32& EarlySkipDetection)
{
	MTC265_Cache* CU_Cache = &h->cache;
	UInt32 nCUSize = h->ucMaxCUWidth>>uiDepth;
	Int32* CUrelIdx = CU_Cache->RelativeIdx;
	UInt32 CURelIdx = CUrelIdx[uiDepth];
	UInt32 PUPartIdx = 0;
	Int32* PURelIdx = CU_Cache->PURelativeIdx;
	PURelIdx[uiDepth] = CUrelIdx[uiDepth];
	MTC265_PartSize ePartSize = SIZE_2Nx2N; 
	CU_Cache->subPUX = uiCUX;
	CU_Cache->subPUY = uiCUY;
	CU_Cache->cuX = uiCUX;
	CU_Cache->cuY = uiCUY;

	Interpara *StartTempInter = h->TempInter;
	Interpara *StartChange = 0;
	Interpara *StartInter = 0;
	Int32 copyPURelIdx = 0; 
	Int32 uiLCUIdx = h->uiCUX/h->ucMaxCUWidth + (((h->usWidth+h->PAD_YSIZEw)/h->ucMaxCUWidth) * (h->uiCUY/h->ucMaxCUWidth));
	Int32 TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*CU_Cache->NumPartition*h->PUnumber + uiDepth*CU_Cache->NumPartition*h->PUnumber + PURelIdx[uiDepth]*h->PUnumber + (Int32)ePartSize;
	StartChange = StartTempInter + TempInterToCur;	

	MVpara MergeCandidatesMv[MRG_MAX_NUM_CANDS] = {(0,0),(0,0),(0,0),(0,0),(0,0)};
	MVpara MergeCandidatesMvd[MRG_MAX_NUM_CANDS] = {(0,0),(0,0),(0,0),(0,0),(0,0)};
	MVpara BestMergeMv = {0,0};
	MVpara BestMergeMvd = {0,0};
	UInt numValidMergeCand = 0;
	Int32 uiMergeIndex = 0;
	double MergeBestSAD = 0;
	getInterMergeCandidates(h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PURelIdx[uiDepth], MergeCandidatesMv, MergeCandidatesMvd, numValidMergeCand);

	double MergeMinCost = MAX_DOUBLE;
	double uiCostCand = 0;
	UInt32 totalCoeffNxNBinEst = 0;
	Int32 tempMergeIdxCost = 0;
	Int32 BinNumber_X = 0;
	Int32 BinNumber_Y = 0;
	double uitempCostCand = 0;
	Int32 tempMerge_SrchX = 0;
	Int32 tempMerge_SrchY = 0;
	Int32 BestMerge_SrchX = 0;
	Int32 BestMerge_SrchY = 0;
	for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
	{
		tempMerge_SrchX = (MergeCandidatesMv[uiMergeCand].m_x >> 2) + uiCUX + h->uiCUX + h->usAddRefLength;
		tempMerge_SrchY = (MergeCandidatesMv[uiMergeCand].m_y >> 2) + uiCUY + h->uiCUY + h->usAddRefLength;

		uiCostCand = CalInterDistortion(CU_Cache->pucPixY + uiCUY * MAX_CU_SIZE + uiCUX, h->LargeInter.pucY + h->usLargeWidth * tempMerge_SrchY + tempMerge_SrchX, MAX_CU_SIZE, h->usLargeWidth, nCUSize, ePartSize );
		calcMergeIdxBinNumber( uiMergeCand, tempMergeIdxCost);
		uitempCostCand = uiCostCand + h->sqrt_Inter_lamada *  tempMergeIdxCost;
		if ( uitempCostCand < MergeMinCost )
		{
			MergeMinCost = uitempCostCand;
			BestMergeMv.m_x = MergeCandidatesMv[uiMergeCand].m_x;
			BestMergeMv.m_y = MergeCandidatesMv[uiMergeCand].m_y;
			BestMergeMvd.m_x = MergeCandidatesMvd[uiMergeCand].m_x;
			BestMergeMvd.m_y = MergeCandidatesMvd[uiMergeCand].m_y;
			BestMerge_SrchX = tempMerge_SrchX;
			BestMerge_SrchY = tempMerge_SrchY;
			uiMergeIndex = uiMergeCand;
		}
	}

	if ( MergeMinCost <= SAD_2Nx2N )//在ME更优时，checkPUBestMode时进行loadPUtoRef
	{
		calcMergeIdxBinNumber( uiMergeIndex, tempMergeIdxCost);
		SAD_BestCU = MergeMinCost;
		StartChange->MergeFlag = 1;
		StartChange->MergeIdx = uiMergeIndex;
		StartChange->iMvBestY.m_x = BestMergeMv.m_x;
		StartChange->iMvBestY.m_y = BestMergeMv.m_y;
		StartChange->iMvdBestY.m_x = BestMergeMvd.m_x;
		StartChange->iMvdBestY.m_y = BestMergeMvd.m_y;
		StartChange->iSrchCenBest.m_x = BestMerge_SrchX;
		StartChange->iSrchCenBest.m_y = BestMerge_SrchY;
		StartChange->iSadBest = MergeMinCost;

		//将Merge的PU内的全部StartChange变量均赋值为同样的值
		for (UInt i = 0; i < nCUSize/4;i++)
		{
			for (UInt j = 0; j < nCUSize/4;j++)
			{
				copyPURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx[uiDepth]] + i * (h->ucMaxCUWidth/4) + j];
				TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*CU_Cache->NumPartition*h->PUnumber + uiDepth*CU_Cache->NumPartition*h->PUnumber + copyPURelIdx*h->PUnumber + (Int32)ePartSize;
				//以下的变量由四元组(iLCUNum, uiDepth, copyPUrelIdx, ePartSize)决定
				StartInter = StartTempInter + TempInterToCur;

				StartInter->MergeFlag = StartChange->MergeFlag;
				StartInter->MergeIdx = StartChange->MergeIdx;
				StartInter->iMvBestY.m_x = StartChange->iMvBestY.m_x;
				StartInter->iMvBestY.m_y = StartChange->iMvBestY.m_y;
				StartInter->iMvdBestY.m_x = StartChange->iMvdBestY.m_x;
				StartInter->iMvdBestY.m_y = StartChange->iMvdBestY.m_y;
				StartInter->iSrchCenBest.m_x = StartChange->iSrchCenBest.m_x;
				StartInter->iSrchCenBest.m_y = StartChange->iSrchCenBest.m_y;
				StartInter->iSadBest = StartChange->iSadBest;
			}
		}

		//Copy到Ref中，进行变换R-D比较
		MotionCompensation( h, StartChange->iMvBestY, StartChange->iSrchCenBest.m_x, StartChange->iSrchCenBest.m_y, uiDepth, ePartSize, 0, nCUSize, nCUSize);
		//变换得到Distortion
		xEncInterDistortion( h, ePartSize, uiDepth, uiCUX, uiCUY, totalCoeffNxNBinEst, uiCostCand);
		SAD_BestCU = uiCostCand + h->Inter_lamada *  (totalCoeffNxNBinEst + tempMergeIdxCost);

		//Motion Compensation
		Int nQP = h->iQP;
		Int nQPC = g_aucChromaScale[nQP];
		UInt32 uiSumY, uiSumC[2];
		UInt wbitY=0,wbitC=0;
		UInt8 pCbfY = 0;
		UInt8 pCbfU = 0;
		UInt8 pCbfV = 0;
		UInt8 bRootCbf = 0;
		Pxl* pucInterPredY = CU_Cache->TemppuhRefY + uiDepth * MAX_CU_SIZE * MAX_CU_SIZE + uiCUY * MAX_CU_SIZE + uiCUX;
		Pxl* pucInterPredC[2] = { CU_Cache->TemppuhRefU + uiDepth * MAX_CU_SIZE/2 * MAX_CU_SIZE/2 + uiCUY/2 * MAX_CU_SIZE/2 + uiCUX/2,CU_Cache->TemppuhRefV + uiDepth * MAX_CU_SIZE/2 * MAX_CU_SIZE/2 + uiCUY/2 * MAX_CU_SIZE/2 + uiCUX/2};

		Pxl* pucInterPixY = CU_Cache->pucPixY + MAX_CU_SIZE *(uiCUY) + uiCUX;
		Pxl* pucInterPixC[2]  = { CU_Cache->pucPixU + MAX_CU_SIZE/2 *(uiCUY/2) + uiCUX/2, CU_Cache->pucPixV + MAX_CU_SIZE/2 *(uiCUY/2) + uiCUX/2 };
		Int16* piInterTmp0      = CU_Cache->piTmp[0];
		Int16* piInterTmp1      = CU_Cache->piTmp[1];
		Int16* piInterCoefY     = CU_Cache->psTempCoefY[ uiDepth]+(MAX_CU_SIZE)*(uiCUY) + uiCUX;
		Int16* piInterCoefC[2]  = { CU_Cache->psTempCoefU[ uiDepth]+(MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2, CU_Cache->psTempCoefV[ uiDepth]+(MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2 };

		xSubDct( piInterTmp0, pucInterPixY , pucInterPredY, MAX_CU_SIZE, piInterTmp0, piInterTmp1, nCUSize, nCUSize, MODE_INVALID );
		uiSumY = xQuant( piInterCoefY, piInterTmp0, MAX_CU_SIZE, nQP, nCUSize, nCUSize, SLICE_P );

		for(int i=0; i<2; i++ ) 
		{
			xSubDct( piInterTmp0, pucInterPixC[i], pucInterPredC[i], MAX_CU_SIZE/2, piInterTmp0, piInterTmp1, nCUSize/2, nCUSize/2, MODE_INVALID );
			for (UInt32 i1=0; i1<nCUSize/2;i1++)
			{
				for (UInt32 j=0; j<nCUSize/2;j++)
				{
					wbitC += abs(piInterCoefC[i][i1*(MAX_CU_SIZE/2)+j]);
				}
			}
			uiSumC[i] = xQuant( piInterCoefC[i], piInterTmp0, MAX_CU_SIZE/2, nQPC, nCUSize/2, nCUSize/2, SLICE_P);
		}
		pCbfY = (uiSumY    != 0);
		pCbfU = (uiSumC[0] != 0);
		pCbfV = (uiSumC[1] != 0);
		bRootCbf = pCbfY || pCbfU || pCbfV;
		if (bRootCbf == 0)
		{
			EarlySkipDetection = 1;
			StartChange->SkipFlag = 1;
			StartChange->MergeIdx = uiMergeIndex;
			StartChange->iMvBestY.m_x = MergeCandidatesMv[uiMergeIndex].m_x;
			StartChange->iMvBestY.m_y = MergeCandidatesMv[uiMergeIndex].m_y;
			StartChange->iSrchCenBest.m_x = BestMerge_SrchX;
			StartChange->iSrchCenBest.m_y = BestMerge_SrchY;

			//将PU内的全部StartChange变量均赋值为同样的值
			for (UInt i = 0; i < nCUSize/4;i++)
			{
				for (UInt j = 0; j < nCUSize/4;j++)
				{
					copyPURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx[uiDepth]] + i * (h->ucMaxCUWidth/4) + j];
					TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*CU_Cache->NumPartition*h->PUnumber + uiDepth*CU_Cache->NumPartition*h->PUnumber + copyPURelIdx*h->PUnumber + (Int32)ePartSize;
					//以下的变量由四元组(iLCUNum, uiDepth, copyPUrelIdx, ePartSize)决定
					StartInter = StartTempInter + TempInterToCur;

					StartInter->SkipFlag = StartChange->SkipFlag;
					StartInter->MergeIdx = StartChange->MergeIdx;
					StartInter->iMvBestY.m_x = StartChange->iMvBestY.m_x;
					StartInter->iMvBestY.m_y = StartChange->iMvBestY.m_y;
					StartInter->iSrchCenBest.m_x = StartChange->iSrchCenBest.m_x;
					StartInter->iSrchCenBest.m_y = StartChange->iSrchCenBest.m_y;
				}
			}
		}
	}
	else
	{
		SAD_BestCU = SAD_2Nx2N;
		calcMvdBinNumber(StartChange->iMvdBestY.m_x, BinNumber_X);
		calcMvdBinNumber(StartChange->iMvdBestY.m_y, BinNumber_Y);
		tempMergeIdxCost = BinNumber_X + BinNumber_Y;

		//Copy到Ref中，进行变换R-D比较
		MotionCompensation( h, StartChange->iMvBestY, StartChange->iSrchCenBest.m_x, StartChange->iSrchCenBest.m_y, uiDepth, ePartSize, 0, nCUSize, nCUSize);
		//变换得到Distortion
		xEncInterDistortion( h, ePartSize, uiDepth, uiCUX, uiCUY, totalCoeffNxNBinEst, uiCostCand);
		SAD_BestCU = uiCostCand + h->Inter_lamada *  (totalCoeffNxNBinEst + tempMergeIdxCost);
	}
}

void getInterMergeCandidates( MTC265_t *h, UInt uiLCUIdx, UInt CURelIdx, UInt uiDepth, MTC265_PartSize ePartSize, UInt PURelIdx, MVpara *MergeCandidatesMv, MVpara *MergeCandidatesMvd, UInt& numValidMergeCand )
{
	MTC265_Cache* pCache = &h->cache;
	Int32* PUpartIdx = pCache->PUPartIdx;
	Int32 PUPartIdx = PUpartIdx[uiDepth];
	MVpara tempMergeMv = {0,0};
	MVpara tempLeftMv = {0,0};
	MVpara tempAboveMv = {0,0};
	MVpara tempBelowLeftMv = {0,0};
	MVpara tempAboveLeftMv = {0,0};
	MVpara tempAboveRightMv = {0,0};
	MVpara tempMergeMvd = {0,0};
	MVpara tempLeftMvd = {0,0};
	MVpara tempAboveMvd = {0,0};
	MVpara tempBelowLeftMvd = {0,0};
	MVpara tempAboveLeftMvd = {0,0};
	MVpara tempAboveRightMvd = {0,0};
	bool LeftforAboveEqual = false;
	bool LeftforAboveLeftEqual = false;
	bool LeftforBelowLeftEqual = false;
	bool AboveforAboveLeftEqual = false;
	bool AboveforAboveRightEqual = false;
	Int iCount = 0;
	Int32 xP, yP, nPSW, nPSH;

	UInt ruiPartIdxLT = 0;
	UInt ruiPartIdxRT = 0;
	UInt ruiPartIdxLB = 0;
	UInt ruiPartIdxRB = 0;
	UInt ruiPartIdxCenter = 0;

	UInt ruiLeftLCUIdx = 0;//左LCUindex
	UInt ruiAboveLeftLCUIdx = 0;//左上LCUindex
	UInt ruiAboveLCUIdx = 0;//上LCUindex
	UInt ruiAboveRightLCUIdx = 0;//右上LCUindex
	UInt ruiBelowLeftLCUIdx = 0;//左下LCUindex
	UInt ruiBelowRightLCUIdx = 0;

	UInt ruiLeftPURelIdx = 0;//左PUindex
	UInt ruiAboveLeftPURelIdx = 0;//左上PUindex
	UInt ruiAbovePURelIdx = 0; //上PUindex
	UInt ruiAboveRightPURelIdx = 0;//右上PUindex
	UInt ruiBelowLeftPURelIdx = 0;//左下PUindex
	UInt ruiBelowRightPURelIdx = 0;
	bool bCULeft = false;
	bool bCUAbove = false;
	bool bCUAboveRight = false;
	bool bCUBelowLeft = false;
	UInt uiLeftIdx = 255, uiAboveIdx = 255, uiAboveRightIdx = 255, uiBelowLeftIdx = 255, uiAboveLeftIdx = 255;

	//getInterMergeCandidates
	deriveLeftTopIdxGeneral ( h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PUPartIdx, ruiPartIdxLT );//最后1个值为返回值
	bool AboveLeftValid = getPUAboveLeft	( h, uiLCUIdx, ruiPartIdxLT, ruiAboveLeftLCUIdx, ruiAboveLeftPURelIdx );//最后2个值为返回值, 返回左上PU的LCUIdx和RelativeIdx
	deriveRightTopIdxGeneral( h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PUPartIdx, ruiPartIdxRT );//最后1个值为返回值
	bool AboveValid = getPUAbove( h, uiLCUIdx, ruiPartIdxRT, ruiAboveLCUIdx, ruiAbovePURelIdx );//最后2个值为返回值, 返回上PU的LCUIdx和RelativeIdx
	bool AboveRightValid = getPUAboveRight( h, uiLCUIdx, ruiPartIdxRT, ruiAboveRightLCUIdx, ruiAboveRightPURelIdx );//最后2个值为返回值, 返回右上PU的LCUIdx和RelativeIdx
	deriveLeftBottomIdxGeneral ( h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PUPartIdx, ruiPartIdxLB );//最后1个值为返回值
	bool LeftValid = getPULeft ( h, uiLCUIdx, ruiPartIdxLB, ruiLeftLCUIdx, ruiLeftPURelIdx ); //最后2个值为返回值, 返回左PU的LCUIdx和RelativeIdx
	bool BelowLeftValid = getPUBelowLeft	( h, uiLCUIdx, ruiPartIdxLB, ruiBelowLeftLCUIdx, ruiBelowLeftPURelIdx );//最后2个值为返回值, 返回左下PU的LCUIdx和RelativeIdx
	deriveRightBottomIdxGeneral ( h, uiLCUIdx, CURelIdx, uiDepth, ePartSize, PUPartIdx, ruiPartIdxRB );//最后1个值为返回值
	bool BelowRightValid = getPUBelowRight ( h, uiLCUIdx, ruiPartIdxRB, ruiBelowRightLCUIdx, ruiBelowRightPURelIdx );//最后2个值为返回值, 返回右下PU的LCUIdx和RelativeIdx
	deriveCenterIdxGeneral( h, PURelIdx, uiDepth, ePartSize, ruiPartIdxCenter);//最后1个值为返回值

	nPSW = pCache->subPUWidth;
	nPSH = pCache->subPUHeight;
	xP = pCache->subPUX + h->uiCUX;
	yP = pCache->subPUY + h->uiCUY;

	//Left
	if ( LeftValid && isDiffMER(xP -1, yP+nPSH-1, xP, yP))
	{
		bCULeft = true;
		FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiLeftLCUIdx, ruiLeftPURelIdx, 0, ePartSize, VALID_L, PUPartIdx, tempLeftMv, tempLeftMvd );
		if (!(PUPartIdx == 1 && (ePartSize == SIZE_Nx2N || ePartSize == SIZE_nLx2N || ePartSize == SIZE_nRx2N)))
		{
			FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiLeftLCUIdx, ruiLeftPURelIdx, 0, ePartSize, VALID_L, PUPartIdx, MergeCandidatesMv[iCount], MergeCandidatesMvd[iCount] );
			if ( iCount == MRG_MAX_NUM_CANDS )
			{
				return;
			}
			uiLeftIdx = iCount;
			iCount++;
		}
	}
	//Above
	if (AboveValid && isDiffMER(xP+nPSW-1, yP-1, xP, yP))
	{
		bCUAbove = true;
		FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiAboveLCUIdx, ruiAbovePURelIdx, 0, ePartSize, VALID_T, PUPartIdx, tempAboveMv, tempAboveMvd );
		if(bCULeft)
		{
			LeftforAboveEqual = hasEqualMotion( &tempLeftMv, &tempAboveMv );
		}
		else
		{
			LeftforAboveEqual = false;
		}
		if ((!(PUPartIdx  == 1 && (ePartSize == SIZE_2NxN || ePartSize == SIZE_2NxnU || ePartSize == SIZE_2NxnD)))
			&& (!bCULeft || !LeftforAboveEqual))
		{
			FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiAboveLCUIdx, ruiAbovePURelIdx, 0, ePartSize, VALID_T, PUPartIdx, MergeCandidatesMv[iCount], MergeCandidatesMvd[iCount] );
			if ( iCount == MRG_MAX_NUM_CANDS )
			{
				return;
			}
			uiAboveIdx = iCount;
			iCount++;
		}
	}
	//AboveRight
	if ( AboveRightValid && isDiffMER(xP+nPSW, yP-1, xP, yP))
	{
		bCUAboveRight = true;
		FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiAboveRightLCUIdx, ruiAboveRightPURelIdx, 0, ePartSize, VALID_RT, PUPartIdx, tempAboveRightMv, tempAboveRightMvd );
		if(bCUAbove)
		{
			AboveforAboveRightEqual = hasEqualMotion( &tempAboveMv, &tempAboveRightMv );
		}
		else
		{
			AboveforAboveRightEqual = false;
		}
		if ((!bCUAbove || !AboveforAboveRightEqual))
		{
			FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiAboveRightLCUIdx, ruiAboveRightPURelIdx, 0, ePartSize, VALID_RT, PUPartIdx, MergeCandidatesMv[iCount], MergeCandidatesMvd[iCount] );
			if ( iCount == MRG_MAX_NUM_CANDS )	
			{
				return;
			}
			uiAboveRightIdx = iCount;
			iCount++;
		}
	}
	//BelowLeft
	if ( BelowLeftValid && isDiffMER(xP-1, yP+nPSH, xP, yP))
	{
		bCUBelowLeft = true;
		FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiBelowLeftLCUIdx, ruiBelowLeftPURelIdx, 0, ePartSize, VALID_LB, PUPartIdx, tempBelowLeftMv, tempBelowLeftMvd );
		if(bCULeft)
		{
			LeftforBelowLeftEqual = hasEqualMotion( &tempLeftMv, &tempBelowLeftMv) ;
		}
		else
		{
			LeftforBelowLeftEqual = false;
		}
		if( (!bCULeft) || !LeftforBelowLeftEqual)
		{
			FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiBelowLeftLCUIdx, ruiBelowLeftPURelIdx, 0, ePartSize, VALID_LB, PUPartIdx, MergeCandidatesMv[iCount], MergeCandidatesMvd[iCount] );
			if ( iCount == MRG_MAX_NUM_CANDS )
			{
				return;
			}
			uiBelowLeftIdx = iCount;
			iCount++;
		}
	}
	//AboveLeft
	if( iCount < 4 )
	{
		if (AboveLeftValid && isDiffMER(xP-1, yP-1, xP, yP))
		{
			FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiAboveLeftLCUIdx, ruiAboveLeftPURelIdx, 0, ePartSize, VALID_LT, PUPartIdx, tempAboveLeftMv, tempAboveLeftMvd );
			if(bCULeft)
			{
				LeftforAboveLeftEqual = hasEqualMotion( &tempLeftMv, &tempAboveLeftMv);
			}
			else
			{
				LeftforAboveLeftEqual = false;
			}
			if(bCUAbove)
			{
				AboveforAboveLeftEqual = hasEqualMotion( &tempAboveMv, &tempAboveLeftMv );
			}
			else
			{
				AboveforAboveLeftEqual = false;
			}
			
			if((!bCULeft || !LeftforAboveLeftEqual)	&& ((!bCUAbove || !AboveforAboveLeftEqual)))
			{
				FindMv( h, MODE_MERGE, 0, uiDepth, uiLCUIdx, ruiAboveLeftLCUIdx, ruiAboveLeftPURelIdx, 0, ePartSize, VALID_LT, PUPartIdx, MergeCandidatesMv[iCount], MergeCandidatesMvd[iCount] );
				if ( iCount == MRG_MAX_NUM_CANDS )
				{
					return;
				}
				iCount++;
			}
		}
	}

	//TMVPFlag
	if (BelowRightValid)
	{
		FindMv( h, MODE_MERGE,-1, uiDepth, uiLCUIdx, ruiBelowRightLCUIdx, ruiBelowRightPURelIdx, 0, ePartSize, VALID_T, 0, MergeCandidatesMv[iCount], MergeCandidatesMvd[iCount] );//TMVP
		if ( iCount == MRG_MAX_NUM_CANDS )
		{
			return;
		}
		iCount++;	
	}
	else
	{
		FindMv( h, MODE_MERGE, -1, uiDepth, uiLCUIdx, uiLCUIdx, ruiPartIdxCenter, 0, ePartSize, VALID_T, PUPartIdx, MergeCandidatesMv[iCount], MergeCandidatesMvd[iCount] );
		if ( iCount == MRG_MAX_NUM_CANDS )
		{
			return;
		}
		iCount++;	
	}

	if(iCount > MRG_MAX_NUM_CANDS)
		iCount = MRG_MAX_NUM_CANDS;
	numValidMergeCand = iCount;
}

void getPartPosition( MTC265_t *h, UInt uiLCUIdx, UInt uiAbsCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH )
{
	UInt uiNumPartInLCUWidth, uiNumLCUInPicWidth;
	UInt col, row;
	UInt width;

	uiNumPartInLCUWidth = 1 << h->ucMaxCUDepth;
	uiNumLCUInPicWidth = ( h->usWidth + h->PAD_YSIZEw + h->ucMaxCUWidth - 1 ) / h->ucMaxCUWidth;

	col = (uiLCUIdx % uiNumLCUInPicWidth) * h->ucMaxCUWidth + (g_auiZscanToRaster[uiAbsCUIdx] % uiNumPartInLCUWidth) * h->ucMaxCUWidth;
	row = (uiLCUIdx / uiNumLCUInPicWidth) * h->ucMaxCUWidth + (g_auiZscanToRaster[uiAbsCUIdx] / uiNumPartInLCUWidth) * h->ucMaxCUWidth;
	width = h->ucMaxCUWidth >> uiDepth;

	switch ( eCUMode )
	{
	case SIZE_2NxN:
		nPSW = width;      
		nPSH = width >> 1; 
		xP   = col;
		yP   = (partIdx ==0)? row: row + nPSH;
		break;
	case SIZE_Nx2N:
		nPSW = width >> 1; 
		nPSH = width;      
		xP   = (partIdx ==0)? col: col + nPSW;
		yP   = row;
		break;
	case SIZE_NxN:
		nPSW = width >> 1; 
		nPSH = width >> 1; 
		xP   = col + (partIdx&0x1)*nPSW;
		yP   = row + (partIdx>>1)*nPSH;
		break;
	default:
		nPSW = width;      
		nPSH = width;      
		xP   = col ;
		yP   = row ;
		break;
	}
}

bool isDiffMER(Int xN, Int yN, Int xP, Int yP)
{
	UInt plevel = 2;
	if ((xN>>plevel)!= (xP>>plevel))
		return true;

	if ((yN>>plevel)!= (yP>>plevel))
		return true;

	return false;
}

bool hasEqualMotion( MVpara *mv1, MVpara *mv2 )
{
	if(mv1->m_x != mv2->m_x)
		return false;
	if(mv1->m_y != mv2->m_y)
		return false;
	return true;
}

void setInterDepth(MTC265_t *h, UInt8 uiDepth )
{
	MTC265_Cache* pCache = &h->cache;
	UInt32 nCUSize = h->ucMaxCUWidth>>uiDepth;
	Int32* CURelIdx = pCache->RelativeIdx;
	Int32 RelativeIdx = CURelIdx[uiDepth];
	Int32 copyCURelIdx = 0;
	Int32 copyToCUOffset = 0;
	Int32 copyToFrameOffset = 0;
	UInt32 uiLCUIdx = h->uiCUX/h->ucMaxCUWidth + (((h->usWidth+h->PAD_YSIZEw)/h->ucMaxCUWidth) * (h->uiCUY/h->ucMaxCUWidth));
	for (UInt i = 0; i < nCUSize/4;i++)
	{
		for (UInt j = 0; j < nCUSize/4;j++)
		{
			copyCURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[RelativeIdx] + i * (h->ucMaxCUWidth/4) + j];
			copyToCUOffset = copyCURelIdx;
			copyToFrameOffset = uiLCUIdx * pCache->NumPartition + copyCURelIdx;
			pCache->BestpuhInterDepth[copyToCUOffset] = uiDepth;
			h->FrameBestDepth[copyToFrameOffset] = uiDepth;
		}
	}
}

void CheckCUBestMode(MTC265_t *h, UInt8 uiDepth, double SAD_TotalCU, double& SAD_BestCU)//其中SAD_TotalCU为uiDepth+1的SAD之和,SAD_BestCU为当前uiDepth的SAD
{
	MTC265_Cache* pCache = &h->cache;
	UInt32 uiCUX = pCache->cuX;
	UInt32 uiCUY = pCache->cuY;
	Int32* PartIdx = pCache->PartIdx;
	UInt8 nextDepth = uiDepth + 1;
	UInt32 uiLCUIdx = h->uiCUX/h->ucMaxCUWidth + (((h->usWidth+h->PAD_YSIZEw)/h->ucMaxCUWidth) * (h->uiCUY/h->ucMaxCUWidth));
	UInt32 subCUX = 0;
	UInt32 subCUY = 0;
	UInt32 subTranCUX = 0;
	UInt32 subTranCUY = 0;
	Int32 nCUSize = h->ucMaxCUWidth >> uiDepth;
	Int* RelIdx = pCache->RelativeIdx;
	UInt32 RelativeIdx = 0;
	UInt32 TranUsedRelIdx = 0;
	Int32 CURelIdx = RelIdx[uiDepth];

	Int32 copyCURelIdx = 0;
	Int32 copyFromCUOffset = 0;
	Int32 copyToCUOffset = 0;
	Int32 copyToUpCUOffset = 0;
	Int32 copyToFrameOffset = 0;

	if(SAD_TotalCU <SAD_BestCU )
	{
		SAD_BestCU = SAD_TotalCU;
		for(int partindex=0; partindex<4; partindex++)
		{
			subCUX=0;
			subCUY=0;

			PartIdx[nextDepth]= partindex;
			RelativeIdx = 0;
			for(Int32 num=0;num<=Int32(nextDepth);num++)
			{
				int ipow  = int(pow(double(4),num)); 
				RelativeIdx +=(PartIdx[num])*(pCache->NumPartition)/ipow;
			}
			RelIdx[nextDepth] = RelativeIdx;//RelativeIdx代表当前的CU，0-63的位置
			xEncGetCUPosition( h, nCUSize>>1, nextDepth, uiCUX, uiCUY, &subCUX, &subCUY );//找CU位置，当前CU的左上角坐标为CU_Cache->cuX和CU_Cache->cuY	
			subCUX -= h->uiCUX;//上面的函数中多加了h->uiCUX和h->uiCUY
			subCUY -= h->uiCUY;			

			//如果uiDepth+1的四个CU未做过变换量化
			if (pCache->TranUsedFlag[RelativeIdx] == 0)
			{
				for (int i = 0; i < nCUSize/8;i++)
				{
					for (int j = 0; j < nCUSize/8;j++)
					{
						copyCURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[RelativeIdx] + i * (h->ucMaxCUWidth/4) + j];
						copyToCUOffset = copyCURelIdx;
						copyToFrameOffset = uiLCUIdx * pCache->NumPartition + copyCURelIdx;
						pCache->BestpuhInterDepth[copyToCUOffset] = uiDepth+1;
						h->FrameBestDepth[copyToFrameOffset] = uiDepth + 1;
					}
				}

				//将组合好的2NxN或Nx2N的PU按照拆分的方式进行变换
				if ((pCache->TemppuhInterPartSize[nextDepth*pCache->NumPartition + RelativeIdx] == SIZE_2NxN)||(pCache->TemppuhInterPartSize[nextDepth*pCache->NumPartition + RelativeIdx] == SIZE_Nx2N))
				{
					for(int i = 0; i<4 ; i++)//针对将非方形PU拆分成2部分
					{
						subTranCUX = subCUX;
						subTranCUY = subCUY;
						if(i==0)//得到4个变换块左上角点的坐标
						{
							subTranCUX += (nCUSize>>2)*0 ;
							subTranCUY += (nCUSize>>2)*0 ;
							pCache->RelativeIdx[nextDepth+1] = RelativeIdx + i*(int)pow(double(4),2-nextDepth); 
						}
						else if(i==1)
						{
							subTranCUX += (nCUSize>>2)*1 ;
							subTranCUY += (nCUSize>>2)*0 ;
							pCache->RelativeIdx[nextDepth+1] = RelativeIdx + i*(int)pow(double(4),2-nextDepth); 
						}
						else if(i==2)
						{
							subTranCUX += (nCUSize>>2)*0 ;
							subTranCUY += (nCUSize>>2)*1 ;
							pCache->RelativeIdx[nextDepth+1] = RelativeIdx + i*(int)pow(double(4),2-nextDepth); 
						}
						else if(i==3)
						{
							subTranCUX += (nCUSize>>2)*1 ;
							subTranCUY += (nCUSize>>2)*1 ;
							pCache->RelativeIdx[nextDepth+1] = RelativeIdx + i*(int)pow(double(4),2-nextDepth); 
						}
						if (nextDepth == 2)
						{
							//只能做亮度的拆分，色度需要组合起来，因此变换量化只能对亮度进行
							estInterPredQTLuma(h, nCUSize>>2, nextDepth, pCache->RelativeIdx[nextDepth+1], subTranCUX, subTranCUY );
						}
						else
						{
							estInterPredQT(h, nCUSize>>2, nextDepth, pCache->RelativeIdx[nextDepth+1], subTranCUX, subTranCUY );
						}				
					}
					if (nextDepth == 2)
					{
						//nextDepth层色度的变换、量化
						estInterPredQTChroma(h, nCUSize>>1, nextDepth, RelativeIdx, subCUX, subCUY );
					}
				}
				else
				{
					estInterPredQT(h, nCUSize>>1, nextDepth, RelativeIdx, subCUX, subCUY );
				}
				//说明已经做过变换
				for (int i = 0; i < nCUSize/8;i++)
				{
					for (int j = 0; j < nCUSize/8;j++)
					{
						TranUsedRelIdx = g_auiRasterToZscan[g_auiZscanToRaster[RelativeIdx] + i * (h->ucMaxCUWidth/4) + j];
						pCache->TranUsedFlag[TranUsedRelIdx] = 1;
					}
				}
				//将uiDepth+1层的重建帧、变换量化后的系数复制到上一层
				copyRecCoefCbfUp(h, nCUSize, uiDepth, subCUX, subCUY);
			}
			else
			{
				//将uiDepth+1层的重建帧、变换量化后的系数复制到上一层
				copyRecCoefCbfUp(h, nCUSize, uiDepth, subCUX, subCUY);
			}
		}

		//将最优数据保存在Best数组中
		for (int i = 0; i < nCUSize/4;i++)
		{
			for (int j = 0; j < nCUSize/4;j++)
			{
				copyCURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[CURelIdx] + i * (h->ucMaxCUWidth/4) + j];
				copyFromCUOffset = (uiDepth+1)*pCache->NumPartition + copyCURelIdx;
				copyToUpCUOffset = uiDepth*pCache->NumPartition + copyCURelIdx;
				copyToCUOffset = copyCURelIdx;
				copyToFrameOffset = uiLCUIdx * pCache->NumPartition + copyCURelIdx;
				Int32 iTest = pCache->TempbCbfY[copyFromCUOffset];

				pCache->BestpuhMv[copyToCUOffset].m_x = pCache->TemppuhMv[copyFromCUOffset].m_x;
				pCache->BestpuhMv[copyToCUOffset].m_y = pCache->TemppuhMv[copyFromCUOffset].m_y;
				pCache->BestpuhMvd[copyToCUOffset].m_x = pCache->TemppuhMvd[copyFromCUOffset].m_x;
				pCache->BestpuhMvd[copyToCUOffset].m_y = pCache->TemppuhMvd[copyFromCUOffset].m_y;
				pCache->BestpuhMvpIdx[copyToCUOffset] = pCache->TemppuhMvpIdx[copyFromCUOffset];
				pCache->BestpuhInterSAD[copyToCUOffset] = pCache->TemppuhInterSAD[copyFromCUOffset];
				pCache->BestpuhInterPartSize[copyToCUOffset] = pCache->TemppuhInterPartSize[copyFromCUOffset];
				pCache->BestInterCbfY[copyToCUOffset] = pCache->TempbCbfY[copyFromCUOffset];
				pCache->BestInterCbfU[copyToCUOffset] = pCache->TempbCbfU[copyFromCUOffset];
				pCache->BestInterCbfV[copyToCUOffset] = pCache->TempbCbfV[copyFromCUOffset];
				pCache->BestpuhSkipFlag[copyToCUOffset] = pCache->TemppuhSkipFlag[copyFromCUOffset];
				pCache->BestpuhMergeFlag[copyToCUOffset] = pCache->TemppuhMergeFlag[copyFromCUOffset];
				pCache->BestpuhMergeIdx[copyToCUOffset] = pCache->TemppuhMergeIdx[copyFromCUOffset];
				h->FrameBestMv[copyToFrameOffset].m_x = pCache->TemppuhMv[copyFromCUOffset].m_x;
				h->FrameBestMv[copyToFrameOffset].m_y = pCache->TemppuhMv[copyFromCUOffset].m_y;
				h->FrameBestMvd[copyToFrameOffset].m_x = pCache->TemppuhMvd[copyFromCUOffset].m_x;
				h->FrameBestMvd[copyToFrameOffset].m_y = pCache->TemppuhMvd[copyFromCUOffset].m_y;
				h->FrameBestPartSize[copyToFrameOffset] = pCache->TemppuhInterPartSize[copyFromCUOffset];
				h->FrameBestMergeFlag[copyToFrameOffset] = pCache->TemppuhMergeFlag[copyFromCUOffset];
				h->FrameBestMergeIdx[copyToFrameOffset] = pCache->TemppuhMergeIdx[copyFromCUOffset];
				h->FrameBestSkipFlag[copyToFrameOffset] = pCache->TemppuhSkipFlag[copyFromCUOffset];
				
				//copy上下层
				pCache->TemppuhMv[copyToUpCUOffset].m_x = pCache->TemppuhMv[copyFromCUOffset].m_x;
				pCache->TemppuhMv[copyToUpCUOffset].m_y = pCache->TemppuhMv[copyFromCUOffset].m_y;
				pCache->TemppuhMvd[copyToUpCUOffset].m_x = pCache->TemppuhMvd[copyFromCUOffset].m_x;
				pCache->TemppuhMvd[copyToUpCUOffset].m_y = pCache->TemppuhMvd[copyFromCUOffset].m_y;
				pCache->TemppuhSkipFlag[copyToUpCUOffset] = pCache->TemppuhSkipFlag[copyFromCUOffset];
				pCache->TemppuhMergeFlag[copyToUpCUOffset] = pCache->TemppuhMergeFlag[copyFromCUOffset];
				pCache->TemppuhMergeIdx[copyToUpCUOffset] = pCache->TemppuhMergeIdx[copyFromCUOffset];
				pCache->TemppuhMvpIdx[copyToUpCUOffset] = pCache->TemppuhMvpIdx[copyFromCUOffset];
				pCache->TemppuhInterSAD[copyToUpCUOffset] = pCache->TemppuhInterSAD[copyFromCUOffset];
				pCache->TemppuhInterPartSize[copyToUpCUOffset] = pCache->TemppuhInterPartSize[copyFromCUOffset];
				pCache->TempbCbfY[copyToUpCUOffset] = pCache->TempbCbfY[copyFromCUOffset];
				pCache->TempbCbfU[copyToUpCUOffset] = pCache->TempbCbfU[copyFromCUOffset];
				pCache->TempbCbfV[copyToUpCUOffset] = pCache->TempbCbfV[copyFromCUOffset];
			}
		}
	}
	else
	{
		//将组合好的2NxN或Nx2N的PU按照拆分的方式进行变换
		if ((pCache->TemppuhInterPartSize[uiDepth*pCache->NumPartition + CURelIdx] == SIZE_2NxN)||(pCache->TemppuhInterPartSize[uiDepth*pCache->NumPartition + CURelIdx] == SIZE_Nx2N))
		{
			for(int i = 0; i<4 ; i++)
			{
				subTranCUX = uiCUX;
				subTranCUY = uiCUY;
				if(i==0)//得到4个变换块左上角点的坐标
				{
					subTranCUX += (nCUSize>>1)*0 ;
					subTranCUY += (nCUSize>>1)*0 ;
					pCache->RelativeIdx[nextDepth] = CURelIdx + i*(int)pow(double(4),2-uiDepth); 
				}
				else if(i==1)
				{
					subTranCUX += (nCUSize>>1)*1 ;
					subTranCUY += (nCUSize>>1)*0 ;
					pCache->RelativeIdx[nextDepth] = CURelIdx + i*(int)pow(double(4),2-uiDepth); 
				}
				else if(i==2)
				{
					subTranCUX += (nCUSize>>1)*0 ;
					subTranCUY += (nCUSize>>1)*1 ;
					pCache->RelativeIdx[nextDepth] = CURelIdx + i*(int)pow(double(4),2-uiDepth); 
				}
				else if(i==3)
				{
					subTranCUX += (nCUSize>>1)*1 ;
					subTranCUY += (nCUSize>>1)*1 ;
					pCache->RelativeIdx[nextDepth] = CURelIdx + i*(int)pow(double(4),2-uiDepth); 
				}
				estInterPredQT(h, nCUSize>>1, uiDepth, pCache->RelativeIdx[nextDepth], subTranCUX, subTranCUY );
			}
		}
		else
		{
			estInterPredQT(h, nCUSize, uiDepth, CURelIdx, uiCUX, uiCUY );
		}

		for (int i = 0; i < nCUSize/4;i++)
		{
			for (int j = 0; j < nCUSize/4;j++)
			{
				copyCURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[CURelIdx] + i * (h->ucMaxCUWidth/4) + j];
				copyFromCUOffset = uiDepth*pCache->NumPartition + copyCURelIdx;
				copyToCUOffset = copyCURelIdx;
				copyToFrameOffset = uiLCUIdx * pCache->NumPartition + copyCURelIdx;
				pCache->TranUsedFlag[copyCURelIdx] = 1;//已经做过变换

				pCache->BestpuhMv[copyToCUOffset].m_x = pCache->TemppuhMv[copyFromCUOffset].m_x;
				pCache->BestpuhMv[copyToCUOffset].m_y = pCache->TemppuhMv[copyFromCUOffset].m_y;
				pCache->BestpuhMvd[copyToCUOffset].m_x = pCache->TemppuhMvd[copyFromCUOffset].m_x;
				pCache->BestpuhMvd[copyToCUOffset].m_y = pCache->TemppuhMvd[copyFromCUOffset].m_y;
				pCache->BestpuhMvpIdx[copyToCUOffset] = pCache->TemppuhMvpIdx[copyFromCUOffset];
				pCache->BestpuhSkipFlag[copyToCUOffset] = pCache->TemppuhSkipFlag[copyFromCUOffset];
				pCache->BestpuhMergeFlag[copyToCUOffset] = pCache->TemppuhMergeFlag[copyFromCUOffset];
				pCache->BestpuhMergeIdx[copyToCUOffset] = pCache->TemppuhMergeIdx[copyFromCUOffset];
				pCache->BestpuhInterSAD[copyToCUOffset] = pCache->TemppuhInterSAD[copyFromCUOffset];
				pCache->BestpuhInterPartSize[copyToCUOffset] = pCache->TemppuhInterPartSize[copyFromCUOffset];
				pCache->BestpuhInterDepth[copyToCUOffset] = uiDepth;
				pCache->BestInterCbfY[copyToCUOffset] = pCache->TempbCbfY[copyFromCUOffset];
				pCache->BestInterCbfU[copyToCUOffset] = pCache->TempbCbfU[copyFromCUOffset];
				pCache->BestInterCbfV[copyToCUOffset] = pCache->TempbCbfV[copyFromCUOffset];
				h->FrameBestMv[copyToFrameOffset].m_x = pCache->TemppuhMv[copyFromCUOffset].m_x;
				h->FrameBestMv[copyToFrameOffset].m_y = pCache->TemppuhMv[copyFromCUOffset].m_y;
				h->FrameBestMvd[copyToFrameOffset].m_x = pCache->TemppuhMvd[copyFromCUOffset].m_x;
				h->FrameBestMvd[copyToFrameOffset].m_y = pCache->TemppuhMvd[copyFromCUOffset].m_y;
				h->FrameBestPartSize[copyToFrameOffset] = pCache->TemppuhInterPartSize[copyFromCUOffset];
				h->FrameBestMergeFlag[copyToFrameOffset] = pCache->TemppuhMergeFlag[copyFromCUOffset];
				h->FrameBestMergeIdx[copyToFrameOffset] = pCache->TemppuhMergeIdx[copyFromCUOffset];
				h->FrameBestSkipFlag[copyToFrameOffset] = pCache->TemppuhSkipFlag[copyFromCUOffset];
				h->FrameBestDepth[copyToFrameOffset] = uiDepth;
			}
		}
	}
}

void copyRecCoefCbfUp(MTC265_t *h, Int32 nCUSize, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY)
{
	MTC265_Cache* pCache = &h->cache;
	Pxl* pucInterRecFromY = pCache->pucTempRecY[ uiDepth + 1 ] + (MAX_CU_SIZE)*(uiCUY) + uiCUX;
	Pxl* pucInterRecFromU = pCache->pucTempRecU[ uiDepth + 1 ] + (MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2;
	Pxl* pucInterRecFromV = pCache->pucTempRecV[ uiDepth + 1 ] + (MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2;
	Int16* pucInterCoefFromY = pCache->psTempCoefY[ uiDepth + 1 ] + (MAX_CU_SIZE)*(uiCUY) + uiCUX;
	Int16* pucInterCoefFromU = pCache->psTempCoefU[ uiDepth + 1 ] + (MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2;
	Int16* pucInterCoefFromV = pCache->psTempCoefV[ uiDepth + 1 ] + (MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2;

	Pxl* pucInterRecToY = pCache->pucTempRecY[ uiDepth ] + (MAX_CU_SIZE)*(uiCUY) + uiCUX;
	Pxl* pucInterRecToU = pCache->pucTempRecU[ uiDepth ] + (MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2;
	Pxl* pucInterRecToV = pCache->pucTempRecV[ uiDepth ] + (MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2;
	Int16* pucInterCoefToY = pCache->psTempCoefY[ uiDepth ] + (MAX_CU_SIZE)*(uiCUY) + uiCUX;
	Int16* pucInterCoefToU = pCache->psTempCoefU[ uiDepth ] + (MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2;
	Int16* pucInterCoefToV = pCache->psTempCoefV[ uiDepth ] + (MAX_CU_SIZE/2)*(uiCUY/2) + uiCUX/2;

	Int32 copyCURelIdx = 0;
	Int32 copyFromCUOffset = 0;
	Int32 copyToCUOffset = 0;
	for (int i =0;i<nCUSize ;i++)
	{
		for (int j =0;j<nCUSize ;j++)
		{
			pucInterRecToY[ j*MAX_CU_SIZE + i] = pucInterRecFromY[j*MAX_CU_SIZE +i];
			pucInterCoefToY[ j*MAX_CU_SIZE + i] = pucInterCoefFromY[j*MAX_CU_SIZE +i];
		}
	}
	for (int i =0;i<nCUSize/2 ;i++)
	{
		for (int j =0;j<nCUSize/2 ;j++)
		{
			pucInterCoefToU[ j*MAX_CU_SIZE/2 + i] = pucInterCoefFromU[j*MAX_CU_SIZE/2 +i];
			pucInterCoefToV[ j*MAX_CU_SIZE/2 + i] = pucInterCoefFromV[j*MAX_CU_SIZE/2 +i];
			pucInterRecToU[ j*MAX_CU_SIZE/2 + i] = pucInterRecFromU[j*MAX_CU_SIZE/2 +i];
			pucInterRecToV[ j*MAX_CU_SIZE/2 + i] = pucInterRecFromV[j*MAX_CU_SIZE/2 +i];
		}
	}

	//将PU内的全部StartChange变量均赋值为同样的值
	for (int i = 0; i < nCUSize/4;i++)
	{
		for (int j = 0; j < nCUSize/4;j++)
		{
			copyCURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[pCache->RelativeIdx[uiDepth]] + i * (h->ucMaxCUWidth/4) + j];
			copyFromCUOffset = (uiDepth+1)*pCache->NumPartition + copyCURelIdx;
			copyToCUOffset = uiDepth*pCache->NumPartition + copyCURelIdx;

			pCache->TempbCbfY[copyToCUOffset] = pCache->TempbCbfY[copyFromCUOffset];
			pCache->TempbCbfU[copyToCUOffset] = pCache->TempbCbfU[copyFromCUOffset];
			pCache->TempbCbfV[copyToCUOffset] = pCache->TempbCbfV[copyFromCUOffset];
		}
	}
}

void CheckPUBestMode(MTC265_t *h, UInt8 uiDepth, Int32* PURelIdxCompare_SIZE_2Nx2N, Int32* PURelIdxCompare_SIZE_2NxN, Int32* PURelIdxCompare_SIZE_Nx2N, double SAD_2Nx2N, double SAD_2NxN, double SAD_Nx2N, double& SAD_BestPU)
{
	MTC265_Cache* pCache = &h->cache;
	MTC265_PartSize BestPartSize;
	UInt32 uiLCUIdx = h->uiCUX/h->ucMaxCUWidth + (((h->usWidth+h->PAD_YSIZEw)/h->ucMaxCUWidth) * (h->uiCUY/h->ucMaxCUWidth));
	Int32 TempInterToCur = 0;
	Int32 PURelIdx = 0;
	Int32 copyPURelIdx = 0;
	Int32 copyPUOffset = 0;
	Int32 nCUSize = h->ucMaxCUWidth >> uiDepth;
	UInt32 subPUWidth = 0;
	UInt32 subPUHeight = 0;
	Interpara* StartInter = 0;
	Interpara* StartTempInter = h->TempInter;
	if( SAD_2Nx2N <= SAD_2NxN )
	{
		if( SAD_2Nx2N <= SAD_Nx2N )
		{
			BestPartSize = SIZE_2Nx2N;
			SAD_BestPU = SAD_2Nx2N;
		}	
		else
		{		
			BestPartSize = SIZE_Nx2N;
			SAD_BestPU = SAD_Nx2N;
		}
	}
	else
	{
		if( SAD_Nx2N < SAD_2NxN )
		{
			BestPartSize = SIZE_Nx2N;
			SAD_BestPU = SAD_Nx2N;
		}
		else
		{
			BestPartSize = SIZE_2NxN;
			SAD_BestPU = SAD_2NxN;
		}
	}

	if (BestPartSize == SIZE_2Nx2N)
	{
		PURelIdx = PURelIdxCompare_SIZE_2Nx2N[0];
		TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*pCache->NumPartition*h->PUnumber + uiDepth*pCache->NumPartition*h->PUnumber + PURelIdx*h->PUnumber + (Int32)BestPartSize;
		//以下的变量由四元组(uiLCUIdx, uiDepth, PURelIdx, BestPartSize)决定
		StartInter = StartTempInter + TempInterToCur;
		subPUWidth = h->ucMaxCUWidth >> uiDepth; 
		subPUHeight = h->ucMaxCUWidth >> uiDepth;

		for (UInt i = 0; i < subPUHeight/4;i++)
		{
			for (UInt j = 0; j < subPUWidth/4;j++)
			{
				copyPURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx] + i * (h->ucMaxCUWidth/4) + j];
				copyPUOffset = uiDepth*pCache->NumPartition + copyPURelIdx;

				pCache->TemppuhMv[copyPUOffset].m_x = StartInter->iMvBestY.m_x;
				pCache->TemppuhMv[copyPUOffset].m_y = StartInter->iMvBestY.m_y;
				pCache->TemppuhMvd[copyPUOffset].m_x = StartInter->iMvdBestY.m_x;
				pCache->TemppuhMvd[copyPUOffset].m_y = StartInter->iMvdBestY.m_y;
				pCache->TemppuhMvpIdx[copyPUOffset] = StartInter->iMvp_index;
				pCache->TemppuhSkipFlag[copyPUOffset] = StartInter->SkipFlag;
				pCache->TemppuhMergeFlag[copyPUOffset] = StartInter->MergeFlag;
				pCache->TemppuhMergeIdx[copyPUOffset] = StartInter->MergeIdx;
				pCache->TemppuhInterSAD[copyPUOffset] = StartInter->iSadBest;
				pCache->TemppuhInterPartSize[copyPUOffset] = (UInt8)SIZE_2Nx2N;
			}
		}
		MotionCompensation( h, StartInter->iMvBestY, StartInter->iSrchCenBest.m_x, StartInter->iSrchCenBest.m_y, uiDepth, BestPartSize, 0, subPUWidth, subPUHeight);
	}
	else if ((BestPartSize == SIZE_2NxN))
	{
		subPUWidth = h->ucMaxCUWidth >> uiDepth; 
		subPUHeight = h->ucMaxCUWidth >> (uiDepth+1);

		for (int PUpartindex = 0; PUpartindex < 2;PUpartindex++)
		{
			PURelIdx = PURelIdxCompare_SIZE_2NxN[PUpartindex];
			TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*pCache->NumPartition*h->PUnumber + uiDepth*pCache->NumPartition*h->PUnumber + PURelIdx*h->PUnumber + (Int32)BestPartSize;
			//以下的变量由四元组(uiLCUIdx, uiDepth, PURelIdx, BestPartSize)决定
			StartInter = StartTempInter + TempInterToCur;
			for (UInt i = 0; i < subPUHeight/4;i++)
			{
				for (UInt j = 0; j < subPUWidth/4;j++)
				{
					copyPURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx] + i * (h->ucMaxCUWidth/4) + j];
					copyPUOffset = uiDepth*pCache->NumPartition + copyPURelIdx;

					pCache->TemppuhMv[copyPUOffset].m_x = StartInter->iMvBestY.m_x;
					pCache->TemppuhMv[copyPUOffset].m_y = StartInter->iMvBestY.m_y;
					pCache->TemppuhMvd[copyPUOffset].m_x = StartInter->iMvdBestY.m_x;
					pCache->TemppuhMvd[copyPUOffset].m_y = StartInter->iMvdBestY.m_y;
					pCache->TemppuhMvpIdx[copyPUOffset] = StartInter->iMvp_index;
					pCache->TemppuhSkipFlag[copyPUOffset] = StartInter->SkipFlag;
					pCache->TemppuhMergeFlag[copyPUOffset] = StartInter->MergeFlag;
					pCache->TemppuhMergeIdx[copyPUOffset] = StartInter->MergeIdx;
					pCache->TemppuhInterSAD[copyPUOffset] = StartInter->iSadBest;
					pCache->TemppuhInterPartSize[copyPUOffset] = (UInt8)SIZE_2NxN;
				}
			}
			MotionCompensation( h, StartInter->iMvBestY, StartInter->iSrchCenBest.m_x, StartInter->iSrchCenBest.m_y, uiDepth, BestPartSize, PUpartindex, subPUWidth, subPUHeight);
		}
	}
	else if ((BestPartSize == SIZE_Nx2N))
	{
		subPUWidth = h->ucMaxCUWidth >> (uiDepth+1); 
		subPUHeight = h->ucMaxCUWidth >> uiDepth;

		for (int PUpartindex = 0; PUpartindex < 2;PUpartindex++)
		{
			PURelIdx = PURelIdxCompare_SIZE_Nx2N[PUpartindex];
			TempInterToCur = uiLCUIdx*h->ucMaxCUDepth*pCache->NumPartition*h->PUnumber + uiDepth*pCache->NumPartition*h->PUnumber + PURelIdx*h->PUnumber + (Int32)BestPartSize;
			//以下的变量由四元组(uiLCUIdx, uiDepth, PURelIdx, BestPartSize)决定
			StartInter = StartTempInter + TempInterToCur;
			for (UInt i = 0; i < subPUHeight/4;i++)
			{
				for (UInt j = 0; j < subPUWidth/4;j++)
				{
					copyPURelIdx = g_auiRasterToZscan[g_auiZscanToRaster[PURelIdx] + i * (h->ucMaxCUWidth/4) + j];
					copyPUOffset = uiDepth*pCache->NumPartition + copyPURelIdx;

					pCache->TemppuhMv[copyPUOffset].m_x = StartInter->iMvBestY.m_x;
					pCache->TemppuhMv[copyPUOffset].m_y = StartInter->iMvBestY.m_y;
					pCache->TemppuhMvd[copyPUOffset].m_x = StartInter->iMvdBestY.m_x;
					pCache->TemppuhMvd[copyPUOffset].m_y = StartInter->iMvdBestY.m_y;
					pCache->TemppuhMvpIdx[copyPUOffset] = StartInter->iMvp_index;
					pCache->TemppuhSkipFlag[copyPUOffset] = StartInter->SkipFlag;
					pCache->TemppuhMergeFlag[copyPUOffset] = StartInter->MergeFlag;
					pCache->TemppuhMergeIdx[copyPUOffset] = StartInter->MergeIdx;
					pCache->TemppuhInterSAD[copyPUOffset] = StartInter->iSadBest;
					pCache->TemppuhInterPartSize[copyPUOffset] = (UInt8)SIZE_Nx2N;
				}
			}
			MotionCompensation( h, StartInter->iMvBestY, StartInter->iSrchCenBest.m_x, StartInter->iSrchCenBest.m_y, uiDepth, BestPartSize, PUpartindex, subPUWidth, subPUHeight);
		}
	}
}

void MotionCompensation( MTC265_t *h, MVpara MvY, Int32 iCopyStart_x, Int32 iCopyStart_y, UInt8 uiDepth, MTC265_PartSize ePartSize, Int32 PUpartindex, UInt32 subPUWidth, UInt32 subPUHeight)
{
	MTC265_Cache* pCache = &h->cache;
	UInt32 subPUX = 0;
	UInt32 subPUY = 0;
	xEncGetPUPosition( h, ePartSize, uiDepth, PUpartindex, &subPUX, &subPUY );
	pCache->subPUX = subPUX;
	pCache->subPUY = subPUY;

	xPredInterLumaBlk(h, uiDepth, MvY, iCopyStart_x, iCopyStart_y, subPUWidth, subPUHeight);
	xPredInterChromaBlk(h, uiDepth, MvY, iCopyStart_x, iCopyStart_y, subPUWidth, subPUHeight);
}

void xPredInterLumaBlk(MTC265_t *h, UInt8 uiDepth, MVpara MvY, Int32 iCopyStart_x, Int32 iCopyStart_y, UInt32 subPUWidth, UInt32 subPUHeight)
{
	MTC265_Cache* pCache = &h->cache;
	UInt width = subPUWidth;
	UInt height = subPUHeight;
	UInt32 subPUX = pCache->subPUX;
	UInt32 subPUY = pCache->subPUY;
	Int refStride = h->usLargeWidth;  
	Pel *ref      = h->LargeInter.pucY + iCopyStart_y * refStride + iCopyStart_x;

	Int dstStride = MAX_CU_SIZE;
	Pel *dst      = pCache->TemppuhRefY + uiDepth * dstStride * dstStride + subPUY*dstStride + subPUX;

	Int xFrac = MvY.m_x & 0x3;
	Int yFrac = MvY.m_y & 0x3;

	if ( yFrac == 0 )
	{
		filterHorLuma( ref, refStride, dst, dstStride, width, height, xFrac, true );
	}
	else if ( xFrac == 0 )
	{
		filterVerLuma( ref, refStride, dst, dstStride, width, height, yFrac, true, true );
	}
	else
	{
		Int tmpStride =  MAX_CU_SIZE + 16;
		Short *tmp    = pCache->filteredBlockTmp[0];

		Int filterSize = NTAPS_LUMA;
		Int halfFilterSize = ( filterSize >> 1 );

		filterHorLuma(ref - (halfFilterSize-1)*refStride, refStride, tmp, tmpStride, width, height+filterSize-1, xFrac, false     );
		filterVerLuma(tmp + (halfFilterSize-1)*tmpStride, tmpStride, dst, dstStride, width, height,              yFrac, false, true);    
	}
}

void xPredInterChromaBlk(MTC265_t *h, UInt8 uiDepth, MVpara MvY, Int32 iCopyStart_x, Int32 iCopyStart_y, UInt32 subPUWidth, UInt32 subPUHeight)
{
	MTC265_Cache* pCache = &h->cache;
	Int refStride = h->usLargeWidth/2;
	Int dstStride = MAX_CU_SIZE/2;
	UInt32 subPUX = pCache->subPUX;
	UInt32 subPUY = pCache->subPUY;

	Pxl* refCb = h->LargeInter.pucU + iCopyStart_y/2 * refStride + iCopyStart_x/2;
	Pxl* refCr = h->LargeInter.pucV + iCopyStart_y/2 * refStride + iCopyStart_x/2;

	Pxl* dstCb = pCache->TemppuhRefU + uiDepth * dstStride * dstStride + subPUY/2*dstStride + subPUX/2;
	Pxl* dstCr = pCache->TemppuhRefV + uiDepth * dstStride * dstStride + subPUY/2*dstStride + subPUX/2;

	Int xFrac = MvY.m_x & 0x7;
	Int yFrac = MvY.m_y & 0x7;
	UInt cxWidth = subPUWidth >> 1;
	UInt cxHeight = subPUHeight >> 1;

	Int extStride = MAX_CU_SIZE + 16;
	Pxl* extY = pCache->filteredBlockTmp[0];

	Int filterSize = NTAPS_CHROMA;

	Int halfFilterSize = (filterSize>>1);

	if ( yFrac == 0 )
	{
		filterHorChroma(refCb, refStride, dstCb,  dstStride, cxWidth, cxHeight, xFrac, true);    
		filterHorChroma(refCr, refStride, dstCr,  dstStride, cxWidth, cxHeight, xFrac, true);    
	}
	else if ( xFrac == 0 )
	{
		filterVerChroma(refCb, refStride, dstCb, dstStride, cxWidth, cxHeight, yFrac, true, true);    
		filterVerChroma(refCr, refStride, dstCr, dstStride, cxWidth, cxHeight, yFrac, true, true);    
	}
	else
	{
		filterHorChroma(refCb - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
		filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCb, dstStride, cxWidth, cxHeight  , yFrac, false, true);

		filterHorChroma(refCr - (halfFilterSize-1)*refStride, refStride, extY,  extStride, cxWidth, cxHeight+filterSize-1, xFrac, false);
		filterVerChroma(extY  + (halfFilterSize-1)*extStride, extStride, dstCr, dstStride, cxWidth, cxHeight  , yFrac, false, true);    
	}
}