#ifndef __MTC265_H__
#define __MTC265_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "config.h"
#include "bitstream.h"
#include "utils.h"
   
#define CHECK_SEI 1
#define g_uiAddCUDepth 1

// supported slice type
typedef enum {
    SLICE_I = 2,
    SLICE_P = 1,
    SLICE_B = 0
} MTC265_SliceType;

// reference list index
typedef enum 
{
  REF_PIC_LIST_0 = 0,   ///< reference list 0
  REF_PIC_LIST_1 = 1,   ///< reference list 1
  REF_PIC_LIST_C = 2,   ///< combined reference list for uni-prediction in B-Slices
  REF_PIC_LIST_X = 100  ///< special mark
} MTC265_RefPicList;

// chroma formats (according to semantics of chroma_format_idc)
typedef enum {
    CHROMA_400  = 0,
    CHROMA_420  = 1,
    CHROMA_422  = 2,
    CHROMA_444  = 3
} eChromaFormat;

// reference frame
typedef struct MTC265_Frame {
    Pxl   *pucY;
    Pxl   *pucU;
    Pxl   *pucV;
} MTC265_Frame;

// reference frame
typedef struct MTC265_Frame_8Bit {
	UInt8	*pucY;
	UInt8   *pucU;
	UInt8   *pucV;
} MTC265_Frame_8Bit;

// cache for every processor
typedef enum {
    MODE_INVALID    = 255,
    MODE_PLANE      =   0,
    MODE_VER        =   1,
    MODE_HOR        =   2,
    MODE_DC         =   3,
} eIntraMode;

// coefficient scanning type used in ACS
typedef enum {
    SCAN_ZIGZAG = 0,    ///< typical zigzag scan
    SCAN_HOR    = 1,    ///< horizontal first scan
    SCAN_VER    = 2,    ///< vertical first scan
    SCAN_DIAG   = 3     ///< up-right diagonal scan
} eScanType;

//for cu partition and inter prediction
typedef enum 
{
  SIZE_2Nx2N,           ///< symmetric motion partition,  2Nx2N
  SIZE_2NxN,            ///< symmetric motion partition,  2Nx N
  SIZE_Nx2N,            ///< symmetric motion partition,   Nx2N
  SIZE_NxN,             ///< symmetric motion partition,   Nx N
  SIZE_2NxnU,           ///< asymmetric motion partition, 2Nx( N/2) + 2Nx(3N/2)
  SIZE_2NxnD,           ///< asymmetric motion partition, 2Nx(3N/2) + 2Nx( N/2)
  SIZE_nLx2N,           ///< asymmetric motion partition, ( N/2)x2N + (3N/2)x2N
  SIZE_nRx2N,           ///< asymmetric motion partition, (3N/2)x2N + ( N/2)x2N
  SIZE_NONE = 15
}MTC265_PartSize;

typedef enum  
{
  MODE_INTER,           ///< inter-prediction mode
  MODE_INTRA,           ///< intra-prediction mode
  MODE_NONE = 15
}MTC265_PredMode;

typedef enum  
{
	VALID_L,
	VALID_LB,
	VALID_RT,
	VALID_T,
	VALID_LT
}FindMv_Position;

typedef enum  
{
	MODE_AMVP,           ///< inter-prediction mode
	MODE_MERGE           ///< intra-prediction mode
}FindMv_Mode;

typedef struct MVpara
{
	Int32 m_x;
	Int32 m_y;
}  MVpara;

typedef struct Interpara
{
	MVpara iSrchCenBest;
	MVpara iMvBestY;
	MVpara iMvdBestY;
	MVpara Mvp_0;
	MVpara Mvp_1;
	MVpara iMvpBestY;
	Int32 iMvp_index;
	UInt8 ucPointNr;
	Int32 SkipFlag;
	Int32 MergeFlag;
	Int32 MergeIdx;
	double  iSadBest;
} Interpara;

//for cu partition
typedef struct MTC265_Cache {
    UInt32  uiOffset;
    UInt8*  pucTopModeY;                 //上一行的LCU的预测模式：初始化大小为NumPartition*usWidth/MaxCuDepth 
    UInt8*  pucLeftModeY;                //左边LCU的预测模式：初始化大小为NumPartition
	UInt8*  FrameModeY;                  //一整帧的LCU的预测模式  
	UInt8*  FrameDepth;                  //一整帧的LCU的深度
	UInt8*  FramePartSize;

    // current
    Pxl   pucPixY[MAX_CU_SIZE * MAX_CU_SIZE    ];
    Pxl   pucPixU[MAX_CU_SIZE * MAX_CU_SIZE / 4];
    Pxl   pucPixV[MAX_CU_SIZE * MAX_CU_SIZE / 4];
    Pxl   pucRecY[MAX_CU_SIZE * MAX_CU_SIZE    ];
    Pxl   pucRecU[MAX_CU_SIZE * MAX_CU_SIZE / 4];
    Pxl   pucRecV[MAX_CU_SIZE * MAX_CU_SIZE / 4];
    UInt8   nBestModeY;
    UInt8   nBestModeC;

    // IntraPred buffer
    Pxl   pucPixRef[2][4*MAX_CU_SIZE+1];          //< 0:ReconPixel, 1:Filtered
    Pxl   pucPixRefC[2][4*MAX_CU_SIZE/2+1];       //< 0:ReconPixel, 1:Filtered
    Pxl   pucPredY[MAX_CU_SIZE * MAX_CU_SIZE];
    Pxl   pucPredC[3][MAX_CU_SIZE * MAX_CU_SIZE/4];   // 0:U, 1:V, 2:LM
    UInt8   ucMostModeY[3];
	UInt8   TempucMostModeY[3][MAX_CU_SIZE];
    UInt8   ucMostModeC[5];
    UInt8   bValid[5];

    // Encode coeff buffer   
	UInt8 * TempbCbfY;               //NumPartition*MaxDepth
	UInt8 * TempbCbfU;               //NumPartition*MaxDepth
	UInt8 * TempbCbfV;               //NumPartition*MaxDepth
    UInt8   bCbf[3];
    Int16   piTmp[2][MAX_CU_SIZE*MAX_CU_SIZE];

	Pxl   pucTempRecY[MAX_CU_DEPTH ][MAX_CU_SIZE * MAX_CU_SIZE    ];  //临时存放每一深度重建像素
	Pxl   pucTempRecU[MAX_CU_DEPTH ][MAX_CU_SIZE * MAX_CU_SIZE / 4];
	Pxl   pucTempRecV[MAX_CU_DEPTH ][MAX_CU_SIZE * MAX_CU_SIZE / 4];
	Int16   psTempCoefY[MAX_CU_DEPTH ][MAX_CU_SIZE*MAX_CU_SIZE];
	Int16   psTempCoefU[MAX_CU_DEPTH ][MAX_CU_SIZE*MAX_CU_SIZE/4];      //临时存放每一深度变换系数
	Int16   psTempCoefV[MAX_CU_DEPTH ][MAX_CU_SIZE*MAX_CU_SIZE/4];
	Int     Depth;                             //深度
	Int     NumPartition  ;                    //CU中的最大划分块数量
	Int     NumPartitionInWidth  ;             //CU中的最大划分块数量
	UInt8   uiMaxCUWidth;
    UInt8   uiMinCUWidth;
	UInt32  cuX;   //在LCU中的相对位置
	UInt32  cuY;   //在LCU中的相对位置
	Int   * PartIdx;                 //初始化大小为MaxDepth
	Int   * RelativeIdx;                 //初始化大小为MaxDepth
	UInt8 * TemppuhWidth ;              //临时存储划分块的宽度 初始化时分配大小为(MaxDepth+1)NumPartition*sizeof(char)
	UInt8 * TemppuhHeight ;             //临时存储划分块的高度    如上
	UInt8 * TemppuhDepth ;              //临时存储划分块的深度    如上
	UInt8 * TemppePartSize  ;           //临时存储SIZE_2N*2N或者SIZE_N*N   如上
	UInt8 * TemppePredMode  ; 
	UInt8 * TemppuhLumaIntraDir ;       //临时存储划分块的亮度预测模式   如上
	UInt8 * TemppuhChromaIntraDir;      //临时存储划分块的色度预测模式   如上
	double* TempTotalCost ;           //临时存储每一深度的cu的总Cost   初始化时分配大小为MaxDepth*sizeof(Double)
	double* TempTotalDistortion;  
	double* TempSadC;
	UInt8   puhWidth ;              //存储划分块的宽度 初始化时分配大小为NumPartitionsizeof(char)
	UInt8   puhHeight ;             //存储划分块的高度    如上
	UInt8   puhDepth ;              //存储划分块的深度    如上
	UInt8   pePartSize  ;           //存储SIZE_2N*2N或者SIZE_N*N   如上
	UInt8   puhLumaIntraDir ;       //存储划分块的亮度预测模式   如上
	UInt8   puhChromaIntraDir;      //存储划分块的色度预测模式   如上
	double  TotalCost ;           //存储cu的总Cost   初始化时分配大小为NumPartition*MaxDepth*sizeof(Double)
	double  TotalDistortion;      //存储cu的总失真
	double  TotalDistortionC;
	UInt32  TotalBits  ;          //存储cu的总比特数
	UInt32  TotalBins ;           //
	UInt8   pucReferenceLuma;      //查找参考像素指针
	UInt8   pucReferenceChroma;
	double  Lambda;              //拉格朗日系数
	UInt8   bCbfY;              
	UInt8   bCbfU;               
	UInt8   bCbfV;
  
	UInt32 subPUX;//当前PU的位置
	UInt32 subPUY;//当前PU的位置
	UInt32 subPUWidth;
	UInt32 subPUHeight;

	MVpara* BestpuhMvd;//NumPartition，64,存储最优MVd
	MVpara* BestpuhMv;//NumPartition，64,存储最优MV
	Int32* BestpuhMvpIdx;//NumPartition，64,存储最优MVpIdx
	Int32* BestpuhSkipFlag;//NumPartition，64,存储最优SkipFlag
	Int32* BestpuhMergeFlag;//NumPartition，64,存储最优MergeFlag
	Int32* BestpuhMergeIdx;//NumPartition，64,存储最优MergeId
	UInt8* BestpuhInterDepth;//NumPartition，64,存储最优Depth
	UInt8* BestpuhInterPartSize;//NumPartition，64,存储最优PartSize
	double* BestpuhInterSAD;//NumPartition，64,存储各层CU的临时SAD
	UInt8* BestInterCbfY;//NumPartition，64,存储最优CbfY           
	UInt8* BestInterCbfU;//NumPartition，64,存储最优CbfU               
	UInt8* BestInterCbfV;//NumPartition，64,存储最优CbfV

	Pxl* TemppuhRefY;//ucMaxCUDepth*MAX_CU_SIZE*MAX_CU_SIZE,3*64*64,存储各层CU的临时参考帧Y数据
	Pxl* TemppuhRefU;//ucMaxCUDepth*(MAX_CU_SIZE/2)*(MAX_CU_SIZE/2),3*32*32,存储各层CU的临时参考帧U数据
	Pxl* TemppuhRefV;//ucMaxCUDepth*(MAX_CU_SIZE/2)*(MAX_CU_SIZE/2),3*32*32存储各层CU的临时参考帧V数据
	Pxl  pucTempBinRecY[MAX_CU_DEPTH ][MAX_CU_SIZE * MAX_CU_SIZE    ];  //临时存放每一深度重建像素
	Pxl  pucTempBinRecU[MAX_CU_DEPTH ][MAX_CU_SIZE * MAX_CU_SIZE / 4];
	Pxl  pucTempBinRecV[MAX_CU_DEPTH ][MAX_CU_SIZE * MAX_CU_SIZE / 4];
	Pxl  psTempBinCoefY[MAX_CU_DEPTH ][MAX_CU_SIZE*MAX_CU_SIZE];
	Pxl  psTempBinCoefU[MAX_CU_DEPTH ][MAX_CU_SIZE*MAX_CU_SIZE/4];
	Pxl  psTempBinCoefV[MAX_CU_DEPTH ][MAX_CU_SIZE*MAX_CU_SIZE/4];
	MVpara* TemppuhMvd;//ucMaxCUDepth*NumPartition，3*64,存储各层CU的临时MVd
	MVpara* TemppuhMv;//ucMaxCUDepth*NumPartition，3*64,存储各层CU的临时MV
	Int32* TemppuhMvpIdx;//ucMaxCUDepth*NumPartition，3*64,存储各层CU的临时MVpIdx
	Int32* TemppuhSkipFlag;//ucMaxCUDepth*NumPartition，3*64,存储各层CU的临时SkipFlag
	Int32* TemppuhMergeFlag;//ucMaxCUDepth*NumPartition，3*64,存储各层CU的临时MergeFlag
	Int32* TemppuhMergeIdx;//ucMaxCUDepth*NumPartition，3*64,存储各层CU的临时MergeIdx
	UInt8* TemppuhInterPartSize;//ucMaxCUDepth*NumPartition，3*64,存储各层CU的临时PartSize
	double* TemppuhInterSAD;//ucMaxCUDepth*NumPartition，3*64,存储各层CU的临时SAD

	Int32* PUPartIdx;//当前uiDepth的PUPartIdx（会随着PU改变发生变化）
	Int32* PURelativeIdx;//当前uiDepth的PURelativeIdx（会随着PU改变发生变化）
	Int32* TranUsedFlag;//NumPartition，64,存储4个块已经做过变换
	Pxl filteredBlockTmp[4][(MAX_CU_SIZE+16) * (MAX_CU_SIZE+8)];//extWidth为MAX_CU_SIZE+16，extHeight为MAX_CU_SIZE+8
	Pxl filteredBlock[4][4][(MAX_CU_SIZE+16) * (MAX_CU_SIZE+8)];//extWidth为MAX_CU_SIZE+16，extHeight为MAX_CU_SIZE+8
} MTC265_Cache;

typedef struct {
    // Engine
    UInt32  uiLow;
    UInt32  uiRange;
    Int32   iBitsLeft;
    UInt8   ucCache;
    UInt32  uiNumBytes;

    // Context Model
    UInt8   contextModels[MAX_NUM_CTX_MOD];
#define OFF_SPLIT_FLAG_CTX          ( 0 )
#define OFF_SKIP_FLAG_CTX           ( OFF_SPLIT_FLAG_CTX        +   NUM_SPLIT_FLAG_CTX      )
#define OFF_ALF_CTRL_FLAG_CTX       ( OFF_SKIP_FLAG_CTX         +   NUM_SKIP_FLAG_CTX       )
#define OFF_MERGE_FLAG_EXT_CTX      ( OFF_ALF_CTRL_FLAG_CTX     +   NUM_ALF_CTRL_FLAG_CTX   )
#define OFF_MERGE_IDX_EXT_CTX       ( OFF_MERGE_FLAG_EXT_CTX    +   NUM_MERGE_FLAG_EXT_CTX  )
#define OFF_PART_SIZE_CTX           ( OFF_MERGE_IDX_EXT_CTX     +   NUM_MERGE_IDX_EXT_CTX   )
#define OFF_CU_AMP_CTX              ( OFF_PART_SIZE_CTX         +   NUM_PART_SIZE_CTX       )
#define OFF_PRED_MODE_CTX           ( OFF_CU_AMP_CTX            +   NUM_CU_AMP_CTX          )
#define OFF_INTRA_PRED_CTX          ( OFF_PRED_MODE_CTX         +   NUM_PRED_MODE_CTX       )
#define OFF_CHROMA_PRED_CTX         ( OFF_INTRA_PRED_CTX        +   NUM_ADI_CTX             )
#define OFF_INTER_DIR_CTX           ( OFF_CHROMA_PRED_CTX       +   NUM_CHROMA_PRED_CTX     )
#define OFF_MVD_CTX                 ( OFF_INTER_DIR_CTX         +   NUM_INTER_DIR_CTX       )
#define OFF_REF_PIC_CTX             ( OFF_MVD_CTX               +   NUM_MV_RES_CTX          )
#define OFF_DELTA_QP_CTX            ( OFF_REF_PIC_CTX           +   NUM_REF_NO_CTX          )
#define OFF_QT_CBF_CTX              ( OFF_DELTA_QP_CTX          +   NUM_DELTA_QP_CTX        )
#define OFF_QT_ROOT_CBF_CTX         ( OFF_QT_CBF_CTX            + 2*NUM_QT_CBF_CTX          )
#define OFF_SIG_CG_FLAG_CTX         ( OFF_QT_ROOT_CBF_CTX       +   NUM_QT_ROOT_CBF_CTX     )
#define OFF_SIG_FLAG_CTX            ( OFF_SIG_CG_FLAG_CTX       + 2*NUM_SIG_CG_FLAG_CTX     )
#define OFF_LAST_X_CTX              ( OFF_SIG_FLAG_CTX          +   NUM_SIG_FLAG_CTX        )
#define OFF_LAST_Y_CTX              ( OFF_LAST_X_CTX            + 2*NUM_LAST_FLAG_XY_CTX    )
#define OFF_ONE_FLAG_CTX            ( OFF_LAST_Y_CTX            + 2*NUM_LAST_FLAG_XY_CTX    )
#define OFF_ABS_FLAG_CTX            ( OFF_ONE_FLAG_CTX          +   NUM_ONE_FLAG_CTX        )
#define OFF_MVP_IDX_CTX             ( OFF_ABS_FLAG_CTX          +   NUM_ABS_FLAG_CTX        )
#define OFF_TRANS_SUBDIV_FLAG_CTX   ( OFF_MVP_IDX_CTX           +   NUM_MVP_IDX_CTX         )

} MTC265_Cabac;

// main handle
typedef struct MTC265_t {
    // Local
    MTC265_BitStream  bs;
    MTC265_Cabac      cabac;
    MTC265_SliceType  eSliceType;
	MTC265_PartSize   ePartSize;  
	MTC265_PredMode   ePredMode; 
    MTC265_Frame      refn[MAX_REF_NUM+1];
    MTC265_Frame      *pFrameRec;
	MTC265_Frame_8Bit pFrameRec8Bit;
    MTC265_Frame      *pFrameCur;
    MTC265_Cache      cache;
    Int32           iPoc;
    Int32           iQP;
    UInt32          uiCUX;
    UInt32          uiCUY;

    // Interface
    // Profile
    UInt8   ucProfileIdc;
    UInt8   ucLevelIdc;

    // Params
    UInt16  usWidth;
    UInt16  usHeight;
    UInt8   ucMaxCUWidth;
    UInt8   ucMinCUWidth;
    UInt8   ucMaxCUDepth;
    UInt8   ucQuadtreeTULog2MinSize;
    UInt8   ucQuadtreeTULog2MaxSize;
    UInt8   ucQuadtreeTUMaxDepthInter;
    UInt8   ucQuadtreeTUMaxDepthIntra;
    UInt8   ucMaxNumRefFrames;
    UInt8   ucBitsForPOC;
    UInt8   ucMaxNumMergeCand;
    UInt8   ucTSIG;
	Int  NumPartition  ;
	UInt16 PAD_YSIZEw;      //亮度补边宽
	UInt16 PAD_YSIZEh;      //亮度补边高
	UInt16 PAD_CSIZEw;      //色度补边宽
	UInt16 PAD_CSIZEh;      //色度补边高

    // Feature
    UInt8   bUseNewRefSetting;
    UInt8   bUseSAO;
	UInt8   bUseLoopFilter;
	UInt8   bWriteReconFile;
    UInt8   bUseLMChroma;
    UInt8   bMRG;
    UInt8   bLoopFilterDisable;
    UInt8   bSignHideFlag;
    UInt8   bEnableTMVPFlag;

    
	UInt	m_uiNumPartitions;         //一个LCU里最大划分个数
	bool	m_disableDeblockingFilterFlag;
	UInt	numCUInLine;
	bool	bInternalEdge;
	bool	bLeftEdge;
	bool	bTopEdge;
	UInt8*	m_aapucBS[2];
	bool*	m_aapbEdgeFilter[2];
    

    UInt GOPSize;
	Int  IntraPeriod;
	UInt FrameRate;
	bool isIntra;

	//psnr
	double * FpsnrY;    //每一帧的Y分量PSNR
	double * FpsnrU;    //每一帧的U分量PSNR
	double * FpsnrV;    //每一帧的V分量PSNR

	UInt16  usLargeWidth;
	UInt16  usLargeHeight;
	UInt16  usAddRefLength;
	MTC265_Frame LargeInter;//增加边缘的参考帧

	Interpara* TempInter;//存储原始MV,
	Interpara* PrevInter[MAX_REF_NUM+1];//存储参考帧的MV指针数组
	MVpara* FrameBestMv;//最优的MV，64*iLCUNum
	MVpara* FrameBestMvd;//最优的MV，64*iLCUNum
	MVpara* PrevFrameBestMv[MAX_REF_NUM+1];
	MVpara* PrevFrameBestMvd[MAX_REF_NUM+1];
	UInt8* FrameBestPartSize;//最优的PartSize，64*iLCUNum
	UInt8* PrevFrameBestPartSize[MAX_REF_NUM+1];
	UInt8* FrameBestDepth;//最优的Depth，64*iLCUNum
	UInt8* PrevFrameBestDepth[MAX_REF_NUM+1];
	Int32* FrameBestMergeFlag;
	Int32* FrameBestMergeIdx;
	Int32* FrameBestSkipFlag;
	Int32 PUnumber;
	double Inter_lamada;
	double sqrt_Inter_lamada;
} MTC265_t;

// ***************************************************************************
// * bitstream.cpp
// ***************************************************************************
void xWriteSPS( MTC265_t *h );
void xWritePPS( MTC265_t *h );
void xWriteSliceHeader( MTC265_t *h );
void xWriteSliceEnd( MTC265_t *h );
Int32 xPutRBSP(UInt8 *pucDst, UInt8 *pucSrc, UInt32 uiLength);
void xCabacInit( MTC265_t *h );
void xCabacReset( MTC265_Cabac *pCabac );
void xCabacFlush( MTC265_Cabac *pCabac, MTC265_BitStream *pBS );
UInt xCabacGetNumWrittenBits( MTC265_Cabac *pCabac, MTC265_BitStream *pBS );
void xCabacEncodeBin( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue, UInt nCtxState );
void xCabacEncodeBinEP( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue );
void xCabacEncodeBinsEP( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValues, Int numBins );
void xCabacEncodeTerminatingBit( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue );
void xWriteEpExGolomb( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiSymbol, UInt uiCount );
void xWriteGoRiceExGolomb( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiSymbol, UInt &ruiGoRiceParam );
void xWriteUnarySymbol( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue, UInt nCtxState, Int iOffset );
void xWriteUnaryMaxSymbol( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue, UInt nCtxState, Int iOffset, UInt uiMaxSymbol );
//for entropy bits estimate
extern UInt g_uiBinsCoded;
extern Int g_binCountIncrement;
extern UInt64 g_fracBits;
//for SBACRD
void xCabacEncodeBinCounter( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue, UInt nCtxState );
void xCabacEncodeBinEPCounter( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue );
void xCabacEncodeBinsEPCounter( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValues, Int numBins );
void xCabacEncodeTerminatingBitCounter( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue );
void xWriteGoRiceExGolombCounter( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiSymbol, UInt &ruiGoRiceParam );
void xWriteEpExGolombCounter( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiSymbol, UInt uiCount ); 
// ***************************************************************************
// * set.cpp
// ***************************************************************************
void xDefaultParams( MTC265_t *h );
int xCheckParams( MTC265_t *h );

// ***************************************************************************
// * Table.cpp
// ***************************************************************************
extern const UInt8 g_aucIntraFilterType[5][NUM_INTRA_MODE-1];
extern const Int8 g_aucIntraPredAngle[NUM_INTRA_MODE-1];
extern const Int16 g_aucInvAngle[NUM_INTRA_MODE-1];
extern const Int8 g_aiT4[4*4];
extern const Int8 g_aiT8[8*8];
extern const Int8 g_aiT16[16*16];
extern const Int8 g_aiT32[32*32];
extern const Int16 g_quantScales[6];
extern const UInt8 g_invQuantScales[6];
extern const UInt8 g_aucChromaScale[52];
extern const UInt8 g_aucNextStateMPS[128];
extern const UInt8 g_aucNextStateLPS[128];
extern const UInt8 g_aucLPSTable[64][4];
extern const UInt8 g_aucRenormTable[32];
extern const UInt16 *g_ausScanIdx[4][5];
extern const UInt16 g_sigLastScan8x8[4][4];
extern const UInt16 g_sigLastScanCG32x32[64];
extern const UInt8 g_uiMinInGroup[10];
extern const UInt8 g_uiGroupIdx[32];
extern const UInt8 g_uiLastCtx[28];
extern const UInt8 g_auiGoRiceRange[5];
extern const UInt8 g_auiGoRicePrefixLen[5];
extern const UInt g_aauiGoRiceUpdate[5][24];
extern UInt8 g_nextState[128][2];
extern const Int g_entropyBits[128];
void buildNextStateTable();

// ***************************************************************************
// * Encode.cpp
// ***************************************************************************
void xEncInit( MTC265_t *h, UInt nFrames );
void xEncFree( MTC265_t *h );
Int32 xEncEncode( MTC265_t *h, MTC265_Frame *pFrame, UInt8 *pucOutBuf, UInt32 uiBufSize, bool g_bSeqFirst );
void CompressCU(MTC265_t *h,UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY );
void xEncCahceInit( MTC265_t *h );
void xEncCahceDelete( MTC265_t *h );
void xEncCacheStoreCU( MTC265_t *h, UInt uiX, UInt uiY, UInt nCUWidth, UInt8  uiDepth );
void xEncCacheLoadCU( MTC265_t *h, UInt uiX, UInt uiY ,UInt uiWidth );
void xEncCacheUpdate( MTC265_t *h, UInt32 uiX, UInt32 uiY, UInt nWidth, UInt nHeight );
void xEncCahceInitLine( MTC265_t *h, UInt y );
void xEncCacheReset( MTC265_t *h );
//padding
void xFramePadding(MTC265_t *h, MTC265_Frame *pFrm,UInt8 *buf,Pxl *ptr);  
//calculate psnr
void xCalculateAddPSNR(MTC265_t *h,UInt nFrame);
void initZscanToRaster ( Int iMaxDepth, UInt8 iDepth, UInt uiStartVal, UInt*& rpuiCurrIdx );
void initRasterToZscan ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth );
void initRasterToPelXY ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth );
void initMotionReferIdx ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth );

// ***************************************************************************
// * Pixel.cpp
// ***************************************************************************
typedef UInt32 xSad( Int N, UInt8 *pSrc, UInt nStrideSrc, UInt8 *pRef, UInt nStrideRef );
extern xSad *xSadN[MAX_CU_DEPTH+1];
typedef void xDCT( Int16 *pDst, Int16 *pSrc, UInt nStride, Int nLines, Int nShift );
extern xDCT *xDctN[MAX_CU_DEPTH+1];
extern xDCT *xInvDctN[MAX_CU_DEPTH+1];
void xSubDct ( Int16 *pDst, Pxl *pSrc, Pxl *pRef, UInt nStride, Int16 *piTmp0, Int16 *piTmp1, Int iWidth,  Int iHeight, UInt nMode );
void xIDctAdd( Pxl *pDst, Int16 *pSrc, Pxl *pRef, UInt nStride, Int16 *piTmp0, Int16 *piTmp1, Int iWidth, Int iHeight, UInt nMode );
UInt32 xQuant( Int16 *pDst, Int16 *pSrc, UInt nStride, UInt nQP, Int iWidth, Int iHeight, MTC265_SliceType eSType );
void xDeQuant( Int16 *pDst, Int16 *pSrc, UInt nStride, UInt nQP, Int iWidth, Int iHeight, MTC265_SliceType eSType );
UInt32 xSad_new( Int N, Int M, Pxl *pSrc, UInt nStrideSrc, Pxl *pRef, UInt nStrideRef );
double CalDistortion(Pxl * pSrc, Pxl * pPred, Int32 stride, UInt32 CUSize);
double CalInterDistortionSSE(Pxl * pSrc, Pxl * pPred, Int32 strideSrc, Int32 stridePred, UInt32 CUSize, MTC265_PartSize ePartSize);

// ***************************************************************************
// * Entropy.cpp
// ***************************************************************************
static inline bool isZeroCol( Int addr, Int numUnitsPerRow );   
static inline bool isZeroRow( Int addr, Int numUnitsPerRow ); 
void xWriteCU( MTC265_t *h, UInt uiDepth, UInt uiAbsPartIdx );
void codeSplitFlag( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx, UInt uiDepth );
void codePredMode( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
void codePartSize( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx, UInt uiDepth );
void codePredInfo( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx, UInt uiDepth );
//intra
void codeIntraDirLumaAng( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
void codeIntraDirChroma( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
//inter
void codeSkipFlag( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
void codeMergeIndex( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
void codeMergeFlag( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
void codeInterDir( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
void codeRefFrmIdx( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx, UInt NumRefIdx );
void codeMvd( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
void codeMVPIdx( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx );
void codePUWise( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiAbsPartIdx, UInt uiDepth );
//transform coefficients
void xEncodeCoeffNxN( MTC265_t *h, MTC265_Cache *pCache, MTC265_Cabac *pCabac, MTC265_BitStream *pBS, Int16 *psCoef, UInt uiAbsPartIdx, UInt nSize, UInt nDepth, UInt8 bIsLuma, UInt nLumaMode );
//bit rate estimation
UInt codeLastSignificantXYBinEst( UInt nPosX, UInt nPosY, UInt nSize, UInt8 bIsLuma, UInt nScanIdx );
UInt xWriteCoefRemainExGolombBinEst (  UInt symbol, UInt &rParam );
UInt xEncodeCoeffNxNBinEst(MTC265_t *h, MTC265_Cache *pCache, Int16 *psCoef, UInt uiAbsPartIdx, UInt nSize, UInt8 bIsLuma, UInt nMode );

// ***************************************************************************
// * Intra.cpp
// ***************************************************************************
//void xEncIntraPredLuma( MTC265_t *h, UInt nMode, UInt nSize );
//void xEncIntraPredChroma( MTC265_t *h, UInt nMode, UInt nSize );
extern double eBit[4];
void xEncGetCUPosition(MTC265_t *h,UInt32  uiCUWidth, UInt8  uiDepth,UInt32 uiCUX, UInt32 uiCUY, UInt32* uisubCUX, UInt32* uisubCUY );  //获取cu位置
void xEncIntraCompressCU(MTC265_t *h,UInt8 uiDepth, UInt32 uiCUY, UInt32 uiCUX ); 
void xEncCheckRDCost(MTC265_t *h,MTC265_PartSize ePartSize,UInt8 uiDepth,UInt32 uiCUX, UInt32 uiCUY);                                   //计算SIZE_2N*2N/SIZE_N*N的COST
void xEncCheckBestMode(MTC265_t *h,UInt8 Depth,UInt32  uiCUWidth );                                                                     //比较当前深度和上一深度的SAD
void estIntraPredQT(MTC265_t *h,UInt32 nCUSize,UInt8 uiDepth ,MTC265_PartSize ePartSize);                                               //原265帧内预测在这里
void xEncIntraLoadRef( MTC265_t *h, UInt32 uiX, UInt32 uiY, UInt32 nWidth ,UInt8 uiDepth,UInt32 uiCurrPartUnitIdx,UInt32 uiAbsIdxInLCU);//加载参考像素
void setPartSizeSubParts( MTC265_Cache *pCache, MTC265_PartSize eMode, UInt uiAbsPartIdx, UInt8 uiDepth );                              //设置CU划分大小
void setPredModeSubParts( MTC265_Cache *pCache, MTC265_PredMode PredMode, UInt uiAbsPartIdx, UInt8 uiDepth );                           //predmode是区分帧内帧间的
void setCUPameters( MTC265_Cache *pCache, UInt8 uiDepth );                                                                              //保存各种信息

// ***************************************************************************
// * Inter.cpp
// ***************************************************************************
void xEncInterCompressCU(MTC265_t* h, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, double &SAD_BestCU);
void estInterPredQTLuma( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY);
void estInterPredQTChroma( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY);
void estInterPredQT( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY);
void InterPrediction(MTC265_t *h, Interpara * StartInter, MTC265_PartSize ePartSize, UInt8 uiDepth, UInt32 subPUX, UInt32 subPUY, double& PUBestSAD);
void xEncInterCheckRDCost(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth,UInt32 uiCUX, UInt32 uiCUY, double& totalPUbestSAD, Int32* PURelIdxForCompare);
void DiamondSearch ( MTC265_t *h, Interpara *StartIn, UInt8 uiDepth, MTC265_PartSize ePartSize, Int32 iCenX, Int32 iCenY, Int32 iDist, Int32* SrchWindowSize, bool *t );
Int8 xDiamondSearch ( MTC265_t *h, Interpara *StartIn, UInt8 uiDepth, MTC265_PartSize ePartSize, Int32 iCenX, Int32 iCenY, bool t );
void MvpComparison ( MTC265_t *h, Interpara *StartIn, MTC265_PartSize ePartSize, UInt32 nCUSize, UInt32 subPUX, UInt32 subPUY);
void InterSearch( MTC265_t *h, Interpara *StartIn, UInt8 uiDepth, MTC265_PartSize ePartSize, Int32 iDist, Int32* SrchWindowRange );
void PreLargeInter(MTC265_t *h);
double CalInterDistortion(Pxl * pSrc, Pxl * pPred, Int32 strideSrc, Int32 stridePred, UInt32 CUSize, MTC265_PartSize ePartSize);
void xEncGetPUPosition( MTC265_t* h, MTC265_PartSize ePartSize, UInt8 uiDepth, Int32 PUpartIdx, UInt32* uisubCUX, UInt32* uisubCUY);
void deriveLeftTopIdxGeneral    ( MTC265_t *h, UInt uiLCUIdx, UInt uiCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt uiPUIdx, UInt &ruiPartIdxLT );
void deriveRightTopIdxGeneral   ( MTC265_t *h, UInt uiLCUIdx, UInt uiCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt uiPUIdx, UInt &ruiPartIdxRT );
void deriveLeftBottomIdxGeneral ( MTC265_t *h, UInt uiLCUIdx, UInt uiCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt uiPUIdx, UInt &ruiPartIdxLB );
void deriveRightBottomIdxGeneral( MTC265_t *h, UInt uiLCUIdx, UInt uiCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt uiPUIdx, UInt &ruiPartIdxRB );
void deriveCenterIdxGeneral(MTC265_t *h, UInt uiPUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt &ruiPartIdxCenter);
bool getPULeft		( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPUIdx, UInt &ruiLeftLCUIdx,		UInt &ruiLeftPUIdx );
bool getPUAbove		( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPUIdx, UInt &ruiAboveLCUIdx,		UInt &ruiAbovePUIdx );
bool getPUAboveLeft	( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPUIdx, UInt &ruiAboveLeftLCUIdx,	UInt &ruiAboveLeftPUIdx );
bool getPUAboveRight( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPUIdx, UInt &ruiAboveRightLCUIdx,	UInt &ruiAboveRightPUIdx );
bool getPUBelowLeft	( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPUIdx, UInt &ruiBelowLeftLCUIdx,	UInt &ruiBelowLeftPUIdx );
bool getPUBelowRight( MTC265_t *h, UInt uiCurLCUIdx, UInt uiCurPartIdx, UInt &ruiBelowRightLCUIdx, UInt &ruiBelowRightPartIdx );
void FindMv( MTC265_t *h, FindMv_Mode eFindMv, Int iRefidx, UInt8 uiDepth, UInt uiCurLCUIdx, UInt uiMvpLCUIdx, UInt uiMvpPURelIdx, UInt uiCurPURelIdx, MTC265_PartSize ePartSize, FindMv_Position MvPosition, Int32 PUPartIdx, MVpara& iMv, MVpara& iMvd );//前n参考帧，赋值为-n
void CheckPUBestMode(MTC265_t *h, UInt8 uiDepth, Int32* PURelIdxCompare_SIZE_2Nx2N, Int32* PURelIdxCompare_SIZE_2NxN, Int32* PURelIdxCompare_SIZE_Nx2N, double SAD_2Nx2N, double SAD_2NxN, double SAD_Nx2N, double& SAD_BestPU);
void CheckCUBestMode(MTC265_t *h, UInt8 uiDepth, double SAD_TotalCU, double& SAD_BestCU);
void MotionCompensation( MTC265_t *h, MVpara MvY, Int32 iCopyStart_x, Int32 iCopyStart_y, UInt8 uiDepth, MTC265_PartSize ePartSize, Int32 PUpartindex, UInt32 subPUWidth, UInt32 subPUHeight);
void copyRecCoefCbfUp(MTC265_t *h, Int32 nCUSize, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY);
void setInterDepth(MTC265_t *h, UInt8 uiDepth );
void PrevChangeMv( MTC265_t *h, MVpara * PrevFrameBestMv, MVpara * FrameBestMv);
void calcMvdBinNumber(Int32 Mvd, Int32& BinNumber);
void calcMergeIdxBinNumber(Int32 MergeIdx, Int32& BinNumber);
bool isDiffMER(Int xN, Int yN, Int xP, Int yP);
void getPartPosition( MTC265_t *h, UInt uiLCUIdx, UInt uiAbsCUIdx, UInt uiDepth, MTC265_PartSize eCUMode, UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH );
bool hasEqualMotion( MVpara *mv1, MVpara *mv2 );
void getInterMergeCandidates( MTC265_t *h, UInt uiLCUIdx, UInt CURelIdx, UInt uiDepth, MTC265_PartSize ePartSize, UInt PURelIdx, MVpara *MergeCandidatesMv, MVpara *MergeCandidatesMvd, UInt& numValidMergeCand );
void xCheckRDCostMerge2Nx2N(MTC265_t *h, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, double SAD_2Nx2N, double &SAD_BestCU, Int32& EarlySkipDetection);
void xCheckRDCostMerge(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, double SAD_ME, double& MEandMergeBestSAD,	Int32& tempBinNumberCost);
void xEncInterDistortion(MTC265_t* h, MTC265_PartSize ePartSize, UInt8 uiDepth, UInt32 uiCUX, UInt32 uiCUY, UInt32& totalCoeffNxNBinEst, double& InterDistortion);
void estInterPredQTLumaWithCoeffBin( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY, UInt32& CoeffNxNBinEst);
void estInterPredQTChromaWithCoeffBin( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY, UInt32& CoeffNxNBinEstChroma);
void estInterPredQTWithCoeffBin( MTC265_t *h, UInt32 nCUSize, UInt8 uiDepth, Int32 PURelIdx, UInt32 subCUX, UInt32 subCUY, UInt32& CoeffNxNBinEst);
void xPredInterLumaBlk(MTC265_t *h, UInt8 uiDepth, MVpara MvY, Int32 iCopyStart_x, Int32 iCopyStart_y, UInt32 subPUWidth, UInt32 subPUHeight);
void xPredInterChromaBlk(MTC265_t *h, UInt8 uiDepth, MVpara MvY, Int32 iCopyStart_x, Int32 iCopyStart_y, UInt32 subPUWidth, UInt32 subPUHeight);
void xCheckBestMVP( MTC265_t *h, Interpara * StartInter, UInt MVPidxNum);
void xPatternSearchFracDIF(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth, Int32 iCopyStart_x, Int32 iCopyStart_y, MVpara* pcMvInt, MVpara& rcMvHalf, MVpara& rcMvQter, double& ruiCost);
void xExtDIFUpSamplingH(MTC265_t *h,MTC265_PartSize ePartSize, UInt8 uiDepth, Int32 iCopyStart_x, Int32 iCopyStart_y);
void xExtDIFUpSamplingQ(MTC265_t *h, MTC265_PartSize ePartSize, UInt8 uiDepth, Int32 iCopyStart_x, Int32 iCopyStart_y, MVpara halfPelRef );
typedef MVpara pcMvRefine(Int32 RefinePara);
extern pcMvRefine* pcMvRefineN[2];
double xPatternRefinement( MTC265_t *h, UInt8 uiDepth, MTC265_PartSize ePartSize, MVpara baseRefMv, Int iFrac, MVpara& rcMvFrac );
void xTZ2PointSearch(MTC265_t *h, Interpara* StartInter, UInt8 uiDepth, MTC265_PartSize ePartSize);

// ***************************************************************************
// * Macro for build platform information
// ***************************************************************************
#ifdef __GNUC__
#define MTC265_COMPILEDBY "[GCC %d.%d.%d]", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__
#ifdef __IA64__
#define MTC265_ONARCH     "[on 64-bit] "
#else
#define MTC265_ONARCH     "[on 32-bit] "
#endif
#endif

#ifdef __INTEL_COMPILER
#define MTC265_COMPILEDBY "[ICC %d]", __INTEL_COMPILER
#elif  _MSC_VER
#define MTC265_COMPILEDBY "[VS %d]", _MSC_VER
#endif

#ifndef MTC265_COMPILEDBY
#define MTC265_COMPILEDBY "[Unk-CXX]"
#endif

#ifdef _WIN32
#define MTC265_ONOS       "[Windows]"
#elif  __linux
#define MTC265_ONOS       "[Linux]"
#elif  __CYGWIN__
#define MTC265_ONOS       "[Cygwin]"
#elif __APPLE__
#define MTC265_ONOS       "[Mac OS X]"
#else
#define MTC265_ONOS       "[Unk-OS]"
#endif

#define MTC265_BITS       "[%d bit]", (sizeof(void*) == 8 ? 64 : 32) ///< used for checking 64-bit O/S

#endif /* __MTC265_H__ */
