#ifndef __CONFIG_H__
#define __CONFIG_H__

#define MIN_QP	0
#define MAX_QP  51
#define DEFAULT_INTRA_TC_OFFSET 2 ///< Default intra TC offset

// AMVP: advanced motion vector prediction
#define AMVP_MAX_NUM_CANDS          2           ///< max number of final candidates
#define AMVP_MAX_NUM_CANDS_MEM      3           ///< max number of candidates
// MERGE
#define MRG_MAX_NUM_CANDS           5
#define COEF_REMAIN_BIN_REDUCTION   3           ///< indicates the level at which the VLC 
///< transitions from Golomb-Rice to TU+EG(k)

////////////////////////////
// MTC265 start
////////////////////////////
#define ENC_DEC_TRACE                       (0)

#define MAX_CU_DEPTH                        (4)
#define MAX_REF_NUM                         (1)
#define MAX_WIDTH                           (2048)
#define MAX_HEIGHT                          (2048)
#define MIN_CU_SIZE                         ( 4)
#define MAX_CU_SIZE                         (64)
#define MAX_PU_XY                           (MAX_CU_SIZE / MIN_CU_SIZE)
#define MAX_NUM_SPU_W                       (32)
#define MAX_PART_NUM                        (MAX_PU_XY * MAX_PU_XY)

#define NUM_INTRA_MODE                      (36)
#define NUM_CHROMA_MODE                     ( 6)    // total number of chroma modes
#define PLANAR_IDX                          ( 0)
#define VER_IDX                             (26)    // index for intra VERTICAL   mode
#define HOR_IDX                             (10)    // index for intra HORIZONTAL mode
#define DC_IDX                              ( 1)
#define LM_CHROMA_IDX                       (35)    // chroma mode index for derived from luma intra mode
#define DM_CHROMA_IDX                       (36)    // chroma mode index for derived from luma intra mode
#define SHIFT_INV_1ST                       ( 7)    // Shift after first inverse transform stage
#define SHIFT_INV_2ND                       (12)    // Shift after second inverse transform stage
#define MAX_SAD                             ( 1 << 30 )

#define MAX_UINT							0xFFFFFFFFU ///< max. value of unsigned 32-bit integer
#define MAX_INT								2147483647  ///< max. value of signed 32-bit integer
#define MAX_DOUBLE							1.7e+308    ///< max. value of double-type value

#define ALWAYS_INLINE                       __inline
////////////////////////////
// MTC265 end
////////////////////////////

////////////////////////////
// JCT-VC start
////////////////////////////
#define QUANT_IQUANT_SHIFT              20 // Q(QP%6) * IQ(QP%6) = 2^20
#define QUANT_SHIFT                     14 // Q(4) = 2^14
#define SCALE_BITS                      15 // Inherited from TMuC, pressumably for fractional bit estimates in RDOQ
#define MAX_TR_DYNAMIC_RANGE            15 // Maximum transform dynamic range (excluding sign bit)
////////////////////////////
// JCT-VC end
////////////////////////////

////////////////////////////
// JCT-VC H start
////////////////////////////
#define MULTIBITS_DATA_HIDING           1  ///< H0481: multiple sign bit hiding
#define CABAC_INIT_FLAG                 1  ///< H0540:
#define DBL_CONTROL                     1  ///< H0424: PPS deblocking_filter_control_present_flag (JCTVC-H0398); condition for inherit params flag in SH
#define PARALLEL_MERGE                  1  ///< H0082: parallel merge/skip      
////////////////////////////
// JCT-VC G end
////////////////////////////

////////////////////////////
// JCT-VC G start
////////////////////////////
#define MRG_MAX_NUM_CANDS_SIGNALED      5  ///< G091: value of maxNumMergeCand signaled in slice header 
#define MRG_MAX_NUM_CANDS               5  ///< MERGE
#define MLS_GRP_NUM                    64  ///< G644: Max number of coefficient groups, max(16, 64)
#define MLS_CG_SIZE                     4  ///< G644: Coefficient group size of 4x4
#define SCAN_SET_SIZE                  16
#define LOG2_SCAN_SET_SIZE              4
#define C1FLAG_NUMBER                   8  ///< maximum number of largerThan1 flag coded in one chunk :  16 in HM5
#define C2FLAG_NUMBER                   1  ///< maximum number of largerThan2 flag coded in one chunk:  16 in HM5 
////////////////////////////
// JCT-VC G end
////////////////////////////


/////////////////////////////////
// AHG SLICES defines section start
/////////////////////////////////

/////////////////////////////////
// AHG SLICES defines section end
/////////////////////////////////
#define H0566_TLA                               1
#define RPS_IN_SPS                              1 // Adopted during discussion of JCTVC-H0423
#define H0567_DPB_PARAMETERS_PER_TEMPORAL_LAYER 1
#define LOSSLESS_CODING                         1 ///< H0530: lossless and lossy (mixed) coding
#define PIC_CROPPING                            1 ///< Picture cropping and size constraints
#define NAL_REF_FLAG                            1 ///< Change nal_ref_idc to nal_ref_flag (JCTVC-F463)
#define H0412_REF_PIC_LIST_RESTRICTION          1
#define TILES_WPP_ENTRY_POINT_SIGNALLING        1 // JCTVC-H0556. Assumes either Tiles is ON or WPP is ON (not both simultaneously).


typedef unsigned char       UInt8;
typedef   signed char        Int8;
typedef unsigned short      UInt16;
typedef          short       Int16;
typedef unsigned int        UInt32;
typedef          int         Int32;
typedef			 Int16		 Pxl;  
#ifdef _MSC_VER
typedef unsigned __int64    UInt64;
typedef          __int64     Int64;
#else
typedef unsigned long long  UInt64;
typedef          long long   Int64;
#endif

// Fast integer depends on platform
typedef Int32    Int;
typedef UInt32  UInt;

#define FALSE   (0)
#define TRUE    (1)

#endif /* __CONFIG_H__ */
