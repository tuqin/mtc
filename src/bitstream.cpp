#include "MTC265.h"
#include "bitstream.h"
#include "utils.h"

// ***************************************************************************
UInt g_uiBinsCoded;
Int g_binCountIncrement;
UInt64 g_fracBits = 0;
// ***************************************************************************
#define WRITE_CODE( value, length, name)     xWriteCode ( pBS, value, length )
#define WRITE_UVLC( value,         name)     xWriteUvlc ( pBS, value )
#define WRITE_SVLC( value,         name)     xWriteSvlc ( pBS, value )
#define WRITE_FLAG( value,         name)     xWriteFlag ( pBS, value )

// ***************************************************************************
static void xWriteShortTermRefPicSet( MTC265_t *h )
{
    MTC265_BitStream *pBS = &h->bs;

    WRITE_FLAG( 0, "inter_ref_pic_set_prediction_flag" ); // inter_RPS_prediction_flag
    WRITE_UVLC( 1, "num_negative_pics" );
    WRITE_UVLC( 0, "num_positive_pics" );
    WRITE_UVLC( 0, "delta_poc_s0_minus1" );
    WRITE_FLAG( 1, "used_by_curr_pic_s0_flag"); 
}

void xWriteSPS( MTC265_t *h )
{
    MTC265_BitStream *pBS = &h->bs;
    UInt i;

    WRITE_CODE( h->ucProfileIdc,            8,          "profile_idc" );
    WRITE_CODE( 0,                          8,          "reserved_zero_8bits" );
    WRITE_CODE( h->ucLevelIdc,              8,          "level_idc" );
    WRITE_UVLC( 0,                                      "seq_parameter_set_id" );
    WRITE_UVLC( CHROMA_420,                             "chroma_format_idc" );
    WRITE_CODE( 0,                          3,          "max_temporal_layers_minus1" );
	WRITE_UVLC( h->usWidth + h->PAD_YSIZEw,                             "pic_width_in_luma_samples" );
	WRITE_UVLC( h->usHeight+ h->PAD_YSIZEh,                            "pic_height_in_luma_samples" );

#if PIC_CROPPING
	bool m_picCroppingFlag = TRUE;
    WRITE_FLAG( m_picCroppingFlag,                                      "pic_cropping_flag" );
	if( m_picCroppingFlag )
	{
		WRITE_UVLC( 0, "pic_crop_left_offset" );
		WRITE_UVLC( h->PAD_YSIZEw / 2, "pic_crop_right_offset" );
		WRITE_UVLC( 0, "pic_crop_top_offset" );
		WRITE_UVLC( h->PAD_YSIZEh / 2, "pic_crop_bottom_offset" );
	}
#endif

    WRITE_UVLC( 0,                                      "bit_depth_luma_minus8" );
    WRITE_UVLC( 0,                                      "bit_depth_chroma_minus8" );

    WRITE_FLAG( 0,                                      "pcm_enabled_flag");

#if LOSSLESS_CODING
    WRITE_FLAG( 0,                                      "qpprime_y_zero_transquant_bypass_flag" );
#endif
    WRITE_UVLC( h->ucBitsForPOC-4,                      "log2_max_pic_order_cnt_lsb_minus4" );
#if H0567_DPB_PARAMETERS_PER_TEMPORAL_LAYER
	WRITE_UVLC( 1,                                        "max_dec_pic_buffering[i]" );
	WRITE_UVLC( 0,                                        "num_reorder_pics[i]" );
	WRITE_UVLC( 0,                                        "max_latency_increase[i]" );
#else
#error Please Sync Code
#endif

    UInt32 MinCUSize = h->ucMaxCUWidth >> (h->ucMaxCUDepth - 1);
    UInt32 log2MinCUSize = xLog2(MinCUSize)-1;

#if H0412_REF_PIC_LIST_RESTRICTION
    WRITE_FLAG( 1,                                                                    "restricted_ref_pic_lists_flag" );
    WRITE_FLAG( 0,                                                                    "lists_modification_present_flag" );
#endif
    WRITE_UVLC( log2MinCUSize - 3,                                                    "log2_min_coding_block_size_minus3" );
    WRITE_UVLC( h->ucMaxCUDepth - 1,                                                  "log2_diff_max_min_coding_block_size" );
    WRITE_UVLC( h->ucQuadtreeTULog2MinSize - 2,                                       "log2_min_transform_block_size_minus2" );
    WRITE_UVLC( h->ucQuadtreeTULog2MaxSize - h->ucQuadtreeTULog2MinSize,              "log2_diff_max_min_transform_block_size" );
    if(log2MinCUSize == 3) 
	{
        WRITE_FLAG( 1,  "DisableInter4x4" );
    }
    WRITE_UVLC( h->ucQuadtreeTUMaxDepthInter - 1,                                     "max_transform_hierarchy_depth_inter" );
    WRITE_UVLC( h->ucQuadtreeTUMaxDepthIntra - 1,                                     "max_transform_hierarchy_depth_intra" );
    WRITE_FLAG( 0,                                                                    "scaling_list_enabled_flag" ); 
    WRITE_FLAG( h->bUseLMChroma,                                                      "chroma_pred_from_luma_enabled_flag" );
	//if INTRA_TRANSFORMSKIP
    WRITE_FLAG( 0,                                 "transform_skip_enabled_flag" ); 
    WRITE_FLAG( 0,                                                                    "deblocking_filter_in_aps_enabled_flag");
    WRITE_FLAG( 1,                                                                    "seq_loop_filter_across_slices_enabled_flag");
    WRITE_FLAG( 0,                                                                    "asymmetric_motion_partitions_enabled_flag" );
    WRITE_FLAG( 0,                                                                    "non_square_quadtree_enabled_flag" );
    WRITE_FLAG( 0,                                                                    "sample_adaptive_offset_enabled_flag");
    WRITE_FLAG( 0,                                                                    "adaptive_loop_filter_enabled_flag");
    WRITE_FLAG( 0,                                                                    "temporal_id_nesting_flag" );

#if RPS_IN_SPS
    WRITE_UVLC( 1, "num_short_term_ref_pic_sets" );
    xWriteShortTermRefPicSet( h );
    WRITE_FLAG( 0, "long_term_ref_pics_present_flag" );
#endif
    
    UInt32 splitCUSize = (h->ucMaxCUWidth >> h->ucMaxCUDepth);
    UInt32 nAMVPNum = h->ucMaxCUDepth;
    if ( splitCUSize > h->ucQuadtreeTULog2MinSize ) 
	{
        UInt nDiff = xLog2( splitCUSize - 1) - xLog2( h->ucQuadtreeTULog2MinSize );
        nAMVPNum += nDiff;
    }
    // AMVP mode for each depth
    for (i = 0; i < nAMVPNum; i++) 
	{
        WRITE_FLAG( 1, "AMVPMode");
    }
    
#if TILES_WPP_ENTRY_POINT_SIGNALLING
    WRITE_CODE( 0, 2,                                                                 "tiles_or_entropy_coding_sync_idc" );
#endif
    WRITE_FLAG( 0, "sps_extension_flag" );
    xWriteRBSPTrailingBits(pBS);
}

void xWritePPS( MTC265_t *h )
{
    MTC265_BitStream *pBS = &h->bs;
  
    WRITE_UVLC( 0,  "pic_parameter_set_id" );
    WRITE_UVLC( 0,  "seq_parameter_set_id" );

#if MULTIBITS_DATA_HIDING
    WRITE_FLAG( h->bSignHideFlag, "sign_data_hiding_flag" );
    if( h->bSignHideFlag ) 
	{
        WRITE_CODE(h->ucTSIG, 4, "sign_hiding_threshold");
    }
#endif

#if CABAC_INIT_FLAG
    WRITE_FLAG( 1,                                          "cabac_init_present_flag" );
#endif

#if !RPS_IN_SPS
#error Please Sync Code
#endif

#if !H0566_TLA
#error Please Sync Code
#endif
	WRITE_CODE( 0, 3, "num_ref_idx_l0_default_active_minus1");
	WRITE_CODE( 0, 3, "num_ref_idx_l1_default_active_minus1");
    WRITE_SVLC( 0/*h->iQP - 26*/,                           "pic_init_qp_minus26");
    WRITE_FLAG( 0,                                          "constrained_intra_pred_flag" );
    WRITE_FLAG( h->bEnableTMVPFlag,                         "enable_temporal_mvp_flag" );
    WRITE_CODE( 0, 2,                                       "slice_granularity");
    WRITE_UVLC( 0,                                          "max_cu_qp_delta_depth" );

    WRITE_SVLC( 0,                                          "chroma_qp_offset"     );
    WRITE_SVLC( 0,                                          "chroma_qp_offset_2nd" );

    WRITE_FLAG( 0,                                          "weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
    WRITE_CODE( 0, 2,                                       "weighted_bipred_idc" );  // Use of Weighting Bi-Prediction (B_SLICE)

    WRITE_FLAG( 0,                                          "output_flag_present_flag" );

#if DBL_CONTROL
    WRITE_FLAG( 1,                                          "deblocking_filter_control_present_flag");
#endif
#if PARALLEL_MERGE
    WRITE_UVLC( 0,                                          "log2_parallel_merge_level_minus2");
#endif
    WRITE_FLAG( 0, "pps_extension_flag" );
    xWriteRBSPTrailingBits(pBS);
}

void xWriteSliceHeader( MTC265_t *h )
{
    MTC265_BitStream *pBS = &h->bs;
#if ENC_DEC_TRACE  
    xTraceSliceHeader();
#endif
    //write slice address
    WRITE_FLAG( 1,              "first_slice_in_pic_flag" );

    WRITE_UVLC( h->eSliceType,  "slice_type" );
    WRITE_FLAG( 0,              "lightweight_slice_flag" );

    WRITE_UVLC( 0,              "pic_parameter_set_id" );
    if ( h->eSliceType == SLICE_I && h->iPoc == 0 ) 
	{
		WRITE_UVLC( 0, "idr_pic_id" );
		WRITE_FLAG( 0, "no_output_of_prior_pics_flag" );
		h->iPoc = 0;
    }
    else 
	{
		WRITE_CODE( h->iPoc % (1<<h->ucBitsForPOC), h->ucBitsForPOC, "pic_order_cnt_lsb");
		WRITE_FLAG( 1, "short_term_ref_pic_set_pps_flag");
		WRITE_UVLC( 0, "short_term_ref_pic_set_idx" );
    }

    if ( h->eSliceType != SLICE_I ) 
	{
        WRITE_FLAG( 1 ,                               "num_ref_idx_active_override_flag");
        WRITE_CODE( h->ucMaxNumRefFrames - 1, 3,      "num_ref_idx_l0_active_minus1" );
    }

    if ( h->eSliceType != SLICE_I ) 
	{
        // Use P-Tables now
        WRITE_FLAG( 0, "cabac_init_flag" );
    }

    WRITE_SVLC( h->iQP - 26, "slice_qp_delta" );
    WRITE_FLAG( h->bLoopFilterDisable, "loop_filter_disable" );  // should be an IDC

    assert( h->ucMaxNumMergeCand <= MRG_MAX_NUM_CANDS_SIGNALED );
    assert( MRG_MAX_NUM_CANDS_SIGNALED <= MRG_MAX_NUM_CANDS );
    WRITE_UVLC(MRG_MAX_NUM_CANDS - h->ucMaxNumMergeCand, "maxNumMergeCand");

    WRITE_FLAG( 0, "encodeTileMarkerFlag" );
    xWriteAlignOne(pBS);
}

void xWriteSliceEnd( MTC265_t *h )
{
    MTC265_BitStream *pBS = &h->bs;

    WRITE_FLAG( 1, "stop bit" );
#if TILES_WPP_ENTRY_POINT_SIGNALLING
    xWriteAlignZero(pBS);
#endif
    xWriteRBSPTrailingBits(pBS);
}

Int32 xPutRBSP(UInt8 *pucDst, UInt8 *pucSrc, UInt32 uiLength)
{
    assert( uiLength > 2 );

    UInt8 *pucDst0 = pucDst;
    UInt i;
    *pucDst++ = *pucSrc++;
    *pucDst++ = *pucSrc++;

    for( i=2; i < uiLength; i++ ) 
	{
        const Int nLeadZero = (pucSrc[-2] | pucSrc[-1]);
        if ( (nLeadZero == 0) && (pucSrc[0] <= 3) ) 
		{
            *pucDst++ = 0x03;
        }
        *pucDst++ = *pucSrc++;
    }
    return(pucDst - pucDst0);
}

#define CABAC_ENTER \
    UInt32  uiLow = pCabac->uiLow; \
    UInt32  uiRange = pCabac->uiRange; \
    Int32   iBitsLeft = pCabac->iBitsLeft; \
    UInt8   ucCache = pCabac->ucCache; \
    UInt32  uiNumBytes = pCabac->uiNumBytes;

#define CABAC_LEAVE \
    pCabac->uiLow = uiLow; \
    pCabac->uiRange = uiRange; \
    pCabac->iBitsLeft = iBitsLeft; \
    pCabac->ucCache = ucCache; \
    pCabac->uiNumBytes = uiNumBytes;


// ***************************************************************************
// * Cabac tables
// ***************************************************************************
static const UInt8
INIT_SPLIT_FLAG[3][NUM_SPLIT_FLAG_CTX] = 
{
    { 139,  141,  157, }, 
    { 107,  139,  126, }, 
    { 107,  139,  126, }, 
};

static const UInt8 
INIT_SKIP_FLAG[3][NUM_SKIP_FLAG_CTX] = 
{
    { CNU,  CNU,  CNU, }, 
    { 197,  185,  201, }, 
    { 197,  185,  201, }, 
};

static const UInt8 
INIT_ALF_CTRL_FLAG[3][NUM_ALF_CTRL_FLAG_CTX] = 
{
    { 118, }, 
    { 102, }, 
    { 102, }, 
};

static const UInt8 
INIT_MERGE_FLAG_EXT[3][NUM_MERGE_FLAG_EXT_CTX] = 
{
    { CNU, }, 
    { 110, }, 
    { 154, }, 
};

static const UInt8 
INIT_MERGE_IDX_EXT[3][NUM_MERGE_IDX_EXT_CTX] = 
{
    { CNU, }, 
    { 122, }, 
    { 137, }, 
};

static const UInt8 
INIT_PART_SIZE[3][NUM_PART_SIZE_CTX] = 
{
    { 184,  CNU,  CNU,  CNU, }, 
    { 154,  139,  CNU,  CNU, }, 
    { 154,  139,  CNU,  CNU, }, 
};

static const UInt8 
INIT_CU_AMP_POS[3][NUM_CU_AMP_CTX] = 
{
    { CNU, }, 
    { 154, }, 
    { 154, }, 
};

static const UInt8 
INIT_PRED_MODE[3][NUM_PRED_MODE_CTX] = 
{
    { CNU, }, 
    { 149, }, 
    { 134, }, 
};

static const UInt8 
INIT_INTRA_PRED_MODE[3][NUM_ADI_CTX] = 
{
    { 184, }, 
    { 154, }, 
    { 183, }, 
};

static const UInt8 
INIT_CHROMA_PRED_MODE[3][NUM_CHROMA_PRED_CTX] = 
{
    {  63,  139, }, 
    { 152,  139, }, 
    { 152,  139, }, 
};

static const UInt8 
INIT_INTER_DIR[3][NUM_INTER_DIR_CTX] = 
{
    { CNU,  CNU,  CNU,  CNU, CNU }, 
    {  95,   79,   63,   31, 31 }, 
    {  95,   79,   63,   31, 31 }, 
};

static const UInt8 
INIT_MVD[3][NUM_MV_RES_CTX] = 
{
    { CNU,  CNU, }, 
    { 140,  198, }, 
    { 169,  198, }, 
};

static const UInt8 
INIT_REF_PIC[3][NUM_REF_NO_CTX] = 
{
    { CNU,  CNU,  CNU,  CNU, }, 
    { 153,  153,  139,  CNU, }, 
    { 153,  153,  168,  CNU, }, 
};

static const UInt8 
INIT_DQP[3][NUM_DELTA_QP_CTX] = 
{
    { 154,  154,  154, }, 
    { 154,  154,  154, }, 
    { 154,  154,  154, }, 
};

static const UInt8 
INIT_QT_CBF[3][2*NUM_QT_CBF_CTX] = 
{
    { 111,  141,  CNU,  CNU,  CNU,   94,  138,  182,  CNU,  CNU, }, 
    { 153,  111,  CNU,  CNU,  CNU,  149,  107,  167,  CNU,  CNU, }, 
    { 153,  111,  CNU,  CNU,  CNU,  149,   92,  167,  CNU,  CNU, }, 
};

static const UInt8 
INIT_QT_ROOT_CBF[3][NUM_QT_ROOT_CBF_CTX] = {
    { CNU, }, 
    {  79, }, 
    {  79, }, 
};

static const UInt8 
INIT_LAST[3][2*NUM_LAST_FLAG_XY_CTX] = 
{
    { 
	  110,  110,  124,  110,  140,  111,  125,  111,  127,  111,  111,  156,  127,  127,  111, 
      108,  123,   63,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, 
    },  
	{ 
	  125,  110,   94,  110,  125,  110,  125,  111,  111,  110,  139,  111,  111,  111,  125,  
      108,  123,  108,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,
    }, 
    { 
	  125,  110,  124,  110,  125,  110,  125,  111,  111,  110,  139,  111,  111,  111,  125, 
      108,  123,   93,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, 
	}, 
};

static const UInt8 
INIT_SIG_CG_FLAG[3][2 * NUM_SIG_CG_FLAG_CTX] = 
{
    {
      91,  171,  
     134,  141, 
    },
    {
     121,  140, 
      61,  154, 
    },
    {
     121,  140,  
      61,  154, 
    },
};

static const UInt8 
INIT_SIG_FLAG[3][NUM_SIG_FLAG_CTX] = 
{
	{ 141,  111,  125,  110,  110,   94,  124,  108,  124,  125,  139,  124,   63,  139,  168,  138,  107,  123,   92,  111,  141,  107,  125,  141,  179,  153,  125,  140,  139,  182,  123,   47,  153,  182,  137,  149,  192,  152,  224,  136,   31,  136,   74,  140,  141,  136,  139,  111, },
	{ 170,  154,  139,  153,  139,  123,  123,   63,  153,  168,  153,  152,   92,  152,  152,  137,  122,   92,   61,  155,  185,  166,  183,  140,  136,  153,  154,  155,  153,  123,   63,   61,  167,  153,  167,  136,  149,  107,  136,  121,  122,   91,  149,  170,  185,  151,  183,  140, },
	{ 170,  154,  139,  153,  139,  123,  123,   63,  124,  139,  153,  152,   92,  152,  152,  137,  137,   92,   61,  170,  185,  166,  183,  140,  136,  153,  154,  155,  153,  138,  107,   61,  167,  153,  167,  136,  121,  122,  136,  121,  122,   91,  149,  170,  170,  151,  183,  140, },
};

static const UInt8 
INIT_ONE_FLAG[3][NUM_ONE_FLAG_CTX] = 
{
    { 140,   92,  137,  138,  140,  152,  138,  139,  153,   74,  149,   92,  139,  107,  122,  152,  140,  179,  166,  182,  140,  227,  122,  197, }, 
    { 154,  196,  196,  167,  154,  152,  167,  182,  182,  134,  149,  136,  153,  121,  136,  137,  169,  194,  166,  167,  154,  167,  137,  182, }, 
    { 154,  196,  167,  167,  154,  152,  167,  182,  182,  134,  149,  136,  153,  121,  136,  122,  169,  208,  166,  167,  154,  152,  167,  182, }, 
};

static const UInt8 
INIT_ABS_FLAG[3][NUM_ABS_FLAG_CTX] = 
{
    { 138,  153,  136,  167,  152,  152, }, 
    { 107,  167,   91,  122,  107,  167, }, 
    { 107,  167,   91,  107,  107,  167, }, 
};

static const UInt8 
INIT_MVP_IDX[3][NUM_MVP_IDX_CTX] = 
{
    { CNU,  CNU, }, 
    { 168,  CNU, }, 
    { 168,  CNU, }, 
};

static const UInt8
INIT_TRANS_SUBDIV_FLAG[3][NUM_TRANS_SUBDIV_FLAG_CTX] = 
{
    { CNU,  224,  167,  122,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, }, 
    { CNU,  124,  138,   94,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, }, 
    { CNU,  153,  138,  138,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, }, 
};

// ***************************************************************************
// * Entropy Functions
// ***************************************************************************
void xCabacInitEntry( UInt n, const Int qp, UInt8 *pucState, const UInt8 *pInitValue )
{
    UInt i;
    assert( (qp >= 0) && (qp <= 51) );

    for( i=0; i<n; i++ ) 
	{
        Int initValue = pInitValue[i];
        // [9.2.1.1]
        Int  slopeIdx   = ( initValue >> 4);
        Int  intersecIdx= ( initValue & 15 );
        Int  m          = slopeIdx * 5 - 45;
        Int  n          = ( intersecIdx << 3 ) - 16;
        Int  initState  =  Clip3( 1, 126, ( ( ( m * qp ) >> 4 ) + n ) );
        UInt valMPS     = (initState >= 64 );
        pucState[i]     = ( (valMPS ? (initState - 64) : (63 - initState)) <<1) + valMPS;
    }
}

void xCabacInit( MTC265_t *h )
{
	MTC265_Cabac   *pCabac  = &h->cabac;
	const UInt          nSlice  = h->eSliceType;
	const Int           iQp     = h->iQP;
	UInt8        *pucState= pCabac->contextModels;
	UInt          nOffset = 0;

	if ( h->isIntra )
	{
	#define INIT_CABAC_I( n, m, v ) \
		xCabacInitEntry( (m)*(n), iQp, pucState, (v)[0] ); \
		pucState += (n)*(m); \
		nOffset  += (n)*(m);
		assert( nOffset == OFF_SPLIT_FLAG_CTX );
		INIT_CABAC_I( 1, NUM_SPLIT_FLAG_CTX,          INIT_SPLIT_FLAG         );
		assert( nOffset == OFF_SKIP_FLAG_CTX );
		INIT_CABAC_I( 1, NUM_SKIP_FLAG_CTX,           INIT_SKIP_FLAG          );
		assert( nOffset == OFF_ALF_CTRL_FLAG_CTX );
		INIT_CABAC_I( 1, NUM_ALF_CTRL_FLAG_CTX,       INIT_ALF_CTRL_FLAG      );
		assert( nOffset == OFF_MERGE_FLAG_EXT_CTX );
		INIT_CABAC_I( 1, NUM_MERGE_FLAG_EXT_CTX,      INIT_MERGE_FLAG_EXT     );
		assert( nOffset == OFF_MERGE_IDX_EXT_CTX );
		INIT_CABAC_I( 1, NUM_MERGE_IDX_EXT_CTX,       INIT_MERGE_IDX_EXT      );
		assert( nOffset == OFF_PART_SIZE_CTX );
		INIT_CABAC_I( 1, NUM_PART_SIZE_CTX,           INIT_PART_SIZE          );
		assert( nOffset == OFF_CU_AMP_CTX );
		INIT_CABAC_I( 1, NUM_CU_AMP_CTX,              INIT_CU_AMP_POS         );
		assert( nOffset == OFF_PRED_MODE_CTX );
		INIT_CABAC_I( 1, NUM_PRED_MODE_CTX,           INIT_PRED_MODE          );
		assert( nOffset == OFF_INTRA_PRED_CTX );
		INIT_CABAC_I( 1, NUM_ADI_CTX,                 INIT_INTRA_PRED_MODE    );
		assert( nOffset == OFF_CHROMA_PRED_CTX );
		INIT_CABAC_I( 1, NUM_CHROMA_PRED_CTX,         INIT_CHROMA_PRED_MODE   );
		assert( nOffset == OFF_INTER_DIR_CTX );
		INIT_CABAC_I( 1, NUM_INTER_DIR_CTX,           INIT_INTER_DIR          );
		assert( nOffset == OFF_MVD_CTX );
		INIT_CABAC_I( 1, NUM_MV_RES_CTX,              INIT_MVD                );
		assert( nOffset == OFF_REF_PIC_CTX );
		INIT_CABAC_I( 1, NUM_REF_NO_CTX,              INIT_REF_PIC            );
		assert( nOffset == OFF_DELTA_QP_CTX );
		INIT_CABAC_I( 1, NUM_DELTA_QP_CTX,            INIT_DQP                );
		assert( nOffset == OFF_QT_CBF_CTX );
		INIT_CABAC_I( 2, NUM_QT_CBF_CTX,              INIT_QT_CBF             );
		assert( nOffset == OFF_QT_ROOT_CBF_CTX );
		INIT_CABAC_I( 1, NUM_QT_ROOT_CBF_CTX,         INIT_QT_ROOT_CBF        );
		assert( nOffset == OFF_SIG_CG_FLAG_CTX );
		INIT_CABAC_I( 2, NUM_SIG_CG_FLAG_CTX,         INIT_SIG_CG_FLAG        );
		assert( nOffset == OFF_SIG_FLAG_CTX );
		INIT_CABAC_I( 1, NUM_SIG_FLAG_CTX,            INIT_SIG_FLAG           );
		assert( nOffset == OFF_LAST_X_CTX );
		INIT_CABAC_I( 2, NUM_LAST_FLAG_XY_CTX,        INIT_LAST               );
		assert( nOffset == OFF_LAST_Y_CTX );
		INIT_CABAC_I( 2, NUM_LAST_FLAG_XY_CTX,        INIT_LAST               );
		assert( nOffset == OFF_ONE_FLAG_CTX );
		INIT_CABAC_I( 1, NUM_ONE_FLAG_CTX,            INIT_ONE_FLAG           );
		assert( nOffset == OFF_ABS_FLAG_CTX );
		INIT_CABAC_I( 1, NUM_ABS_FLAG_CTX,            INIT_ABS_FLAG           );
		assert( nOffset == OFF_MVP_IDX_CTX );
		INIT_CABAC_I( 1, NUM_MVP_IDX_CTX,             INIT_MVP_IDX            );
		assert( nOffset == OFF_TRANS_SUBDIV_FLAG_CTX );
		INIT_CABAC_I( 1, NUM_TRANS_SUBDIV_FLAG_CTX,   INIT_TRANS_SUBDIV_FLAG  );

	#undef INIT_CABAC_I
	}
	else
	{
#define INIT_CABAC_P( n, m, v ) \
	xCabacInitEntry( (m)*(n), iQp, pucState, (v)[1] ); \
	pucState += (n)*(m); \
		nOffset  += (n)*(m);
		assert( nOffset == OFF_SPLIT_FLAG_CTX );
		INIT_CABAC_P( 1, NUM_SPLIT_FLAG_CTX,          INIT_SPLIT_FLAG         );
		assert( nOffset == OFF_SKIP_FLAG_CTX );
		INIT_CABAC_P( 1, NUM_SKIP_FLAG_CTX,           INIT_SKIP_FLAG          );
		assert( nOffset == OFF_ALF_CTRL_FLAG_CTX );
		INIT_CABAC_P( 1, NUM_ALF_CTRL_FLAG_CTX,       INIT_ALF_CTRL_FLAG      );
		assert( nOffset == OFF_MERGE_FLAG_EXT_CTX );
		INIT_CABAC_P( 1, NUM_MERGE_FLAG_EXT_CTX,      INIT_MERGE_FLAG_EXT     );
		assert( nOffset == OFF_MERGE_IDX_EXT_CTX );
		INIT_CABAC_P( 1, NUM_MERGE_IDX_EXT_CTX,       INIT_MERGE_IDX_EXT      );
		assert( nOffset == OFF_PART_SIZE_CTX );
		INIT_CABAC_P( 1, NUM_PART_SIZE_CTX,           INIT_PART_SIZE          );
		assert( nOffset == OFF_CU_AMP_CTX );
		INIT_CABAC_P( 1, NUM_CU_AMP_CTX,              INIT_CU_AMP_POS         );
		assert( nOffset == OFF_PRED_MODE_CTX );
		INIT_CABAC_P( 1, NUM_PRED_MODE_CTX,           INIT_PRED_MODE          );
		assert( nOffset == OFF_INTRA_PRED_CTX );
		INIT_CABAC_P( 1, NUM_ADI_CTX,                 INIT_INTRA_PRED_MODE    );
		assert( nOffset == OFF_CHROMA_PRED_CTX );
		INIT_CABAC_P( 1, NUM_CHROMA_PRED_CTX,         INIT_CHROMA_PRED_MODE   );
		assert( nOffset == OFF_INTER_DIR_CTX );
		INIT_CABAC_P( 1, NUM_INTER_DIR_CTX,           INIT_INTER_DIR          );
		assert( nOffset == OFF_MVD_CTX );
		INIT_CABAC_P( 1, NUM_MV_RES_CTX,              INIT_MVD                );
		assert( nOffset == OFF_REF_PIC_CTX );
		INIT_CABAC_P( 1, NUM_REF_NO_CTX,              INIT_REF_PIC            );
		assert( nOffset == OFF_DELTA_QP_CTX );
		INIT_CABAC_P( 1, NUM_DELTA_QP_CTX,            INIT_DQP                );
		assert( nOffset == OFF_QT_CBF_CTX );
		INIT_CABAC_P( 2, NUM_QT_CBF_CTX,              INIT_QT_CBF             );
		assert( nOffset == OFF_QT_ROOT_CBF_CTX );
		INIT_CABAC_P( 1, NUM_QT_ROOT_CBF_CTX,         INIT_QT_ROOT_CBF        );
		assert( nOffset == OFF_SIG_CG_FLAG_CTX );
		INIT_CABAC_P( 2, NUM_SIG_CG_FLAG_CTX,         INIT_SIG_CG_FLAG        );
		assert( nOffset == OFF_SIG_FLAG_CTX );
		INIT_CABAC_P( 1, NUM_SIG_FLAG_CTX,            INIT_SIG_FLAG           );
		assert( nOffset == OFF_LAST_X_CTX );
		INIT_CABAC_P( 2, NUM_LAST_FLAG_XY_CTX,        INIT_LAST               );
		assert( nOffset == OFF_LAST_Y_CTX );
		INIT_CABAC_P( 2, NUM_LAST_FLAG_XY_CTX,        INIT_LAST               );
		assert( nOffset == OFF_ONE_FLAG_CTX );
		INIT_CABAC_P( 1, NUM_ONE_FLAG_CTX,            INIT_ONE_FLAG           );
		assert( nOffset == OFF_ABS_FLAG_CTX );
		INIT_CABAC_P( 1, NUM_ABS_FLAG_CTX,            INIT_ABS_FLAG           );
		assert( nOffset == OFF_MVP_IDX_CTX );
		INIT_CABAC_P( 1, NUM_MVP_IDX_CTX,             INIT_MVP_IDX            );
		assert( nOffset == OFF_TRANS_SUBDIV_FLAG_CTX );
		INIT_CABAC_P( 1, NUM_TRANS_SUBDIV_FLAG_CTX,   INIT_TRANS_SUBDIV_FLAG  );

#undef INIT_CABAC_P
	}
	assert( nOffset < MAX_NUM_CTX_MOD );
}

void xCabacReset( MTC265_Cabac *pCabac )
{
    pCabac->uiLow         = 0;
    pCabac->uiRange       = 510;
    pCabac->iBitsLeft     = 23;
    pCabac->ucCache       = 0xFF;
    pCabac->uiNumBytes    = 0;
}

void xCabacFlush( MTC265_Cabac *pCabac, MTC265_BitStream *pBS )
{
    CABAC_ENTER;
    if ( uiLow >> (32 - iBitsLeft) ) 
	{
        WRITE_CODE( ucCache + 1, 8, "xCabacFlush0" );
        while( uiNumBytes > 1 )
		{
            WRITE_CODE( 0x00, 8, "xCabacFlush1" );
            uiNumBytes--;
        }
        uiLow -= 1 << ( 32 - iBitsLeft );
    }
    else  
	{
        if ( uiNumBytes > 0 ) 
		{
            WRITE_CODE( ucCache, 8, "xCabacFlush2" );
        }
        while ( uiNumBytes > 1 ) 
		{
            WRITE_CODE( 0xFF, 8, "xCabacFlush3" );
            uiNumBytes--;
        }    
    }
    WRITE_CODE( uiLow >> 8, 24 - iBitsLeft, "xCabacFlush4" );
    CABAC_LEAVE;
}

UInt xCabacGetNumWrittenBits( MTC265_Cabac *pCabac, MTC265_BitStream *pBS )
{
    Int32 iLen = xBitFlush(pBS);
    return iLen + 8 * pCabac->uiNumBytes + 23 - pCabac->iBitsLeft;
}

#define GetMPS( state )     ( (state) &  1 )
#define GetState( state )   ( (state) >> 1 )
#define UpdateLPS( state )  ( (state) = g_aucNextStateLPS[ (state) ] )
#define UpdateMPS( state )  ( (state) = g_aucNextStateMPS[ (state) ] )

static void testAndWriteOut( MTC265_Cabac *pCabac, MTC265_BitStream *pBS )
{
    CABAC_ENTER;
    while( iBitsLeft < 12 ) 
	{
        UInt leadByte = uiLow >> (24 - iBitsLeft);
        iBitsLeft += 8;
        uiLow     &= 0xFFFFFFFF >> iBitsLeft;

        if ( leadByte == 0xff ) 
		{
            uiNumBytes++;
        }
        else 
		{
            if ( uiNumBytes > 0 ) 
			{
                UInt carry = leadByte >> 8;
                UInt byte = ucCache + carry;
                ucCache = leadByte & 0xff;
                WRITE_CODE( byte, 8, "testAndWriteOut0" );

                byte = ( 0xff + carry ) & 0xff;
                while ( uiNumBytes > 1 ) 
				{
                    WRITE_CODE( byte, 8, "testAndWriteOut0" );
                    uiNumBytes--;
                }
            }
            else 
			{
                uiNumBytes = 1;
                ucCache = leadByte;
            }
        }
    }
    CABAC_LEAVE;
}

void xCabacEncodeBin( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue, UInt nCtxState )
{
    CABAC_ENTER;
    UInt8 ucState = pCabac->contextModels[nCtxState];
    UInt  uiLPS   = g_aucLPSTable[ GetState( ucState ) ][ ( uiRange >> 6 ) & 3 ];
    uiRange    -= uiLPS;
  
    if( binValue != GetMPS(ucState) ) 
	{
        Int numBits = g_aucRenormTable[ uiLPS >> 3 ];
        uiLow     = ( uiLow + uiRange ) << numBits;
        uiRange   = uiLPS << numBits;
        UpdateLPS( ucState );
        
        iBitsLeft -= numBits;
    }
    else 
	{
        UpdateMPS( ucState );
        if ( uiRange < 256 ) 
		{
            uiLow <<= 1;
            uiRange <<= 1;
            iBitsLeft--;
        }
    }
    
    pCabac->contextModels[nCtxState] = ucState;
    CABAC_LEAVE;

    testAndWriteOut( pCabac, pBS );
}

void xCabacEncodeBinEP( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue )
{
    CABAC_ENTER;

    uiLow <<= 1;
    if( binValue ) 
	{
        uiLow += uiRange;
    }
    iBitsLeft--;
    CABAC_LEAVE;

    testAndWriteOut( pCabac, pBS );
}

void xCabacEncodeBinsEP( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValues, Int numBins )
{
    while ( numBins > 8 ) 
	{
        numBins -= 8;
        UInt pattern = binValues >> numBins; 
        pCabac->uiLow <<= 8;
        pCabac->uiLow += pCabac->uiRange * pattern;
        binValues -= pattern << numBins;
        pCabac->iBitsLeft -= 8;

        testAndWriteOut( pCabac, pBS );
    }
    
    pCabac->uiLow <<= numBins;
    pCabac->uiLow  += pCabac->uiRange * binValues;
    pCabac->iBitsLeft -= numBins;

    testAndWriteOut( pCabac, pBS );
}

void xCabacEncodeTerminatingBit( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue )
{
    CABAC_ENTER;

    uiRange -= 2;
    if( binValue ) 
	{
        uiLow  += uiRange;
        uiLow <<= 7;
        uiRange = 2 << 7;
        iBitsLeft -= 7;
    }

    else if ( uiRange < 256 ) 
	{
        uiLow   <<= 1;
        uiRange <<= 1;
        iBitsLeft--;    
    }

    CABAC_LEAVE;
    testAndWriteOut( pCabac, pBS );
}

void xWriteEpExGolomb( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiSymbol, UInt uiCount )
{
    UInt bins = 0;
    Int numBins = 0;
  
    while( uiSymbol >= (UInt)(1<<uiCount) ) 
	{
        bins = 2 * bins + 1;
        numBins++;
        uiSymbol -= 1 << uiCount;
        uiCount  ++;
    }
    bins = 2 * bins + 0;
    numBins++;
  
    bins = (bins << uiCount) | uiSymbol;
    numBins += uiCount;
  
    assert( numBins <= 32 );
    xCabacEncodeBinsEP( pCabac, pBS, bins, numBins );
}

void xWriteGoRiceExGolomb( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt uiSymbol, UInt &ruiGoRiceParam )
{
    UInt uiMaxVlc     = g_auiGoRiceRange[ ruiGoRiceParam ];
    UInt bExGolomb    = ( uiSymbol > uiMaxVlc );
    UInt uiCodeWord   = MIN( uiSymbol, ( uiMaxVlc + 1 ) );
    UInt uiQuotient   = uiCodeWord >> ruiGoRiceParam;
    UInt uiMaxPreLen  = g_auiGoRicePrefixLen[ ruiGoRiceParam ];
    
    UInt binValues;
    Int numBins;
    
    if ( uiQuotient >= uiMaxPreLen ) 
	{
        numBins = uiMaxPreLen;
        binValues = ( 1 << numBins ) - 1;
    }
    else 
	{
        numBins = uiQuotient + 1;
        binValues = ( 1 << numBins ) - 2;
    }
    
    xCabacEncodeBinsEP( pCabac, pBS, ( binValues << ruiGoRiceParam ) + uiCodeWord - ( uiQuotient << ruiGoRiceParam ), numBins + ruiGoRiceParam );
    
    ruiGoRiceParam = g_aauiGoRiceUpdate[ruiGoRiceParam][MIN(uiSymbol, 23)];
    
    if( bExGolomb ) 
	{
        uiSymbol -= uiMaxVlc + 1;
        xWriteEpExGolomb( pCabac, pBS, uiSymbol, 0 );
    }
}
// for inter
void xWriteUnarySymbol( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue, UInt nCtxState, Int iOffset )
{
	xCabacEncodeBin( pCabac, pBS, binValue ? 1 : 0, nCtxState );

	if( 0 == binValue )
	{
		return;
	}

	while( binValue-- )
	{
		xCabacEncodeBin( pCabac, pBS, binValue ? 1 : 0, nCtxState + iOffset );
	}

	return;
}

void xWriteUnaryMaxSymbol( MTC265_Cabac *pCabac, MTC265_BitStream *pBS, UInt binValue, UInt nCtxState, Int iOffset, UInt uiMaxSymbol )
{
	if (uiMaxSymbol == 0)
	{
		return;
	}

	xCabacEncodeBin( pCabac, pBS, binValue ? 1 : 0, nCtxState );

	if ( binValue == 0 )
	{
		return;
	}

	bool bCodeLast = ( uiMaxSymbol > binValue );

	while( --binValue )
	{
		xCabacEncodeBin( pCabac, pBS, 1, nCtxState + iOffset );
	}
	if( bCodeLast )
	{
		xCabacEncodeBin( pCabac, pBS, 0, nCtxState + iOffset );
	}

	return;
}
