#include "MTC265.h"
#include "version.h"

#include <time.h>
#include <math.h>

#define MTC265_VERSION "MTC-2013-5-27"
#define MAX_OUTBUF_SIZE     (1 * 1024 * 1024)
UInt8 buf[MAX_WIDTH * MAX_HEIGHT * 3 / 2];
UInt  totalbits = 0;
double eBit[4];

int main( int argc, char *argv[] )
{
    MTC265_t h;
    UInt8 *pucOutBuf0 = (UInt8 *)MALLOC(MAX_OUTBUF_SIZE);
    UInt8 *pucOutBuf1 = (UInt8 *)MALLOC(MAX_OUTBUF_SIZE);
    assert( pucOutBuf0 != NULL );
    assert( pucOutBuf1 != NULL );
    int i;
    char   *inFile   = NULL;
    char   *outFile  = NULL;
    UInt    nWidth   = 352;
    UInt    nHeight  = 288;
    UInt    nFrames  = 1;
    Int     iQP      = 32;
    UInt    bLMChroma= FALSE;

	bool    g_bSeqFirst;
	UInt    GOPSize  = 1;
	Int     IntraPeriod = 1;
	UInt    FrameRate = 30;
	bool    g_enableLoopFilter = FALSE;
	bool    g_enableWriteReconFile = FALSE;
	//for rate control
	bool    g_enableRateCtrl = FALSE;
	UInt    TargetBitrate = 1000;

    fprintf( stdout, "\n" );
    fprintf( stdout, "MTC265 Version [%s] ", MTC265_VERSION );
    fprintf( stdout, "Built on [%s]", __DATE__ );
    fprintf( stdout, MTC265_ONOS );
    fprintf( stdout, MTC265_BITS );
    fprintf( stdout, MTC265_COMPILEDBY );
    fprintf( stdout, "\n\n" );

    if ( argc < 2 ) 
	{
        fprintf( stderr, "Usage:\n\t%s -i inYuvFile [-o hevcBin] [-w width] [-h height] [-f frames] [-q QP] [-lm] [-g GOPSize] [-ip IntraPeriod] [-lp][-wr][-fps FrameRate] [-rc] [-tbr TargetBitrate]\n", argv[0] );
        return 0;
    }
    for( i=1; i<argc; i++ ) 
	{
        if ( !strcmp(argv[i], "-i") ) 
		{
            inFile = argv[++i];
        }
        else if ( !strcmp(argv[i], "-o") ) 
		{
            outFile = argv[++i];
        }
        else if ( !strcmp(argv[i], "-w") ) 
		{
            nWidth = atoi(argv[++i]);
        }
        else if ( !strcmp(argv[i], "-h") ) 
		{
            nHeight = atoi(argv[++i]);
        }
        else if ( !strcmp(argv[i], "-f") ) 
		{
            nFrames = atoi(argv[++i]);
        }
        else if ( !strcmp(argv[i], "-q") ) 
		{
            iQP = atoi(argv[++i]);
        }
        else if ( !strcmp(argv[i], "-lm") )
		{
            bLMChroma = TRUE;
        }
		else if ( !strcmp(argv[i], "-g") ) 
		{
            GOPSize = atoi(argv[++i]);
        }
		else if ( !strcmp(argv[i], "-ip") ) 
		{
            IntraPeriod = atoi(argv[++i]);
        }
		else if ( !strcmp(argv[i], "-lp") )
		{
            g_enableLoopFilter = TRUE;
        }
		else if ( !strcmp(argv[i], "-wr") )
		{
            g_enableWriteReconFile = TRUE;
        }
		else if ( !strcmp(argv[i], "-fps") )
		{
            FrameRate = atoi(argv[++i]);
        }
	    else if ( !strcmp(argv[i], "-rc") ) 
		{
            g_enableRateCtrl = TRUE;
        }
	    else if ( !strcmp(argv[i], "-tbr") ) 
		{
            TargetBitrate = atoi(argv[++i]);
        }
    }

    xDefaultParams(&h);
    h.usWidth       = nWidth;
    h.usHeight      = nHeight;
	if(h.usWidth == 352 && h.usHeight ==288)
	{  
		eBit[0] = 0 ;
		eBit[1] = 0 ;
		eBit[2] = 1 ;
		eBit[3] = 1 ;
	}
	else if(h.usWidth == 832 && h.usHeight ==480)
	{ 
		eBit[0] = 0 ; 
		eBit[1] = 0 ;
		eBit[2] = 0.5 ;
		eBit[3] = 0.5 ;
	}
	else if(h.usWidth == 1280 && h.usHeight ==720)
	{ 
		eBit[0] = 0 ; 
		eBit[1] = 0 ;
		eBit[2] = 0.5 ;
		eBit[3] = 0.5 ;
	}
	else
	{ 
		eBit[0] = 1 ;
		eBit[1] = 1 ;
		eBit[2] = 2 ;
		eBit[3] = 2 ;
	}

    h.ucMaxCUWidth  = 32;
    h.ucMaxCUDepth  =  3;
	h.ucMinCUWidth  = h.ucMaxCUWidth>>h.ucMaxCUDepth;
    h.NumPartition  = (h.ucMaxCUWidth/4)*(h.ucMaxCUWidth/4); //1<<(h.ucMaxCUDepth<<1);
    h.eSliceType    = SLICE_I;
    h.iQP           = iQP;
	double lamada_scale = Clip3((double)0.5, (double)1, (double)(1-0.05*(GOPSize-1)));
	h.Inter_lamada  = lamada_scale * pow((double)2,(double)(iQP-12)/3);
	h.sqrt_Inter_lamada = sqrt(h.Inter_lamada);
    h.bUseLMChroma  = bLMChroma;
	h.bUseLoopFilter = g_enableLoopFilter;
	h.bWriteReconFile = g_enableWriteReconFile;

	h.GOPSize       = GOPSize;
	h.IntraPeriod   = IntraPeriod;
	h.FrameRate     = FrameRate;

	//calculate padding size
	h.PAD_YSIZEw = nWidth%h.ucMaxCUWidth == 0 ? 0 : h.ucMaxCUWidth - nWidth%h.ucMaxCUWidth;
	h.PAD_YSIZEh = nHeight%h.ucMaxCUWidth == 0 ? 0 : h.ucMaxCUWidth - nHeight%h.ucMaxCUWidth;
	h.PAD_CSIZEw = (h.PAD_YSIZEw)>>1;
	h.PAD_CSIZEh = (h.PAD_YSIZEh)>>1;
	Pxl *ptr;
	ptr = (Pxl *)MALLOC(sizeof(Pxl)*(nWidth+h.PAD_YSIZEw )*(nHeight+h.PAD_YSIZEh) * 3 / 2);
	assert( ptr != NULL );
    xCheckParams(&h);

    const UInt32  nYSize = nWidth*nHeight;
    const UInt32  nImageSize = nYSize*3/2;

    xEncInit( &h, nFrames );

    FILE *fpi = fopen( inFile, "rb");
    FILE *fpo = fopen( outFile, "wb");
    assert( fpi != NULL );
    assert( fpo != NULL );

	double totaltime = 0;

	for( i=0; i<(Int32)nFrames; i++ ) 
	{
		MTC265_Frame frame;
		if(nWidth%h.ucMaxCUWidth == 0&&nHeight%h.ucMaxCUWidth == 0)
		{      
			fread( buf, 1, nImageSize, fpi );
			xFramePadding(&h, &frame, buf ,ptr);
		}
		else
		{  
			assert(ptr != NULL);
			fread( buf, 1, nImageSize, fpi );
			xFramePadding(&h, &frame, buf ,ptr);
		}
		printf("Encoding Frame[%3d]  ", i);

		if( i==0 )
		{
			g_bSeqFirst = TRUE;
		}
		else
		{
            g_bSeqFirst = FALSE;
		}

		long iBeforeTime = clock();

		if( ( g_bSeqFirst == FALSE ) && ( g_enableRateCtrl == TRUE ) )
		{
			if( 0.001 * totalbits * h.FrameRate / i > TargetBitrate )
			{
			    h.iQP ++;
			}

			if( 0.001 * totalbits * h.FrameRate / i < TargetBitrate )
			{
				h.iQP --;
			}
		}

	    printf( " < QP %d > ", h.iQP );

        Int32 iEncSize = xEncEncode( &h, &frame, pucOutBuf1, MAX_OUTBUF_SIZE, g_bSeqFirst );

		//calculate PSNR
		xCalculateAddPSNR( &h, i );

	    double dEncTime = (double)(clock()-iBeforeTime);
		printf(" [ET %5.0fms ]\n", dEncTime );
		totaltime += dEncTime;
        fwrite( pucOutBuf1, 1, iEncSize, fpo );
    }
	//PSNR 求平均
	double M_psnrY = 0;
	double M_psnrU = 0;
	double M_psnrV = 0;
	for( i=0; i<(Int32)nFrames; i++ )
	{
		M_psnrY += h.FpsnrY[i];
	    M_psnrU += h.FpsnrU[i];
	    M_psnrV += h.FpsnrV[i];
	}
	M_psnrY = M_psnrY/nFrames;
	M_psnrU = M_psnrU/nFrames;
	M_psnrV = M_psnrV/nFrames;
	printf( " M-Y-PSNR    "  "M-U-PSNR    "  "M-V-PSNR \n" );
    printf( "%8.4lf  "   "%8.4lf  "    "%8.4lf\n",M_psnrY,M_psnrU,M_psnrV  );
	//PSNR求平均end
	printf("\n");
    printf("TotalTime: %5.0f ms\n", totaltime);
	printf("Bytes written to file: %u (%.3f kbps)\n", totalbits / 8, 0.001 * totalbits * h.FrameRate / nFrames );

    FREE(pucOutBuf0);
    FREE(pucOutBuf1);
    fclose(fpi);
    fclose(fpo);
    xEncFree(&h);
	if(h.PAD_YSIZEw!=0 || h.PAD_YSIZEh!=0)
	{
		free(ptr);
		ptr = NULL;
	}
    printf("\nAll Done!\n");
    return 0;
}
