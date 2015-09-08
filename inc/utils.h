#ifndef __UTILS_H__
#define __UTILS_H__

static int xLog2(UInt32 x)
{
    UInt32 log2Size = 0;
    while(x > 0) 
	{
        x >>= 1;
        log2Size++;
    }
    return(log2Size);
}

static Int32 Clip3( Int32 minVal, Int32 maxVal, Int32 a )
{
    if ( a < minVal )
        a = minVal;
    else if ( a > maxVal )
        a = maxVal;
    return a;
}

static double Clip3( double minVal, double maxVal, double a )
{
	if ( a < minVal )
		a = minVal;
	else if ( a > maxVal )
		a = maxVal;
	return a;
}

#define Clip(x)         Clip3( 0, 255, (x))
#define xSHR(x, n)      ( (n)>=32 ? 0 : ((x)>>(n)) )
#define xSHL(x, n)      ( (n)>=32 ? 0 : ((x)<<(n)) )
#define MAX(a, b)       ( (a) > (b) ? (a) : (b) )
#define MIN(a, b)       ( (a) < (b) ? (a) : (b) )
#define MALLOC(n)       malloc(n)
#define FREE(p)         free(p)
#define ASIZE(x)        ( sizeof(x)/sizeof((x)[0]) )

#endif /* __UTILS_H__ */
