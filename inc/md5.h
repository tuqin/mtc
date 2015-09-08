#ifndef __MD5_H__
#define __MD5_H__

#include "MTC265.h"

typedef struct MD5Context {
    UInt32 buf[4];
    UInt32 bits[2];
    unsigned char in[64];
} MD5Context;

void MD5Init(MD5Context *context);
void MD5Update(MD5Context *context, unsigned char *buf, UInt32 len);
void MD5Final(MD5Context *ctx, UInt8 *digest);
void MD5Transform(UInt32 *buf, UInt32 *in);

#endif /* __MD5_H__ */
