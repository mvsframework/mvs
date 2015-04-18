// 
//  Copyright © 2015 Claus Christmann <hcc |ä| gatech.edu>.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// 


#include "GustInterface/checksum.h"
#include "GustInterface/datalinkmessages.h"
#include "GustInterface/3rdParty/onboard_ref.h"
// #include "GustInterface/3rdParty/onboard.h"

namespace GUST {

/**
 * \brief Calculate and return the checksum of a buffer array
 *
 * Calculates the checksum of a character buffer using a (modified) 32-bit 
 * Fletcher checksum. (The modification allows the handling of an odd number of
 * bytes by calculating the checksum as ifthere were an additional zero-byte 
 * appended to the end of the data.)
 * 
 * The code of this function is copied from the corresponding sources in GUST.
 *
 * \param buf pointer to a character buffer array
 * \param byteCount the size in bytes of the character buffer
 */
unsigned int datalinkCheckSumCompute( unsigned char *buf, int byteCount ) {

    unsigned int sum1 = 0xffff;
    unsigned int sum2 = 0xffff;
    unsigned int tlen = 0;
    unsigned int shortCount = byteCount / sizeof(short);
    unsigned int oddLength  = byteCount % 2;

    /* this is Fletcher32 checksum modified to handle buffers with an odd number of bytes */

    while( shortCount ) {
		/* 360 is the largest number of sums that can be performed without overflow */
        tlen = shortCount > 360 ? 360 : shortCount;
        shortCount -= tlen;
        do {
            sum1 +=  *buf++;
            sum1 += (*buf++ << 8);
            sum2 += sum1;
        } while (--tlen);

        /* add last byte if there's an odd number of bytes (equivalent to appending a zero-byte) */
        if( (oddLength==1) && (shortCount<1) ) {
            sum1 += *buf++;
            sum2 += sum1;
        }

        sum1 = (sum1 & 0xffff) + (sum1 >> 16);
        sum2 = (sum2 & 0xffff) + (sum2 >> 16);
    }

    /* Second reduction step to reduce sums to 16 bits */
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);

    return( sum2 << 16 | sum1 );
}


/**
 * \brief datalinkCheckSumEncode sets the header checksum and payload checksum of a
 * character buffer to be sent as a datalink message
 *
 * \param buf pointer to a character buffer
 * \param byteCount size of the character buffer in bytes
 */
void datalinkCheckSumEncode( unsigned char *buf, int byteCount ) {

    struct datalinkHeader_ref *h = (struct datalinkHeader_ref *)buf;

    h->hcsum = datalinkCheckSumCompute(  buf, sizeof( struct datalinkHeader_ref ) - sizeof( int )*2 );
    h->csum  = datalinkCheckSumCompute( &(buf[sizeof( struct datalinkHeader_ref )]),
                                  byteCount - sizeof( struct datalinkHeader_ref ) );

}


/**
 * @brief datalinkCheckSumComputeOld() legacy 8-bit XOR checksum
 *
 * @param buf pointer to a character buffer
 * @param byteCount size of the character buffer in bytes
 */
unsigned char datalinkCheckSumComputeOld( unsigned char *buf, int byteCount ) {

  int m;
  unsigned char csum;

  csum = 0;
  for( m=0; m<byteCount; m++ )
    csum ^= buf[m];

  return( csum );

}


/**
 * @brief datalinkCheckSumEncodeOld() legacy checksum encode
 *
 * @param buf pointer to a character buffer
 * @param byteCount size of the character buffer in bytes
 */
void datalinkCheckSumEncodeOld( unsigned char *buf, int byteCount ) {

    struct datalinkHeaderOld_ref *h = (struct datalinkHeaderOld_ref *)buf;

    h->csum  = 0;
    h->hcsum = 0;
    h->csum  = datalinkCheckSumComputeOld( buf, byteCount );
    h->hcsum = datalinkCheckSumComputeOld( buf, sizeof( struct datalinkHeaderOld_ref ) );

}

} //end namespace esim