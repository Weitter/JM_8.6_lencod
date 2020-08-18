
/*!
 *************************************************************************************
 * \file biariencode.c
 *
 * \brief
 *    Routines for binary arithmetic encoding
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Detlev Marpe                    <marpe@hhi.de>
 *    - Gabi Blaettermann               <blaetter@hhi.de>
 *************************************************************************************
 */
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "biariencode.h"
#include <assert.h>

/*!
 ************************************************************************
 * Macro for writing bytes of code
 ***********************************************************************
 */

#define put_byte() { \
                     Ecodestrm[(*Ecodestrm_len)++] = Ebuffer; \
                     Ebits_to_go = 8; \
                     while (eep->C > 7) { \
                       eep->C-=8; \
                       eep->E++; \
                     } \
                    } 

#define put_one_bit(b) { \
                         Ebuffer <<= 1; Ebuffer |= (b); \
                         if (--Ebits_to_go == 0) \
                           put_byte(); \
                       }

#define put_one_bit_plus_outstanding(b) { \
                                          put_one_bit(b); \
                                          while (Ebits_to_follow > 0) \
                                          { \
                                            Ebits_to_follow--; \
                                            put_one_bit(!(b)); \
                                          } \
                                         }


/*!
 ************************************************************************
 * \brief
 *    Allocates memory for the EncodingEnvironment struct
 ************************************************************************
 */
EncodingEnvironmentPtr arienco_create_encoding_environment()
{
  EncodingEnvironmentPtr eep;

  if ( (eep = (EncodingEnvironmentPtr) calloc(1,sizeof(EncodingEnvironment))) == NULL)
    no_mem_exit("arienco_create_encoding_environment: eep");

  return eep;
}



/*!
 ************************************************************************
 * \brief
 *    Frees memory of the EncodingEnvironment struct
 ************************************************************************
 */
void arienco_delete_encoding_environment(EncodingEnvironmentPtr eep)
{
  if (eep == NULL)
  {
    snprintf(errortext, ET_SIZE, "Error freeing eep (NULL pointer)");
    error (errortext, 200);
  }
  else
    free(eep);
}



/*!
 ************************************************************************
 * \brief
 *    Initializes the EncodingEnvironment for the arithmetic coder
 ************************************************************************
 */
void arienco_start_encoding(EncodingEnvironmentPtr eep,
                            unsigned char *code_buffer,
                            int *code_len, /* int *last_startcode, */ int slice_type )
{
  Elow = 0;
  Ebits_to_follow = 0;
  Ebuffer = 0;
  Ebits_to_go = 9; // to swallow first redundant bit

  Ecodestrm = code_buffer;
  Ecodestrm_len = code_len;
//  Ecodestrm_laststartcode = last_startcode;

  Erange = HALF-2;

  eep->C = 0;
  eep->B = *code_len;
  eep->E = 0;

}

/*!
 ************************************************************************
 * \brief
 *    Returns the number of currently written bits
 ************************************************************************
 */
int arienco_bits_written(EncodingEnvironmentPtr eep)
{
   return (8 * (*Ecodestrm_len /*-*Ecodestrm_laststartcode*/) + Ebits_to_follow + 8  - Ebits_to_go);
}


/*!
 ************************************************************************
 * \brief
 *    Terminates the arithmetic codeword, writes stop bit and stuffing bytes (if any)
 ************************************************************************
 */
void arienco_done_encoding(EncodingEnvironmentPtr eep)
{
  put_one_bit_plus_outstanding((Elow >> (B_BITS-1)) & 1);
  put_one_bit((Elow >> (B_BITS-2))&1);
  put_one_bit(1);

  stat->bit_use_stuffingBits[img->type]+=(8-Ebits_to_go);

  while (Ebits_to_go != 8)
    put_one_bit(0);

  eep->E= eep->E*8 + eep->C; // no of processed bins
  eep->B= (*Ecodestrm_len - eep->B); // no of written bytes
  eep->E -= (img->current_mb_nr-img->currentSlice->start_mb_nr);
  eep->E = (eep->E + 31)>>5;
  // eep->E now contains the minimum number of bytes for the NAL unit
}


/*!
 ************************************************************************
 * \brief
 *    Actually arithmetic encoding of one binary symbol by using
 *    the probability estimate of its associated context model
 ************************************************************************
 */
 //bi_ct就是概率状态索引 和 MPS的符号
void biari_encode_symbol(EncodingEnvironmentPtr eep, signed short symbol, BiContextTypePtr bi_ct )
{
  register unsigned int range = Erange;//R 区间范围
  register unsigned int low = Elow;//L 区间起点
  unsigned int rLPS = rLPS_table_64x4[bi_ct->state][(range>>6) & 3];//概率区间估计表得到Rlps

  extern int cabac_encoding;

  if( cabac_encoding )
  {
    bi_ct->count++;
  }

  /* covers all cases where code does not bother to shift down symbol to be 
   * either 0 or 1, e.g. in some cases for cbp, mb_Type etc the code symply 
   * masks off the bit position and passes in the resulting value */

  if (symbol != 0) //非0即1
    symbol = 1;
  
  range -= rLPS;//得到Rmps 大概率的范围
  if (symbol != bi_ct->MPS) //当前的符号不是预计的大概率符号 而是小概率符号
  {
    low += range;//区间下限改变
    range = rLPS;//区间范围改变
    
    if (!bi_ct->state)//索引值为0 当前符号任然是小概率 则需要调换MPS和LPS的值
      bi_ct->MPS = bi_ct->MPS ^ 1;               // switch LPS if necessary
    bi_ct->state = AC_next_state_LPS_64[bi_ct->state]; // next state 查表得下一个索引
  } 
  else //当前符号是大概率的MPS
    bi_ct->state = AC_next_state_MPS_64[bi_ct->state]; // next state  查表得下一个索引
 

  /* renormalisation重新正常化 */    
  while (range < QUARTER)
  {
    if (low >= HALF)
    {
      put_one_bit_plus_outstanding(1);
      low -= HALF;
    }
    else 
      if (low < QUARTER)
      {
        put_one_bit_plus_outstanding(0);
      }
      else
      {
        Ebits_to_follow++;
        low -= QUARTER;
      }
    low <<= 1;
    range <<= 1;
  }
  Erange = range;
  Elow = low;
  eep->C++;

}




/*!
 ************************************************************************
 * \brief
 *    Arithmetic encoding of one binary symbol assuming 
 *    a fixed prob. distribution with p(symbol) = 0.5
 ************************************************************************
 */
void biari_encode_symbol_eq_prob(EncodingEnvironmentPtr eep, signed short symbol)
{
  register unsigned int low = (Elow<<1);
  
  if (symbol != 0)
    low += Erange;

  /* renormalisation as for biari_encode_symbol; 
     note that low has already been doubled */ 
  if (low >= ONE)
  {
    put_one_bit_plus_outstanding(1);
    low -= ONE;
  }
  else 
    if (low < HALF)
    {
      put_one_bit_plus_outstanding(0);
    }
    else
    {
      Ebits_to_follow++;
      low -= HALF;
    }
    Elow = low;
    eep->C++;
    
}

/*!
 ************************************************************************
 * \brief
 *    Arithmetic encoding for last symbol before termination
 ************************************************************************
 */
void biari_encode_symbol_final(EncodingEnvironmentPtr eep, signed short symbol)
{
  register unsigned int range = Erange-2;
  register unsigned int low = Elow;
  
  if (symbol) {//symbol=1
    low += range;
    range = 2;
  }
  
  while (range < QUARTER)
  {
    if (low >= HALF)
    {
      put_one_bit_plus_outstanding(1);
      low -= HALF;
    }
    else 
      if (low < QUARTER)
      {
        put_one_bit_plus_outstanding(0);
      }
      else
      {
        Ebits_to_follow++;
        low -= QUARTER;
      }
      low <<= 1;
      range <<= 1;
  }
  Erange = range;
  Elow = low;
  eep->C++;
}



/*!
 ************************************************************************
 * \brief
 *    Initializes a given context with some pre-defined probability state
 ************************************************************************
 */
void biari_init_context (BiContextTypePtr ctx, const int* ini)
{
  int pstate;

  pstate = ((ini[0]*img->qp)>>4) + ini[1];
  pstate = min (max ( 1, pstate), 126);

  if ( pstate >= 64 )
  {
    ctx->state  = pstate - 64;
    ctx->MPS    = 1;
  }
  else
  {
    ctx->state  = 63 - pstate;
    ctx->MPS    = 0;
  }
  
  ctx->count = 0;
}

