
/*!
 ***************************************************************************
 * \file vlc.c
 *
 * \brief
 *    (CA)VLC coding functions
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Inge Lille-Lang�y               <inge.lille-langoy@telenor.com>
 *    - Detlev Marpe                    <marpe@hhi.de>
 *    - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 ***************************************************************************
 */

#include "contributors.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "elements.h"
#include "vlc.h"

#if TRACE
#define SYMTRACESTRING(s) strncpy(sym.tracestring,s,TRACESTRING_SIZE)
#else
#define SYMTRACESTRING(s) // do nothing
#endif


/*! 
 *************************************************************************************
 * \brief
 *    ue_v, writes an ue(v) syntax element, returns the length in bits
 *
 * \param tracestring
 *    the string for the trace file
 * \param value
 *    the value to be coded
 *  \param part
 *    the Data Partition ���洢����ı�������the value should be coded into
 *
 * \return
 *    Number of bits used by the coded syntax element
 *
 * \ note
 *    This function writes always the bit buffer for the progressive scan flag, and
 *    should not be used (or should be modified appropriately) for the interlace crap
 *    When used in the context of the Parameter Sets, this is obviously not a
 *    problem.
 *
 *************************************************************************************
 */
int ue_v (char *tracestring, int value, DataPartition *part)
{
	//1.��value ��װ���﷨Ԫ������ 
	//2.ָ��ʹ��ue_linfo����(�޷���0��ָ�����ײ�����) �õ�len info
	//3.��ʹ��symbol2ulvc�����õ�[MZeros][1][INFO]��bitpattern
	//4.д�뵽streamBuffer��
  SyntaxElement symbol, *sym=&symbol;
  sym->type = SE_HEADER;
  sym->mapping = ue_linfo;               // ����ָ�� ָ��0��ָ�����ײ����뺯��Mapping rule: unsigned integer
  sym->value1 = value; //�����﷨Ԫ�ص�����ֵ
  sym->value2 = 0;
#if TRACE
  strncpy(sym->tracestring,tracestring,TRACESTRING_SIZE);
#endif
  assert (part->bitstream->streamBuffer != NULL);
  return writeSyntaxElement_UVLC (sym, part);//���﷨Ԫ�ص�����ֵӳ��ɱ�����ʽ����д������Ƭ��
}


/*! 
 *************************************************************************************
 * \brief
 *    se_v, writes an se(v) syntax element, returns the length in bits
 *
 * \param tracestring
 *    the string for the trace file
 * \param value
 *    the value to be coded
 *  \param part
 *    the Data Partition the value should be coded into
 *
 * \return
 *    Number of bits used by the coded syntax element
 *
 * \ note
 *    This function writes always the bit buffer for the progressive scan flag, and
 *    should not be used (or should be modified appropriately) for the interlace crap
 *    When used in the context of the Parameter Sets, this is obviously not a
 *    problem.
 *
 *************************************************************************************
 */
int se_v (char *tracestring, int value, DataPartition *part)
{
  SyntaxElement symbol, *sym=&symbol;
  sym->type = SE_HEADER;
  sym->mapping = se_linfo;               //����ָ�� �޷���תΪ�з��� ָ��0���з���ָ�����ײ����뺯�� Mapping rule: signed integer
  sym->value1 = value;
  sym->value2 = 0;
#if TRACE
  strncpy(sym->tracestring,tracestring,TRACESTRING_SIZE);
#endif
  assert (part->bitstream->streamBuffer != NULL);
  return writeSyntaxElement_UVLC (sym, part);
}


/*! 
 *************************************************************************************
 * \brief
 *    u_1, writes a flag (u(1) syntax element, returns the length in bits, 
 *    always 1
 *
 * \param tracestring
 *    the string for the trace file
 * \param value
 *    the value to be coded
 *  \param part
 *    the Data Partition the value should be coded into
 *
 * \return
 *    Number of bits used by the coded syntax element (always 1)
 *
 * \ note
 *    This function writes always the bit buffer for the progressive scan flag, and
 *    should not be used (or should be modified appropriately) for the interlace crap
 *    When used in the context of the Parameter Sets, this is obviously not a
 *    problem.
 *
 *************************************************************************************
 */
int u_1 (char *tracestring, int value, DataPartition *part)
{
  SyntaxElement symbol, *sym=&symbol;
	//��������mapping��ʽ����Ϊֻ��Ҫ��value�����һ��bitд������
  sym->bitpattern = value;// ���Ǻܶ���
  sym->len = 1; //���볤��Ϊ1 ���һ������д������
  sym->type = SE_HEADER;
  sym->value1 = value;
  sym->value2 = 0;
#if TRACE
  strncpy(sym->tracestring,tracestring,TRACESTRING_SIZE);
#endif
  assert (part->bitstream->streamBuffer != NULL);
  return writeSyntaxElement_fixed(sym, part);//�̶�len=1�ĳ���д��strambuffer
}


/*! 
 *************************************************************************************
 * \brief
 *    u_v, writes a a n bit fixed length syntax element, returns the length in bits, 
 *
 * \param n
 *    length in bits
 * \param tracestring
 *    the string for the trace file
 * \param value
 *    the value to be coded
 *  \param part
 *    the Data Partition the value should be coded into
 *
 * \return
 *    Number of bits used by the coded syntax element 
 *
 * \ note
 *    This function writes always the bit buffer for the progressive scan flag, and
 *    should not be used (or should be modified appropriately) for the interlace crap
 *    When used in the context of the Parameter Sets, this is obviously not a
 *    problem.
 *
 *************************************************************************************
 */

int u_v (int n, char *tracestring, int value, DataPartition *part)
{
  SyntaxElement symbol, *sym=&symbol;

  sym->bitpattern = value;
  sym->len = n; //���n������д������
  sym->type = SE_HEADER;
  sym->value1 = value;
  sym->value2 = 0;
#if TRACE
  strncpy(sym->tracestring,tracestring,TRACESTRING_SIZE);
#endif
  assert (part->bitstream->streamBuffer != NULL);
  return writeSyntaxElement_fixed(sym, part);//��ȡ�̶�len=n���ȵ�valueд�뵽strambuffer��
}


/*!
 ************************************************************************
 * \brief
 *    mapping for ue(v) syntax elements
 * \param ue
 *    value to be mapped
 * \param dummy
 *    dummy parameter
 * \param info
 *    returns mapped value
 * \param len
 *    returns mapped value length
 ************************************************************************
 */
void ue_linfo(int ue, int dummy, int *len,int *info)//�޷���0��ָ�����ײ����뺯��
{
  int i,nn;

  nn=(ue+1)/2;

  for (i=0; i < 16 && nn != 0; i++)//ue���2^15
  {
    nn /= 2;
  }
  *len= 2*i + 1;//i=log2(ue+1) i=log(ue+1)/log2 i��ǰ׺����
  *info=ue+1-(int)pow(2,i);//����infoʱ��û�а����м��1 
}


/*!
 ************************************************************************
 * \brief
 *    mapping for se(v) syntax elements
 * \param se
 *    value to be mapped
 * \param dummy
 *    dummy argument
 * \param len
 *    returns mapped value length
 * \param info
 *    returns mapped value
 ************************************************************************
 */
void se_linfo(int se, int dummy, int *len,int *info)
{

  int i,n,sign,nn;

  sign=0;
//����������ӳ�䵽����
//0:0 -1:1 1:2 -2:3 2:4.....
  if (se <= 0)
  {
    sign=1;
  }
  n=abs(se) << 1;
  
  /*
  n+1 is the number in the code table.  Based on this we find length and info
  */

  nn=n/2;
  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len=i*2 + 1;
  *info=n - (int)pow(2,i) + sign;
}


/*!
 ************************************************************************
 * \par Input:
 *    Number in the code table
 * \par Output:
 *    length and info
 ************************************************************************
 */
void cbp_linfo_intra(int cbp, int dummy, int *len,int *info)
{
//cbp ͨ������NCBP[48][2]ӳ�䣬�õ���ֵ���޷���ָ�����ײ����룬�õ�len info
  extern const int NCBP[48][2];//cpb coded block pattern 
  ue_linfo(NCBP[cbp][0], dummy, len, info);// 0 ��ʾ֡��
}


/*!
 ************************************************************************
 * \par Input:
 *    Number in the code table
 * \par Output:
 *    length and info
 ************************************************************************
 */
void cbp_linfo_inter(int cbp, int dummy, int *len,int *info)
{
  extern const int NCBP[48][2];
  ue_linfo(NCBP[cbp][1], dummy, len, info);// 1��ʾ֡��
}


/*!
 ************************************************************************
 * \brief
 *    2x2 transform of chroma DC
 * \par Input:
 *    level and run for coefficients
 * \par Output:
 *    length and info
 * \note
 *    see ITU document for bit assignment
 ************************************************************************
 */
 //ɫ�Ȳв��DC����
 //�����γ̱����level run,ӳ���n,ͨ���з���0��ָ�����ײ����룬������len info
void levrun_linfo_c2x2(int level,int run,int *len,int *info)
{
  const int NTAB[2][2]=
  {
    {1,5},
    {3,0}
  };
  const int LEVRUN[4]=
  {
    2,1,0,0
  };

  int levabs,i,n,sign,nn;

  if (level == 0) // �������� check if the coefficient sign EOB (level=0)
  {
    *len=1;
    return;
  }
  sign=0;
  if (level <= 0)
  {
    sign=1;
  }
  levabs=abs(level);
  if (levabs <= LEVRUN[run])
  {
    n=NTAB[levabs-1][run]+1;//ӳ��
  }
  else
  {
    n=(levabs-LEVRUN[run])*8 + run*2;
  }

  nn=n/2;

  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len= 2*i + 1;
  *info=n-(int)pow(2,i)+sign;
}


/*!
 ************************************************************************
 * \brief
 *    Single scan coefficients
 * \par Input:
 *    level and run for coefficiets
 * \par Output:
 *    lenght and info
 * \note
 *    see ITU document for bit assignment
 ************************************************************************
 */
 
void levrun_linfo_inter(int level,int run,int *len,int *info)
{
  const byte LEVRUN[16]=
  {
    4,2,2,1,1,1,1,1,1,1,0,0,0,0,0,0
  };
  const byte NTAB[4][10]=
  {
    { 1, 3, 5, 9,11,13,21,23,25,27},
    { 7,17,19, 0, 0, 0, 0, 0, 0, 0},
    {15, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {29, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  };

  int levabs,i,n,sign,nn;

  if (level == 0)           //  check for EOB
  {
    *len=1;
    return;
  }

  if (level <= 0)
    sign=1;
  else
    sign=0;

  levabs=abs(level);
  if (levabs <= LEVRUN[run])
  {
    n=NTAB[levabs-1][run]+1;
  }
  else
  {
    n=(levabs-LEVRUN[run])*32 + run*2;
  }

  nn=n/2;

  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len= 2*i + 1;
  *info=n-(int)pow(2,i)+sign;

}


/*!
 ************************************************************************
 * \brief
 *    Double scan coefficients
 * \par Input:
 *    level and run for coefficiets
 * \par Output:
 *    lenght and info
 * \note
 *    see ITU document for bit assignment
 ************************************************************************
 */
void levrun_linfo_intra(int level,int run,int *len,int *info)
{
  const byte LEVRUN[8]=
  {
    9,3,1,1,1,0,0,0
  };

  const byte NTAB[9][5] =
  {
    { 1, 3, 7,15,17},
    { 5,19, 0, 0, 0},
    { 9,21, 0, 0, 0},
    {11, 0, 0, 0, 0},
    {13, 0, 0, 0, 0},
    {23, 0, 0, 0, 0},
    {25, 0, 0, 0, 0},
    {27, 0, 0, 0, 0},
    {29, 0, 0, 0, 0},
  };

  int levabs,i,n,sign,nn;

  if (level == 0)     //  check for EOB
  {
    *len=1;
    return;
  }
  if (level <= 0)
    sign=1;
  else
    sign=0;

  levabs=abs(level);
  if (levabs <= LEVRUN[run])
  {
    n=NTAB[levabs-1][run]+1;
  }
  else
  {
    n=(levabs-LEVRUN[run])*16 + 16 + run*2;
  }

  nn=n/2;

  for (i=0; i < 16 && nn != 0; i++)
  {
    nn /= 2;
  }
  *len= 2*i + 1;
  *info=n-(int)pow(2,i)+sign;
}


/*!
 ************************************************************************
 * \brief
 *    Makes code word and passes it back
 *    A code word has the following format: 0 0 0 ... 1 Xn ...X2 X1 X0.
 *
 * \par Input:
 *    Info   : Xn..X2 X1 X0                                             \n
 *    Length : Total number of bits in the codeword
 ************************************************************************
 */
 // NOTE this function is called with sym->inf > (1<<(sym->len/2)).  The upper bits of inf are junk
int symbol2uvlc(SyntaxElement *sym)
{
	//ǰ���0Ϊʲôû�мӽ�ȥ?
  int suffix_len=sym->len/2;  //0��ָ�����ײ� ��1���ҵĸ�����ͬ sym->inf&((1<<suffix_len)-1)��ȡ��Xn ...X2 X1 X0
  sym->bitpattern = (1<<suffix_len)|(sym->inf&((1<<suffix_len)-1));//1<<suffix_len):Xn ...X2 X1 X0ǰ�����һ��1
  return 0;
}


/*!
 ************************************************************************
 * \brief
 *    generates UVLC code and passes the codeword to the buffer
 ************************************************************************
 */
int writeSyntaxElement_UVLC(SyntaxElement *se, DataPartition *this_dataPart)
{
  se->mapping(se->value1,se->value2,&(se->len),&(se->inf));//ͨ��seָ����mapping����ӳ�䷽ʽ���õ�len info
  symbol2uvlc(se);//�õ�[MZeros][1][INFO]��bitpattern

  writeUVLC2buffer(se, this_dataPart->bitstream);

  if(se->type != SE_HEADER)
    this_dataPart->bitstream->write_flag = 1;

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    passes the fixed codeword to the buffer
 ************************************************************************
 */
int writeSyntaxElement_fixed(SyntaxElement *se, DataPartition *this_dataPart)
{  
  writeUVLC2buffer(se, this_dataPart->bitstream);//ֱ�Ӹ���value �趨�Ĺ̶����Ƚ�ȡ������streambuffer
  
  if(se->type != SE_HEADER)
    this_dataPart->bitstream->write_flag = 1;

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    generates code and passes the codeword to the buffer
 ************************************************************************
 */
int writeSyntaxElement_Intra4x4PredictionMode(SyntaxElement *se, DataPartition *this_dataPart)
{

  if (se->value1 == -1)//Ԥ�����ʵ��֡��Ԥ��ģʽ��ͬ
  {
    se->len = 1;
    se->inf = 1;
  }
  else //��ͬ
  {
    se->len = 4;  //�̶����� 0��8��9�ֿ��� ��4bit
    se->inf = se->value1;//����ͬʱ�����ʵ��֡��Ԥ��ģʽд�뵽��������
  }

  se->bitpattern = se->inf;
  writeUVLC2buffer(se, this_dataPart->bitstream);

  if(se->type != SE_HEADER)
    this_dataPart->bitstream->write_flag = 1;

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    generates UVLC code and passes the codeword to the buffer
 * \author
 *  Tian Dong
 ************************************************************************
 */
int writeSyntaxElement2Buf_UVLC(SyntaxElement *se, Bitstream* this_streamBuffer )
{

  se->mapping(se->value1,se->value2,&(se->len),&(se->inf));

  symbol2uvlc(se);

  writeUVLC2buffer(se, this_streamBuffer );

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    writes UVLC code to the appropriate buffer
 ************************************************************************
 */
void  writeUVLC2buffer(SyntaxElement *se, Bitstream *currStream)
{

  int i;
  unsigned int mask = 1 << (se->len-1);//��ʼ������mask 100...00 

  // Add the new bits to the bitstream.
  // Write out a byte if it is full
  //д���bit �ȱ�����byte_buf�У���byte_buf�ռ���1��byte(8 bit)����һ��д�뵽streambuffer��
  /*
	����se->bitpattern=00101   len=5 mask:1000 byte_buf=0
	i=0 byte_buf=00     (se->bitpattern & mask)=0000   byte_buf=00     bits_to_go=7 mask=100
	i=1 byte_buf=000    (se->bitpattern & mask)=100    byte_buf=001    bits_to_go=6 mask=10
	i=2 byte_buf=0010   (se->bitpattern & mask)=00     byte_buf=0010   bits_to_go=5 mask=1
	i=3 byte_buf=00100  (se->bitpattern & mask)=1      byte_buf=00101  bits_to_go=4 mask=0
	i=4 byte_buf=001010 (se->bitpattern & mask)=0      byte_buf=001010 bits_to_go=3 mask=0
	
	���ʣ�1. bits_to_go��ʲôʱ��ʼ����
		  2.����i��ѭ������Ӧ���� i<(se->len-1)
  */
  for (i=0; i<se->len; i++)
  {
    currStream->byte_buf <<= 1;
    if (se->bitpattern & mask)
      currStream->byte_buf |= 1;
    currStream->bits_to_go--;
    mask >>= 1;
    if (currStream->bits_to_go==0)
    {
      currStream->bits_to_go = 8;
      currStream->streamBuffer[currStream->byte_pos++]=currStream->byte_buf;
      currStream->byte_buf = 0;
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    generates UVLC code and passes the codeword to the buffer
 * \author
 *  Tian Dong
 ************************************************************************
 */
int writeSyntaxElement2Buf_Fixed(SyntaxElement *se, Bitstream* this_streamBuffer )
{

  writeUVLC2buffer(se, this_streamBuffer );

#if TRACE
  if(se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    Makes code word and passes it back
 *
 * \par Input:
 *    Info   : Xn..X2 X1 X0                                             \n
 *    Length : Total number of bits in the codeword
 ************************************************************************
 */
// vlc������ǰ׺suffix
int symbol2vlc(SyntaxElement *sym)
{
  int info_len = sym->len;

  // Convert info into a bitpattern int
  sym->bitpattern = 0;

  // vlc coding
  /*
	00111 inf=11 info_len=5 bitpattern = 0
	info_len=4 bitpattern=0  (sym->inf >> info_len)=0  sym->bitpattern=0
	info_len=3 bitpattern=0  (sym->inf >> info_len)=0  sym->bitpattern=0
	info_len=2 bitpattern=0  (sym->inf >> info_len)=0  sym->bitpattern=0
	info_len=1 bitpattern=0  (sym->inf >> info_len)=1  sym->bitpattern=1
	info_len=0 bitpattern=10 (sym->inf >> info_len)=11 sym->bitpattern=1

	sym->bitpattern 0,0,0,1,1
  */
  while(--info_len >= 0)
  {
    sym->bitpattern <<= 1;
    sym->bitpattern |= (0x01 & (sym->inf >> info_len));// ?? û����
  }
  return 0;
}


/*!
 ************************************************************************
 * \brief
 *    generates VLC code and passes the codeword to the buffer
 ************************************************************************
 */
int writeSyntaxElement_VLC(SyntaxElement *se, DataPartition *this_dataPart)
{

  se->inf = se->value1;//
  se->len = se->value2;
  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for NumCoeff and TrailingOnes
 ************************************************************************
 */

int writeSyntaxElement_NumCoeffTrailingOnes(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[3][4][17] = 
  {
    {   // 0702
      { 1, 6, 8, 9,10,11,13,13,13,14,14,15,15,16,16,16,16},
      { 0, 2, 6, 8, 9,10,11,13,13,14,14,15,15,15,16,16,16},
      { 0, 0, 3, 7, 8, 9,10,11,13,13,14,14,15,15,16,16,16},
      { 0, 0, 0, 5, 6, 7, 8, 9,10,11,13,14,14,15,15,16,16},
    },                                                 
    {                                                  
      { 2, 6, 6, 7, 8, 8, 9,11,11,12,12,12,13,13,13,14,14},
      { 0, 2, 5, 6, 6, 7, 8, 9,11,11,12,12,13,13,14,14,14},
      { 0, 0, 3, 6, 6, 7, 8, 9,11,11,12,12,13,13,13,14,14},
      { 0, 0, 0, 4, 4, 5, 6, 6, 7, 9,11,11,12,13,13,13,14},
    },                                                 
    {                                                  
      { 4, 6, 6, 6, 7, 7, 7, 7, 8, 8, 9, 9, 9,10,10,10,10},
      { 0, 4, 5, 5, 5, 5, 6, 6, 7, 8, 8, 9, 9, 9,10,10,10},
      { 0, 0, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,10,10,10},
      { 0, 0, 0, 4, 4, 4, 4, 4, 5, 6, 7, 8, 8, 9,10,10,10},
    },

  };

  int codtab[3][4][17] = 
  {
    {
      { 1, 5, 7, 7, 7, 7,15,11, 8,15,11,15,11,15,11, 7,4}, 
      { 0, 1, 4, 6, 6, 6, 6,14,10,14,10,14,10, 1,14,10,6}, 
      { 0, 0, 1, 5, 5, 5, 5, 5,13, 9,13, 9,13, 9,13, 9,5}, 
      { 0, 0, 0, 3, 3, 4, 4, 4, 4, 4,12,12, 8,12, 8,12,8},
    },
    {
      { 3,11, 7, 7, 7, 4, 7,15,11,15,11, 8,15,11, 7, 9,7}, 
      { 0, 2, 7,10, 6, 6, 6, 6,14,10,14,10,14,10,11, 8,6}, 
      { 0, 0, 3, 9, 5, 5, 5, 5,13, 9,13, 9,13, 9, 6,10,5}, 
      { 0, 0, 0, 5, 4, 6, 8, 4, 4, 4,12, 8,12,12, 8, 1,4},
    },
    {
      {15,15,11, 8,15,11, 9, 8,15,11,15,11, 8,13, 9, 5,1}, 
      { 0,14,15,12,10, 8,14,10,14,14,10,14,10, 7,12, 8,4},
      { 0, 0,13,14,11, 9,13, 9,13,10,13, 9,13, 9,11, 7,3},
      { 0, 0, 0,12,11,10, 9, 8,13,12,12,12, 8,12,10, 6,2},
    },
  };
  int vlcnum;

  vlcnum = se->len;//����ط��ĳ�����ʲô�ط���ֵ�ģ�����  se->len ���ڸı䣿

  // se->value1 : numcoeff
  // se->value2 : numtrailingones

  if (vlcnum == 3)
  {
    se->len = 6;  // 4 + 2 bit FLC
    if (se->value1 > 0)
    {
      se->inf = ((se->value1-1) << 2) | se->value2;
    }
    else
    {
      se->inf = 3;
    }
  }
  else
  {
    se->len = lentab[vlcnum][se->value2][se->value1];//����ϵ������(value1)����βϵ������(value2)ֱ�Ӳ��õ�len info
    se->inf = codtab[vlcnum][se->value2][se->value1];
  }
  //se->inf = 0;

  if (se->len == 0)
  {
    printf("ERROR: (numcoeff,trailingones) not valid: vlc=%d (%d, %d)\n", 
      vlcnum, se->value1, se->value2);
    exit(-1);
  }

  symbol2vlc(se);//�õ�bitpattern

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for NumCoeff and TrailingOnes for Chroma DC
 ************************************************************************
 */
int writeSyntaxElement_NumCoeffTrailingOnesChromaDC(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[4][5] = 
  {
    { 2, 6, 6, 6, 6,},          
    { 0, 1, 6, 7, 8,}, 
    { 0, 0, 3, 7, 8,}, 
    { 0, 0, 0, 6, 7,},
  };

  int codtab[4][5] = 
  {
    {1,7,4,3,2},
    {0,1,6,3,3},
    {0,0,1,2,2},
    {0,0,0,5,0},
  };

  // se->value1 : numcoeff
  // se->value2 : numtrailingones
  se->len = lentab[se->value2][se->value1];
  se->inf = codtab[se->value2][se->value1];

  if (se->len == 0)
  {
    printf("ERROR: (numcoeff,trailingones) not valid: (%d, %d)\n", 
      se->value1, se->value2);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for TotalZeros
 ************************************************************************
 */
int writeSyntaxElement_TotalZeros(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[TOTRUN_NUM][16] = 
  {
    { 1,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9},  
    { 3,3,3,3,3,4,4,4,4,5,5,6,6,6,6},  
    { 4,3,3,3,4,4,3,3,4,5,5,6,5,6},  
    { 5,3,4,4,3,3,3,4,3,4,5,5,5},  
    { 4,4,4,3,3,3,3,3,4,5,4,5},  
    { 6,5,3,3,3,3,3,3,4,3,6},  
    { 6,5,3,3,3,2,3,4,3,6},  
    { 6,4,5,3,2,2,3,3,6},  
    { 6,6,4,2,2,3,2,5},  
    { 5,5,3,2,2,2,4},  
    { 4,4,3,3,1,3},  
    { 4,4,2,1,3},  
    { 3,3,1,2},  
    { 2,2,1},  
    { 1,1},  
  };

  int codtab[TOTRUN_NUM][16] = 
  {
    {1,3,2,3,2,3,2,3,2,3,2,3,2,3,2,1},
    {7,6,5,4,3,5,4,3,2,3,2,3,2,1,0},
    {5,7,6,5,4,3,4,3,2,3,2,1,1,0},
    {3,7,5,4,6,5,4,3,3,2,2,1,0},
    {5,4,3,7,6,5,4,3,2,1,1,0},
    {1,1,7,6,5,4,3,2,1,1,0},
    {1,1,5,4,3,3,2,1,1,0},
    {1,1,1,3,3,2,2,1,0},
    {1,0,1,3,2,1,1,1,},
    {1,0,1,3,2,1,1,},
    {0,1,1,2,1,3},
    {0,1,1,1,1},
    {0,1,1,1},
    {0,1,1},
    {0,1},  
  };
  int vlcnum;

  vlcnum = se->len;

  // se->value1 : TotalZeros
  se->len = lentab[vlcnum][se->value1];
  se->inf = codtab[vlcnum][se->value1];

  if (se->len == 0)
  {
    printf("ERROR: (TotalZeros) not valid: (%d)\n",se->value1);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for TotalZeros for Chroma DC
 ************************************************************************
 */
int writeSyntaxElement_TotalZerosChromaDC(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[3][4] = 
  {
    { 1, 2, 3, 3,},
    { 1, 2, 2, 0,},
    { 1, 1, 0, 0,}, 
  };

  int codtab[3][4] = 
  {
    { 1, 1, 1, 0,},
    { 1, 1, 0, 0,},
    { 1, 0, 0, 0,},
  };
  int vlcnum;

  vlcnum = se->len;

  // se->value1 : TotalZeros
  se->len = lentab[vlcnum][se->value1];
  se->inf = codtab[vlcnum][se->value1];

  if (se->len == 0)
  {
    printf("ERROR: (TotalZeros) not valid: (%d)\n",se->value1);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for Run Before Next Coefficient, VLC0
 ************************************************************************
 */
int writeSyntaxElement_Run(SyntaxElement *se, DataPartition *this_dataPart)
{
  int lentab[TOTRUN_NUM][16] = 
  {
    {1,1},
    {1,2,2},
    {2,2,2,2},
    {2,2,2,3,3},
    {2,2,3,3,3,3},
    {2,3,3,3,3,3,3},
    {3,3,3,3,3,3,3,4,5,6,7,8,9,10,11},
  };

  int codtab[TOTRUN_NUM][16] = 
  {
    {1,0},
    {1,1,0},
    {3,2,1,0},
    {3,2,1,1,0},
    {3,2,3,2,1,0},
    {3,0,1,3,2,5,4},
    {7,6,5,4,3,2,1,1,1,1,1,1,1,1,1},
  };
  int vlcnum;

 	//se->len=zeroLeft
 	//se->value1=run_before
  vlcnum = se->len;

  // se->value1 : run
  se->len = lentab[vlcnum][se->value1];
  se->inf = codtab[vlcnum][se->value1];

  if (se->len == 0)
  {
    printf("ERROR: (run) not valid: (%d)\n",se->value1);
    exit(-1);
  }

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for Coeff Level (VLC1)
 ************************************************************************
 */
int writeSyntaxElement_Level_VLC1(SyntaxElement *se, DataPartition *this_dataPart)
{
  int level, levabs, sign;

  level = se->value1;
  levabs = abs(level);
  sign = (level < 0 ? 1 : 0);

  
  if (levabs < 8)
  {
    se->len = levabs * 2 + sign - 1;
    se->inf = 1;//��1ǰ�油0���ﵽlen�ĳ���
  }
  else if (levabs < 8+8)
  {
    // escape code1
    se->len = 14 + 1 + 4;
    se->inf = (1 << 4) | ((levabs - 8) << 1) | sign;//inf����Ϊ1+4
  }
  else
  {
    // escape code2
    se->len = 14 + 2 + 12;
    se->inf = (0x1 << 12) | ((levabs - 16)<< 1) | sign;//inf����Ϊ2+12
  }


  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    write VLC for Coeff Level
 ************************************************************************
 */
int writeSyntaxElement_Level_VLCN(SyntaxElement *se, int vlc, DataPartition *this_dataPart)
{
  int iCodeword;
  int iLength;

  int level = se->value1;

  int levabs = abs(level);
  int sign = (level < 0 ? 1 : 0);  

  int shift = vlc-1;
  int escape = (15<<shift)+1;

  int numPrefix = (levabs-1)>>shift;

  int sufmask = ~((0xffffffff)<<shift);
  int suffix = (levabs-1)&sufmask;

  if (levabs < escape)
  {
    iLength = numPrefix + vlc + 1;
    iCodeword = (1<<(shift+1))|(suffix<<1)|sign;
  }
  else
  {
    iLength = 15+1+12;
    iCodeword = (1<<12)|((levabs-escape)<<1)|sign;
  }
  se->len = iLength;
  se->inf = iCodeword;

  symbol2vlc(se);

  writeUVLC2buffer(se, this_dataPart->bitstream);
#if TRACE
  if (se->type <= 1)
    trace2out (se);
#endif

  return (se->len);
}


/*!
 ************************************************************************
 * \brief
 *    Write out a trace string on the trace file
 ************************************************************************
 */
#if TRACE
void trace2out(SyntaxElement *sym)
{
  static int bitcounter = 0;
  int i, chars;

  if (p_trace != NULL)
  {
    putc('@', p_trace);//���ַ���@��д�뵽�ļ���
    chars = fprintf(p_trace, "%i", bitcounter);
    while(chars++ < 6)
      putc(' ',p_trace);

    chars += fprintf(p_trace, "%s", sym->tracestring);
    while(chars++ < 55)
      putc(' ',p_trace);

  // Align bitpattern
    if(sym->len<15)
    {
      for(i=0 ; i<15-sym->len ; i++)
        fputc(' ', p_trace);
    }
    
    // Print bitpattern
    bitcounter += sym->len;//���bit������
    for(i=1 ; i<=sym->len ; i++)
    {
    /*
		sym->bitpattern=1101  sym->len=4
		i=1  (sym->bitpattern >> (sym->len-i)=1     ((sym->bitpattern >> (sym->len-i)) & 0x1)=1
		i=2  (sym->bitpattern >> (sym->len-i)=11    ((sym->bitpattern >> (sym->len-i)) & 0x1)=1
		i=3  (sym->bitpattern >> (sym->len-i)=110   ((sym->bitpattern >> (sym->len-i)) & 0x1)=0
		i=4  (sym->bitpattern >> (sym->len-i)=1101  ((sym->bitpattern >> (sym->len-i)) & 0x1)=1
		������ȡ���﷨Ԫ��bitpattern�����ÿһλbit
	*/
      if((sym->bitpattern >> (sym->len-i)) & 0x1)
        fputc('1', p_trace);
      else
        fputc('0', p_trace);
    }
    fprintf(p_trace, " (%3d) \n",sym->value1);
  }
  fflush (p_trace);
}
#endif


/*!
 ************************************************************************
 * \brief
 *    puts the less than 8 bits in the byte buffer of the Bitstream into
 *    the streamBuffer.  
 *
 * \param
 *   currStream: the Bitstream the alignment should be established
 *
 ************************************************************************
 */
void writeVlcByteAlign(Bitstream* currStream)
{
  if (currStream->bits_to_go < 8)
  { // trailing bits to process
    currStream->byte_buf = (currStream->byte_buf <<currStream->bits_to_go) | (0xff >> (8 - currStream->bits_to_go));
    stat->bit_use_stuffingBits[img->type]+=currStream->bits_to_go;
    currStream->streamBuffer[currStream->byte_pos++]=currStream->byte_buf;
    currStream->bits_to_go = 8;
  }
}
