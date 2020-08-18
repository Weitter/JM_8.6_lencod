
/*!
 *************************************************************************************
 * \file block.c
 *
 * \brief
 *    Process one block
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Inge Lille-Lang鴜               <inge.lille-langoy@telenor.com>
 *    - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *    - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *    - Jani Lainema                    <jani.lainema@nokia.com>
 *    - Detlev Marpe                    <marpe@hhi.de>
 *    - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *    - Ragip Kurceren                  <ragip.kurceren@nokia.com>
 *    - Greg Conklin                    <gregc@real.com>
 *************************************************************************************
 */

#include "contributors.h"


#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "block.h"
#include "refbuf.h"
#include "vlc.h"
#include "mb_access.h"
#include "image.h"


#define Q_BITS          15
#define DQ_BITS         6
#define DQ_ROUND        (1<<(DQ_BITS-1))//2^(5)=32

//通过QP%6的值选出是哪一行，再通过4*4的i,j坐标算出对应块处的量化矩阵值
static const int quant_coef[6][4][4] = {
  {{13107, 8066,13107, 8066},{ 8066, 5243, 8066, 5243},{13107, 8066,13107, 8066},{ 8066, 5243, 8066, 5243}},
  {{11916, 7490,11916, 7490},{ 7490, 4660, 7490, 4660},{11916, 7490,11916, 7490},{ 7490, 4660, 7490, 4660}},
  {{10082, 6554,10082, 6554},{ 6554, 4194, 6554, 4194},{10082, 6554,10082, 6554},{ 6554, 4194, 6554, 4194}},
  {{ 9362, 5825, 9362, 5825},{ 5825, 3647, 5825, 3647},{ 9362, 5825, 9362, 5825},{ 5825, 3647, 5825, 3647}},
  {{ 8192, 5243, 8192, 5243},{ 5243, 3355, 5243, 3355},{ 8192, 5243, 8192, 5243},{ 5243, 3355, 5243, 3355}},
  {{ 7282, 4559, 7282, 4559},{ 4559, 2893, 4559, 2893},{ 7282, 4559, 7282, 4559},{ 4559, 2893, 4559, 2893}}
};
//反量化矩阵
static const int dequant_coef[6][4][4] = {
  {{10, 13, 10, 13},{ 13, 16, 13, 16},{10, 13, 10, 13},{ 13, 16, 13, 16}},
  {{11, 14, 11, 14},{ 14, 18, 14, 18},{11, 14, 11, 14},{ 14, 18, 14, 18}},
  {{13, 16, 13, 16},{ 16, 20, 16, 20},{13, 16, 13, 16},{ 16, 20, 16, 20}},
  {{14, 18, 14, 18},{ 18, 23, 18, 23},{14, 18, 14, 18},{ 18, 23, 18, 23}},
  {{16, 20, 16, 20},{ 20, 25, 20, 25},{16, 20, 16, 20},{ 20, 25, 20, 25}},
  {{18, 23, 18, 23},{ 23, 29, 23, 29},{18, 23, 18, 23},{ 23, 29, 23, 29}}
};
//权重矩阵？
static const int A[4][4] = {
  { 16, 20, 16, 20},
  { 20, 25, 20, 25},
  { 16, 20, 16, 20},
  { 20, 25, 20, 25}
};


// Notation for comments regarding prediction and predictors.
// The pels of the 4x4 block are labelled a..p. The predictor pels above
// are labelled A..H, from the left I..P, and from above left X, as follows:
//
//  X A B C D E F G H
//  I a b c d
//  J e f g h
//  K i j k l
//  L m n o p
//

// Predictor array index definitions
#define P_X (PredPel[0])
#define P_A (PredPel[1])
#define P_B (PredPel[2])
#define P_C (PredPel[3])
#define P_D (PredPel[4])
#define P_E (PredPel[5])
#define P_F (PredPel[6])
#define P_G (PredPel[7])
#define P_H (PredPel[8])
#define P_I (PredPel[9])
#define P_J (PredPel[10])
#define P_K (PredPel[11])
#define P_L (PredPel[12])

/*!
 ************************************************************************
 * \brief
 *    Make intra 4x4 prediction according to all 9 prediction modes.
 *    The routine uses left and upper neighbouring points from
 *    previous(先前的) coded blocks to do this (if available). Notice that
 *    inaccessible(难以达到的) neighbouring points are signalled with a negative
 *    value in the predmode array .
 *
 *  \par Input:
 *     Starting point of current 4x4 block image posision
 *
 *  \par Output:
 *      none
 ************************************************************************
 */
 //4*4块9种帧内预测
void intrapred_luma(int img_x,int img_y, int *left_available, int *up_available, int *all_available)
{
  int i,j;
  int s0;
  int PredPel[13];  // array of predictor pels
  byte **imgY = enc_picture->imgY;  // For MB level frame/field coding tools -- set default to imgY

  int ioff = (img_x & 15);//取出八位二进制
  int joff = (img_y & 15);//(ioff,joff) 应该是宏块左上角为原点 各个4*4块的左上角的像素坐标
  int mb_nr=img->current_mb_nr;
/*
	db   c
	axxxx
	axxxx
	axxxx
	axxxx
  */
  PixelPos pix_a[4];
  PixelPos pix_b, pix_c, pix_d;

  int block_available_up;//b
  int block_available_left;//a
  int block_available_up_left;//d
  int block_available_up_right;//c

  for (i=0;i<4;i++)
  {
    getNeighbour(mb_nr, ioff -1 , joff +i , 1, &pix_a[i]);//左边像素 a[0] a[1] a[2] a[3]
  }
  
  
  getNeighbour(mb_nr, ioff    , joff -1 , 1, &pix_b);//左上像素 b
  getNeighbour(mb_nr, ioff +4 , joff -1 , 1, &pix_c);//右上像素 c
  getNeighbour(mb_nr, ioff -1 , joff -1 , 1, &pix_d);//左上像素 d
/*
	ooooo
	oxoxo
	ooooo
	oxoxo
	ooooo
  */
  pix_c.available = pix_c.available && !(((ioff==4)||(ioff==12)) && ((joff==4)||(joff==12)));//暂时想不通？

  if (input->UseConstrainedIntraPred)//帧间预测宏块的相邻宏块不能是帧内预测
  {
    for (i=0, block_available_left=1; i<4;i++)
      block_available_left  &= pix_a[i].available ? img->intra_block[pix_a[i].mb_addr]: 0;//不是很懂
    block_available_up       = pix_b.available ? img->intra_block [pix_b.mb_addr] : 0;
    block_available_up_right = pix_c.available ? img->intra_block [pix_c.mb_addr] : 0;
    block_available_up_left  = pix_d.available ? img->intra_block [pix_d.mb_addr] : 0;
  }
  else//帧间预测宏块的相邻宏块可以是帧内预测
  {
    block_available_left     = pix_a[0].available;
    block_available_up       = pix_b.available;
    block_available_up_right = pix_c.available;
    block_available_up_left  = pix_d.available;
  }
  
  *left_available = block_available_left;
  *up_available   = block_available_up;
  *all_available  = block_available_up && block_available_left && block_available_up_left;

  i = (img_x & 15);// -> x
  j = (img_y & 15);// 向下 j(y)
//  X A B C D E F G H
//  I a b c d
//  J e f g h
//  K i j k l
//  L m n o p

  // form predictor pels
  if (block_available_up)
  {
    P_A = imgY[pix_b.pos_y][pix_b.pos_x+0];//A 点的亮度像素值0到255
    P_B = imgY[pix_b.pos_y][pix_b.pos_x+1];
    P_C = imgY[pix_b.pos_y][pix_b.pos_x+2];
    P_D = imgY[pix_b.pos_y][pix_b.pos_x+3];

  }
  else
  {
    P_A = P_B = P_C = P_D = 128;
  }

  if (block_available_up_right)
  {
    P_E = imgY[pix_c.pos_y][pix_c.pos_x+0];
    P_F = imgY[pix_c.pos_y][pix_c.pos_x+1];
    P_G = imgY[pix_c.pos_y][pix_c.pos_x+2];
    P_H = imgY[pix_c.pos_y][pix_c.pos_x+3];
  }
  else
  {
    P_E = P_F = P_G = P_H = P_D;
  }

  if (block_available_left)
  {
    P_I = imgY[pix_a[0].pos_y][pix_a[0].pos_x];
    P_J = imgY[pix_a[1].pos_y][pix_a[1].pos_x];
    P_K = imgY[pix_a[2].pos_y][pix_a[2].pos_x];
    P_L = imgY[pix_a[3].pos_y][pix_a[3].pos_x];
  }
  else
  {
    P_I = P_J = P_K = P_L = 128;
  }

  if (block_available_up_left)
  {
    P_X = imgY[pix_d.pos_y][pix_d.pos_x];
  }
  else
  {
    P_X = 128;
  }

  for(i=0;i<9;i++)
    img->mprr[i][0][0]=-1;//初始化 i表示预测像素的方式，对于预测方向存储着预测的块值
  //  X A B C D E F G H
  //  I a b c d
  //  J e f g h
  //  K i j k l
  //  L m n o p
  ///////////////////////////////
  // make DC prediction  
  ///////////////////////////////
  s0 = 0;
  if (block_available_up && block_available_left)
  {   
    // no edge
    s0 = (P_A + P_B + P_C + P_D + P_I + P_J + P_K + P_L + 4)/(2*BLOCK_SIZE);
  }
  else if (!block_available_up && block_available_left)
  {
    // upper edge
    s0 = (P_I + P_J + P_K + P_L + 2)/BLOCK_SIZE;             
  }
  else if (block_available_up && !block_available_left)
  {
    // left edge
    s0 = (P_A + P_B + P_C + P_D + 2)/BLOCK_SIZE;             
  }
  else //if (!block_available_up && !block_available_left)
  {
    // top left corner, nothing to predict from
    s0 = 128;                           
  }

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < BLOCK_SIZE; i++)
    {
      // store DC prediction
      img->mprr[DC_PRED][i][j] = s0;//把DC预测的值保存在 img->mprr[DC_PRED][i][j] 数组中
    }
  }
  //  X A B C D E F G H
  //  I a b c d
  //  J e f g h
  //  K i j k l
  //  L m n o p
  ///////////////////////////////
  // make horiz and vert prediction
  ///////////////////////////////
 /* for (i=0; i < BLOCK_SIZE; i++)
  {
    img->mprr[VERT_PRED][0][i] =  P_A;
    img->mprr[VERT_PRED][1][i] =  P_B;
    img->mprr[VERT_PRED][2][i] =  P_C;
    img->mprr[VERT_PRED][3][i] =  P_D;// 0 1 2 3表示列号
    img->mprr[HOR_PRED][i][0]  =  P_I;
    img->mprr[HOR_PRED][i][1]  =  P_J;
    img->mprr[HOR_PRED][i][2]  =  P_K;
    img->mprr[HOR_PRED][i][3]  =  P_L;
  }
*/
  for (i=0; i < BLOCK_SIZE; i++)
  {
    img->mprr[VERT_PRED][0][i] = 
    img->mprr[VERT_PRED][1][i] = 
    img->mprr[VERT_PRED][2][i] = 
    img->mprr[VERT_PRED][3][i] = (&P_A)[i];// 0 1 2 3表示列号
    img->mprr[HOR_PRED][i][0]  = 
    img->mprr[HOR_PRED][i][1]  = 
    img->mprr[HOR_PRED][i][2]  = 
    img->mprr[HOR_PRED][i][3]  = (&P_I)[i];
  }

  if(!block_available_up)img->mprr[VERT_PRED][0][0]=-1;//在上面赋值之前是不是就应该判断
  if(!block_available_left)img->mprr[HOR_PRED][0][0]=-1;

  if (block_available_up) 
  {
    // Mode DIAG_DOWN_LEFT_PRED  和书上讲的不一样
    img->mprr[DIAG_DOWN_LEFT_PRED][0][0] = (P_A + P_C + 2*(P_B) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][0][1] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][1][0] = (P_B + P_D + 2*(P_C) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][0][2] =
    img->mprr[DIAG_DOWN_LEFT_PRED][1][1] =
    img->mprr[DIAG_DOWN_LEFT_PRED][2][0] = (P_C + P_E + 2*(P_D) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][0][3] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][1][2] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][2][1] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][3][0] = (P_D + P_F + 2*(P_E) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][1][3] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][2][2] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][3][1] = (P_E + P_G + 2*(P_F) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][2][3] = 
    img->mprr[DIAG_DOWN_LEFT_PRED][3][2] = (P_F + P_H + 2*(P_G) + 2) / 4;
    img->mprr[DIAG_DOWN_LEFT_PRED][3][3] = (P_G + 3*(P_H) + 2) / 4;

    // Mode VERT_LEFT_PRED
    img->mprr[VERT_LEFT_PRED][0][0] = (P_A + P_B + 1) / 2;
    img->mprr[VERT_LEFT_PRED][0][1] = 
    img->mprr[VERT_LEFT_PRED][2][0] = (P_B + P_C + 1) / 2;
    img->mprr[VERT_LEFT_PRED][0][2] = 
    img->mprr[VERT_LEFT_PRED][2][1] = (P_C + P_D + 1) / 2;
    img->mprr[VERT_LEFT_PRED][0][3] = 
    img->mprr[VERT_LEFT_PRED][2][2] = (P_D + P_E + 1) / 2;
    img->mprr[VERT_LEFT_PRED][2][3] = (P_E + P_F + 1) / 2;
    img->mprr[VERT_LEFT_PRED][1][0] = (P_A + 2*P_B + P_C + 2) / 4;
    img->mprr[VERT_LEFT_PRED][1][1] = 
    img->mprr[VERT_LEFT_PRED][3][0] = (P_B + 2*P_C + P_D + 2) / 4;
    img->mprr[VERT_LEFT_PRED][1][2] = 
    img->mprr[VERT_LEFT_PRED][3][1] = (P_C + 2*P_D + P_E + 2) / 4;
    img->mprr[VERT_LEFT_PRED][1][3] = 
    img->mprr[VERT_LEFT_PRED][3][2] = (P_D + 2*P_E + P_F + 2) / 4;
    img->mprr[VERT_LEFT_PRED][3][3] = (P_E + 2*P_F + P_G + 2) / 4;

  }

  /*  Prediction according to 'diagonal' modes  对角线模式*/
  if (block_available_left) 
  {
    // Mode HOR_UP_PRED
    img->mprr[HOR_UP_PRED][0][0] = (P_I + P_J + 1) / 2;
    img->mprr[HOR_UP_PRED][0][1] = (P_I + 2*P_J + P_K + 2) / 4;
    img->mprr[HOR_UP_PRED][0][2] = 
    img->mprr[HOR_UP_PRED][1][0] = (P_J + P_K + 1) / 2;
    img->mprr[HOR_UP_PRED][0][3] = 
    img->mprr[HOR_UP_PRED][1][1] = (P_J + 2*P_K + P_L + 2) / 4;
    img->mprr[HOR_UP_PRED][1][2] = 
    img->mprr[HOR_UP_PRED][2][0] = (P_K + P_L + 1) / 2;
    img->mprr[HOR_UP_PRED][1][3] = 
    img->mprr[HOR_UP_PRED][2][1] = (P_K + 2*P_L + P_L + 2) / 4;
    img->mprr[HOR_UP_PRED][3][0] = 
    img->mprr[HOR_UP_PRED][2][2] = 
    img->mprr[HOR_UP_PRED][2][3] = 
    img->mprr[HOR_UP_PRED][3][1] = 
    img->mprr[HOR_UP_PRED][3][2] = 
    img->mprr[HOR_UP_PRED][3][3] = P_L;
  }

  /*  Prediction according to 'diagonal' modes */
  if (block_available_up && block_available_left && block_available_up_left) 
  {
    // Mode DIAG_DOWN_RIGHT_PRED
    img->mprr[DIAG_DOWN_RIGHT_PRED][3][0] = (P_L + 2*P_K + P_J + 2) / 4; 
    img->mprr[DIAG_DOWN_RIGHT_PRED][2][0] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][3][1] = (P_K + 2*P_J + P_I + 2) / 4; 
    img->mprr[DIAG_DOWN_RIGHT_PRED][1][0] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][2][1] = 
    img->mprr[DIAG_DOWN_RIGHT_PRED][3][2] = (P_J + 2*P_I + P_X + 2) / 4; 
    img->mprr[DIAG_DOWN_RIGHT_PRED][0][0] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][1][1] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][2][2] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][3][3] = (P_I + 2*P_X + P_A + 2) / 4; 
    img->mprr[DIAG_DOWN_RIGHT_PRED][0][1] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][1][2] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][2][3] = (P_X + 2*P_A + P_B + 2) / 4;
    img->mprr[DIAG_DOWN_RIGHT_PRED][0][2] =
    img->mprr[DIAG_DOWN_RIGHT_PRED][1][3] = (P_A + 2*P_B + P_C + 2) / 4;
    img->mprr[DIAG_DOWN_RIGHT_PRED][0][3] = (P_B + 2*P_C + P_D + 2) / 4;

     // Mode VERT_RIGHT_PRED
    img->mprr[VERT_RIGHT_PRED][0][0] = 
    img->mprr[VERT_RIGHT_PRED][2][1] = (P_X + P_A + 1) / 2;
    img->mprr[VERT_RIGHT_PRED][0][1] = 
    img->mprr[VERT_RIGHT_PRED][2][2] = (P_A + P_B + 1) / 2;
    img->mprr[VERT_RIGHT_PRED][0][2] = 
    img->mprr[VERT_RIGHT_PRED][2][3] = (P_B + P_C + 1) / 2;
    img->mprr[VERT_RIGHT_PRED][0][3] = (P_C + P_D + 1) / 2;
    img->mprr[VERT_RIGHT_PRED][1][0] = 
    img->mprr[VERT_RIGHT_PRED][3][1] = (P_I + 2*P_X + P_A + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][1][1] = 
    img->mprr[VERT_RIGHT_PRED][3][2] = (P_X + 2*P_A + P_B + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][1][2] = 
    img->mprr[VERT_RIGHT_PRED][3][3] = (P_A + 2*P_B + P_C + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][1][3] = (P_B + 2*P_C + P_D + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][2][0] = (P_X + 2*P_I + P_J + 2) / 4;
    img->mprr[VERT_RIGHT_PRED][3][0] = (P_I + 2*P_J + P_K + 2) / 4;

    // Mode HOR_DOWN_PRED
    img->mprr[HOR_DOWN_PRED][0][0] = 
    img->mprr[HOR_DOWN_PRED][1][2] = (P_X + P_I + 1) / 2;
    img->mprr[HOR_DOWN_PRED][0][1] = 
    img->mprr[HOR_DOWN_PRED][1][3] = (P_I + 2*P_X + P_A + 2) / 4;
    img->mprr[HOR_DOWN_PRED][0][2] = (P_X + 2*P_A + P_B + 2) / 4;
    img->mprr[HOR_DOWN_PRED][0][3] = (P_A + 2*P_B + P_C + 2) / 4;
    img->mprr[HOR_DOWN_PRED][1][0] = 
    img->mprr[HOR_DOWN_PRED][2][2] = (P_I + P_J + 1) / 2;
    img->mprr[HOR_DOWN_PRED][1][1] = 
    img->mprr[HOR_DOWN_PRED][2][3] = (P_X + 2*P_I + P_J + 2) / 4;
    img->mprr[HOR_DOWN_PRED][2][0] = 
    img->mprr[HOR_DOWN_PRED][3][2] = (P_J + P_K + 1) / 2;
    img->mprr[HOR_DOWN_PRED][2][1] = 
    img->mprr[HOR_DOWN_PRED][3][3] = (P_I + 2*P_J + P_K + 2) / 4;
    img->mprr[HOR_DOWN_PRED][3][0] = (P_K + P_L + 1) / 2;
    img->mprr[HOR_DOWN_PRED][3][1] = (P_J + 2*P_K + P_L + 2) / 4;
  }
}

/*!
 ************************************************************************
 * \brief
 *    16x16 based luma prediction 有4中预测模式
 *
 * \par Input:
 *    Image parameters
 *
 * \par Output:
 *    none
 ************************************************************************
 */
 //16*16 宏块4种方式帧内预测
void intrapred_luma_16x16()
{
  int s0=0,s1,s2;
  int s[16][2];
  int i,j;

  int ih,iv;
  int ib,ic,iaa;

  byte   **imgY_pred = enc_picture->imgY;  // For Mb level field/frame coding tools -- default to frame pred
  int          mb_nr = img->current_mb_nr;

  PixelPos up;          //!< pixel position p(0,-1)
  PixelPos left[17];    //!< pixel positions p(-1, -1..15)

  int up_avail, left_avail, left_up_avail;
/*
	left[0] up
	left[1] curr
	left[2]
	left[3]
	left[4]
	..
	left[16]
	
	
	*/
  for (i=0;i<17;i++)
  {
    getNeighbour(mb_nr, -1 ,  i-1 , 1, &left[i]);//左边列的17个像素 left[i]的x坐标一样
  }
  
  getNeighbour(mb_nr, 0     ,  -1 , 1, &up);//当前宏块左上角像素的上边一个像素

  if (!(input->UseConstrainedIntraPred))//帧内预测的相邻块可以是帧间预测
  {
    up_avail   = up.available;
    left_avail = left[1].available;
    left_up_avail = left[0].available;
  }
  else////帧内预测的相邻块不能是帧间预测 也就是相邻块必须是帧内预测
  {
    up_avail      = up.available ? img->intra_block[up.mb_addr] : 0;
    for (i=1, left_avail=1; i<17;i++)
      left_avail  &= left[i].available ? img->intra_block[left[i].mb_addr]: 0;
    left_up_avail = left[0].available ? img->intra_block[left[0].mb_addr]: 0;
  }

  s1=s2=0;
  // make DC prediction 直流DC预测
  for (i=0; i < MB_BLOCK_SIZE; i++)
  {
    if (up_avail)
      s1 += imgY_pred[up.pos_y][up.pos_x+i];    // sum hor pix 水平方向像素和
    if (left_avail)
      s2 += imgY_pred[left[i+1].pos_y][left[i+1].pos_x];    // sum vert pix 垂直方向像素和
  }
  if (up_avail && left_avail)
    s0=(s1+s2+16)/(2*MB_BLOCK_SIZE);             // no edge
  if (!up_avail && left_avail)
    s0=(s2+8)/MB_BLOCK_SIZE;                     // upper edge
  if (up_avail && !left_avail)
    s0=(s1+8)/MB_BLOCK_SIZE;                     // left edge
  if (!up_avail && !left_avail)
    s0=128;                                      // top left corner, nothing to predict from

  for (i=0; i < MB_BLOCK_SIZE; i++)
  {
    // vertical prediction
    if (up_avail)
      s[i][0]=imgY_pred[up.pos_y][up.pos_x+i];//可以放到上面优化速度
    // horizontal prediction
    if (left_avail)
      s[i][1]=imgY_pred[left[i+1].pos_y][left[i+1].pos_x];//和 left[i+1].pos_y][left[1].pos_x 一样
  }

  for (j=0; j < MB_BLOCK_SIZE; j++)
  {
    for (i=0; i < MB_BLOCK_SIZE; i++)
    {
      img->mprr_2[VERT_PRED_16][j][i]=s[i][0]; // store vertical prediction垂直预测
      img->mprr_2[HOR_PRED_16 ][j][i]=s[j][1]; // store horizontal prediction 水平预测
      img->mprr_2[DC_PRED_16  ][j][i]=s0;      // store DC prediction 平均值
    }
  }
  if (!up_avail || !left_avail || !left_up_avail) // edge 这句话可以方在最前面
    return;

  // 16 bit integer plan pred

  ih=0;
  iv=0;
  for (i=1;i<9;i++)
  {
    if (i<8)
      ih += i*(imgY_pred[up.pos_y][up.pos_x+7+i] - imgY_pred[up.pos_y][up.pos_x+7-i]);
    else
      ih += i*(imgY_pred[up.pos_y][up.pos_x+7+i] - imgY_pred[left[0].pos_y][left[0].pos_x]);
    
    iv += i*(imgY_pred[left[8+i].pos_y][left[8+i].pos_x] - imgY_pred[left[8-i].pos_y][left[8-i].pos_x]);// 8是否应该改为7
  }
  ib=(5*ih+32)>>6;
  ic=(5*iv+32)>>6;
  
  iaa=16*(imgY_pred[up.pos_y][up.pos_x+15]+imgY_pred[left[16].pos_y][left[16].pos_x]);

  for (j=0;j< MB_BLOCK_SIZE;j++)
  {
    for (i=0;i< MB_BLOCK_SIZE;i++)
    {
      img->mprr_2[PLANE_16][j][i]=max(0,min(255,(iaa+(i-7)*ib +(j-7)*ic + 16)/32));// store plane prediction
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    For new intra pred routines
 *
 * \par Input:
 *    Image par, 16x16 based intra mode
 *
 * \par Output:
 *    none
 ************************************************************************
 */
 //根据预测模式 计算残差 DCT 量化
int dct_luma_16x16(int new_intra_mode)
{
  int qp_const;
  int i,j;
  int ii,jj;
  int i1,j1;
  int M1[16][16];
  int M4[4][4];
  int M5[4],M6[4];
  int M0[4][4][4][4];
  int run,scan_pos,coeff_ctr,level;
  int qp_per,qp_rem,q_bits;
  int ac_coef = 0;

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  int   b8, b4;
  int*  DCLevel = img->cofDC[0][0];//DC哈达玛变换后的幅值
  int*  DCRun   = img->cofDC[0][1];//中间间隔零数
  int*  ACLevel;
  int*  ACRun;

  qp_per    = (currMB->qp-MIN_QP)/6;//qp除以6的分数
  qp_rem    = (currMB->qp-MIN_QP)%6;//qp除以6的余数

  q_bits    = Q_BITS+qp_per;//(15+QP/6)
  
  qp_const  = (1<<q_bits)/3;//(帧内预测)量化区间偏移量 增大可以减小死区

  for (j=0;j<16;j++)
  {
    for (i=0;i<16;i++)
    {
    //img->mprr_2[new_intra_mode][j][i] 不同的预测模式里面保存着对应的预测值
      M1[i][j]=imgY_org[img->opix_y+j][img->opix_x+i]-img->mprr_2[new_intra_mode][j][i];//宏块残差数据！！！
      M0[i%4][i/4][j%4][j/4]=M1[i][j];//把残差数据的坐标做一个分割映射
    }
  }
/*
	(i/4,j/4)是划分的4*4块，以当前宏块左上角为原点，4*4像素的块坐标
	(i%4,j%4)是划分的4*4块内部，以4*4子块左上角为原点，的像素坐标
	0000 0010 0020 0030 0001 0011 0021 0031 0002 0012 0022 0032 0003 0013 0023 0033
	1000 1010 1020 1030 1001 1011 1021 1031
	2000 2010 2020 2030 2001 2011 2021 2031
	3000 3010 3020 3030 3001 3011 3021 3031
	
	0100 0110 0120 0130 0101 0111 0121 0131
	1100 1110 1120 1130 1101 1111 1121 1131
	2100
	3100
	0200
	0300
  */
  //蝶形运算 下面进行变换 行变换和列变换
  for (jj=0;jj<4;jj++)//jj 表示4*4大块的行号
  {
    for (ii=0;ii<4;ii++)//ii 表示4*4大块的列号       	ii,JJ两个循环是确定一个4*4子块
    {
      for (j=0;j<4;j++)//j 表示块内的行号 选中4*4子块内某一行
      {
        for (i=0;i<2;i++)
        {
          i1=3-i;
          M5[i]=  M0[i][ii][j][jj]+M0[i1][ii][j][jj];//0000+3000 第0列加第3-i列 
          M5[i1]= M0[i][ii][j][jj]-M0[i1][ii][j][jj];//0000-3000 第0列减第3-i列
        }
		/*
		M5[0]、M5[1]、M5[2]、M5[3] 是中间变量
		M5[0]=第0列+第3列 M5[1]=第1列+第2列
		M5[3]=第0列-第3列 M5[2]=第1列-第2列
		*/
        M0[0][ii][j][jj]=M5[0]+M5[1];//第0、1、2、3列相加
        M0[2][ii][j][jj]=M5[0]-M5[1];//
        M0[1][ii][j][jj]=M5[3]*2+M5[2];
        M0[3][ii][j][jj]=M5[3]-M5[2]*2;
      }
      // vertical 另外一个方向 每一列做一维变换，再对其结果的每一行做一维变换
      for (i=0;i<4;i++)//i 表示块内的列号
      {
        for (j=0;j<2;j++)//
        {
          j1=3-j;
          M5[j] = M0[i][ii][j][jj]+M0[i][ii][j1][jj];//0000+0030 第0行加第3-i行 
          M5[j1]= M0[i][ii][j][jj]-M0[i][ii][j1][jj];//0000-0030 第0行减第3-i行 
        }
		/*
		M5[0]、M5[1]、M5[2]、M5[3] 是中间变量
		*/
		//最终变换结果
        M0[i][ii][0][jj]=M5[0]+M5[1];
        M0[i][ii][2][jj]=M5[0]-M5[1];
        M0[i][ii][1][jj]=M5[3]*2+M5[2];
        M0[i][ii][3][jj]=M5[3] -M5[2]*2;
      }
    }
  }
  
//以上得到的M0[][][]就是DCT变换后的结果

  // pick out DC coeff

  for (j=0;j<4;j++)
    for (i=0;i<4;i++)
      M4[i][j]= M0[0][i][0][j];// 取出4*4块左上的DC值
      
//4*4直流系数哈达玛变换
  for (j=0;j<4;j++)
  {
    for (i=0;i<2;i++)
    {
      i1=3-i;
      M5[i]= M4[i][j]+M4[i1][j];//第0列+第3列
      M5[i1]= M4[i][j]-M4[i1][j];//第0列-第3列
    }
	//M5[0] M5[1] M5[2] M5[3]为中间变量
    M4[0][j]=M5[0]+M5[1];
    M4[2][j]=M5[0]-M5[1];
    M4[1][j]=M5[3]+M5[2];
    M4[3][j]=M5[3]-M5[2];
  }

  // vertical

  for (i=0;i<4;i++)
  {
    for (j=0;j<2;j++)
    {
      j1=3-j;
      M5[j]= M4[i][j]+M4[i][j1];
      M5[j1]=M4[i][j]-M4[i][j1];
    }
    M4[i][0]=(M5[0]+M5[1])>>1;//为什么要除以2
    M4[i][2]=(M5[0]-M5[1])>>1;
    M4[i][1]=(M5[3]+M5[2])>>1;
    M4[i][3]=(M5[3]-M5[2])>>1;
  }

  // quant DC 量化

  run=-1;
  scan_pos=0;
 //zigzag扫描
  for (coeff_ctr=0;coeff_ctr<16;coeff_ctr++)
  {
    if (img->field_picture || ( mb_adaptive && img->field_mode )) 
    {  // Alternate scan for field coding
        i=FIELD_SCAN[coeff_ctr][0];//P109页
        j=FIELD_SCAN[coeff_ctr][1];
    }
    else 
    {
        i=SNGL_SCAN[coeff_ctr][0];
        j=SNGL_SCAN[coeff_ctr][1];
    }

    run++;
// DC哈达玛变换后的量化 level即量化后的值 与原始值M4有失真
    level= (abs(M4[i][j]) * quant_coef[qp_rem][0][0] + 2*qp_const)>>(q_bits+1);// p103公式
//游程编码
    if (level != 0)
    {
      DCLevel[scan_pos] = sign(level,M4[i][j]);//根据M4[i][j]的符合给level加上符合
      DCRun  [scan_pos] = run;//level前连续零的个数
      ++scan_pos;
      run=-1;//遇到level非零有值后进行重新置数
    }
    M4[i][j]=sign(level,M4[i][j]);//？？此时等式两边的M4[i][j]值不相同，存在了量化误差
  }
  DCLevel[scan_pos]=0;//最后的停止标志位？

  // invers DC transform  DC反变换
  for (j=0;j<4;j++)
  {
    for (i=0;i<4;i++)
      M5[i]=M4[i][j];

    M6[0]=M5[0]+M5[2];
    M6[1]=M5[0]-M5[2];
    M6[2]=M5[1]-M5[3];
    M6[3]=M5[1]+M5[3];

    for (i=0;i<2;i++)
    {
      i1=3-i;
      M4[i][j]= M6[i]+M6[i1];
      M4[i1][j]=M6[i]-M6[i1];
    }
  }

  for (i=0;i<4;i++)
  {
    for (j=0;j<4;j++)
      M5[j]=M4[i][j];

    M6[0]=M5[0]+M5[2];
    M6[1]=M5[0]-M5[2];
    M6[2]=M5[1]-M5[3];
    M6[3]=M5[1]+M5[3];

    for (j=0;j<2;j++)
    {
      j1=3-j;
      M0[0][i][0][j] = (((M6[j]+M6[j1])*dequant_coef[qp_rem][0][0]<<qp_per)+2)>>2;
      M0[0][i][0][j1]= (((M6[j]-M6[j1])*dequant_coef[qp_rem][0][0]<<qp_per)+2)>>2;
    }
  }

  // AC invers trans/quant for MB  AC量化
  for (jj=0;jj<4;jj++)
  {
    for (ii=0;ii<4;ii++)
    {
      run      = -1;
      scan_pos =  0;
	  /*
		b8
			0 0 2 2
			0 0 2 2
			1 1 3 3
			1 1 3 3
		b4
			0 2 0 2
			1 3 1 3
			0 2 0 2
			1 3 1 3
	  */
      b8       = 2*(jj/2) + (ii/2);//???
      b4       = 2*(jj%2) + (ii%2);//
      ACLevel  = img->cofAC [b8][b4][0];
      ACRun    = img->cofAC [b8][b4][1];

      for (coeff_ctr=1;coeff_ctr<16;coeff_ctr++) // set in AC coeff 1到15
      {

        if (img->field_picture || ( mb_adaptive && img->field_mode )) 
        {  // Alternate scan for field coding
          i=FIELD_SCAN[coeff_ctr][0];
          j=FIELD_SCAN[coeff_ctr][1];
        }
        else 
        {
          i=SNGL_SCAN[coeff_ctr][0];
          j=SNGL_SCAN[coeff_ctr][1];
        }
        run++;

		//15个AC系数逐个量化
        level= ( abs( M0[i][ii][j][jj]) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;
		//游程编码
        if (level != 0)
        {
          ac_coef = 15;
          ACLevel[scan_pos] = sign(level,M0[i][ii][j][jj]);
          ACRun  [scan_pos] = run;
          ++scan_pos;
          run=-1;
        }
        M0[i][ii][j][jj]=sign(level*dequant_coef[qp_rem][i][j]<<qp_per,M0[i][ii][j][jj]);//反量化
      }
      ACLevel[scan_pos] = 0;


      // IDCT horizontal  AC 反变换

      for (j=0;j<4;j++)
      {
        for (i=0;i<4;i++)
        {
          M5[i]=M0[i][ii][j][jj];
        }

        M6[0]= M5[0]+M5[2];
        M6[1]= M5[0]-M5[2];
        M6[2]=(M5[1]>>1) -M5[3];
        M6[3]= M5[1]+(M5[3]>>1);

        for (i=0;i<2;i++)
        {
          i1=3-i;
          M0[i][ii][j][jj] =M6[i]+M6[i1];
          M0[i1][ii][j][jj]=M6[i]-M6[i1];
        }
      }

      // vert
      for (i=0;i<4;i++)
      {
        for (j=0;j<4;j++)
          M5[j]=M0[i][ii][j][jj];

        M6[0]= M5[0]+M5[2];
        M6[1]= M5[0]-M5[2];
        M6[2]=(M5[1]>>1) -M5[3];
        M6[3]= M5[1]+(M5[3]>>1);

        for (j=0;j<2;j++)
        {
          j1=3-j;
          M0[i][ii][ j][jj]=M6[j]+M6[j1];
          M0[i][ii][j1][jj]=M6[j]-M6[j1];

        }
      }

    }
  }

  for (j=0;j<16;j++)
  {
    for (i=0;i<16;i++)
    {
      M1[i][j]=M0[i%4][i/4][j%4][j/4];//M1 为经历变换 量化 反量化 反变换后的残差值
    }
  }
	//重构图像像素
	//M1[i][j]=imgY_org[img->opix_y+j][img->opix_x+i]-img->mprr_2[new_intra_mode][j][i];//残差数据！！！
	//这个快的数据经过滤波后将被存储起来预测下一帧的宏块数据
  for (j=0;j<16;j++)
    for (i=0;i<16;i++)
      enc_picture->imgY[img->pix_y+j][img->pix_x+i]=(byte)min(255,max(0,(M1[i][j]+(img->mprr_2[new_intra_mode][j][i]<<DQ_BITS)+DQ_ROUND)>>DQ_BITS));

    return ac_coef;
}


/*!
 ************************************************************************
 * \brief
 *    The routine(常规) performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output_
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.             \n
 *    coeff_cost: Counter for nonzero coefficients, used to discard expencive levels.
 ************************************************************************
 */
int dct_luma(int block_x,int block_y,int *coeff_cost, int old_intra_mode)
{
  int sign(int a,int b);

  int i,j,i1,j1,ilev,m5[4],m6[4],coeff_ctr;
  int qp_const,level,scan_pos,run;
  int nonzero; 
  int qp_per,qp_rem,q_bits;

  int   pos_x   = block_x/BLOCK_SIZE;//block_x是块左上角的像素坐标，pos_x是块坐标
  int   pos_y   = block_y/BLOCK_SIZE;
  int   b8      = 2*(pos_y/2) + (pos_x/2);//？？？
  int   b4      = 2*(pos_y%2) + (pos_x%2);
  int*  ACLevel = img->cofAC[b8][b4][0];
  int*  ACRun   = img->cofAC[b8][b4][1];

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  qp_per    = (currMB->qp-MIN_QP)/6;
  qp_rem    = (currMB->qp-MIN_QP)%6;
  q_bits    = Q_BITS+qp_per;

  if (img->type == I_SLICE)
    qp_const=(1<<q_bits)/3;    // intra
  else
    qp_const=(1<<q_bits)/6;    // inter

  //  Horizontal transform  水平方向变换 蝶形运算
  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < 2; i++)
    {
      i1=3-i;
      m5[i]=img->m7[i][j]+img->m7[i1][j];//m7 直接存储的是原始图像和预测图像的残差数据 ？
      m5[i1]=img->m7[i][j]-img->m7[i1][j];
    }
    img->m7[0][j]=(m5[0]+m5[1]);
    img->m7[2][j]=(m5[0]-m5[1]);
    img->m7[1][j]=m5[3]*2+m5[2];
    img->m7[3][j]=m5[3]-m5[2]*2;
  }

  //  Vertical transform
  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      m5[j]=img->m7[i][j]+img->m7[i][j1];
      m5[j1]=img->m7[i][j]-img->m7[i][j1];
    }
    img->m7[i][0]=(m5[0]+m5[1]);
    img->m7[i][2]=(m5[0]-m5[1]);
    img->m7[i][1]=m5[3]*2+m5[2];
    img->m7[i][3]=m5[3]-m5[2]*2;//4*4的残差数据的DCT变换结果 这里为什么接着把直流系数提取出来？
  }

  // Quant

  nonzero=FALSE;

  run=-1;
  scan_pos=0;
  //zigzag扫描
  for (coeff_ctr=0;coeff_ctr < 16;coeff_ctr++)//0到15  直流系数怎么办？
  {

    if (img->field_picture || ( img->MbaffFrameFlag && currMB->mb_field )) 
    {  // Alternate scan for field coding
        i=FIELD_SCAN[coeff_ctr][0];
        j=FIELD_SCAN[coeff_ctr][1];
    }
    else 
    {
        i=SNGL_SCAN[coeff_ctr][0];
        j=SNGL_SCAN[coeff_ctr][1];
    }
    
    run++;
    ilev=0;
    //量化
    level = (abs (img->m7[i][j]) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;
    
    if (level != 0)
    {
      nonzero=TRUE;//非零系数标志位置1
      // ???
      if (level > 1)
        *coeff_cost += MAX_VALUE;                // set high cost, shall（将） not be discarded
      else
        *coeff_cost += COEFF_COST[run];//run越大 即中间间隔的零越多 “系数成本”越低 ？
      ACLevel[scan_pos] = sign(level,img->m7[i][j]);
      ACRun  [scan_pos] = run;
      ++scan_pos;
      run=-1;                     // reset zero level counter
      ilev=level*dequant_coef[qp_rem][i][j]<<qp_per;// 变换 量化 反量化后的值
    }
    img->m7[i][j]=sign(ilev,img->m7[i][j]);
  }
  ACLevel[scan_pos] = 0;
  
  
  //     IDCT.
  //     horizontal DCT反变换

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < BLOCK_SIZE; i++)
    {
      m5[i]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (i=0; i < 2; i++)
    {
      i1=3-i;
      img->m7[i][j]=m6[i]+m6[i1];
      img->m7[i1][j]=m6[i]-m6[i1];
    }
  }

  //  vertical

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < BLOCK_SIZE; j++)
    {
      m5[j]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);
    //重构像素
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      img->m7[i][j] =min(255,max(0,(m6[j]+m6[j1]+(img->mpr[i+block_x][j+block_y] <<DQ_BITS)+DQ_ROUND)>>DQ_BITS));
      img->m7[i][j1]=min(255,max(0,(m6[j]-m6[j1]+(img->mpr[i+block_x][j1+block_y]<<DQ_BITS)+DQ_ROUND)>>DQ_BITS));
    }
  }

  //  Decoded block moved to frame memory

  for (j=0; j < BLOCK_SIZE; j++)
    for (i=0; i < BLOCK_SIZE; i++)
      enc_picture->imgY[img->pix_y+block_y+j][img->pix_x+block_x+i]=img->m7[i][j];


  return nonzero;//如果判断到该块系数全0 既可以skip 提高压缩比
}



/*!
 ************************************************************************
 * \brief
 *    Transform,quantization,inverse transform for chroma.
 *    The main reason why this is done in a separate routine is the
 *    additional 2x2 transform of DC-coeffs. This routine is called
 *    ones for each of the chroma components.
 *
 * \par Input:
 *    uv    : Make difference between the U and V chroma component  \n
 *    cr_cbp: chroma coded block pattern
 *
 * \par Output:
 *    cr_cbp: Updated chroma coded block pattern.
 ************************************************************************
 */
int dct_chroma(int uv,int cr_cbp)
{
  int i,j,i1,j2,ilev,n2,n1,j1,mb_y,coeff_ctr,qp_const,level ,scan_pos,run;
  int m1[BLOCK_SIZE],m5[BLOCK_SIZE],m6[BLOCK_SIZE];
  int coeff_cost;
  int cr_cbp_tmp;
  int nn0,nn1;
  int DCcoded=0 ;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  int qp_per,qp_rem,q_bits;

  int   b4;
  int*  DCLevel = img->cofDC[uv+1][0];
  int*  DCRun   = img->cofDC[uv+1][1];
  int*  ACLevel;
  int*  ACRun;

  int qpChroma=Clip3(0, 51, currMB->qp + active_pps->chroma_qp_index_offset);//色度的QP 是当前宏块QP加一个偏移值
  
  qp_per    = QP_SCALE_CR[qpChroma-MIN_QP]/6;//色度的QP范围是0到39 重0到52做一个映射到39
  qp_rem    = QP_SCALE_CR[qpChroma-MIN_QP]%6;
  q_bits    = Q_BITS+qp_per;

  if (img->type == I_SLICE)
    qp_const=(1<<q_bits)/3;    // intra 帧内量化偏移
  else
    qp_const=(1<<q_bits)/6;    // inter 帧间量化偏移

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)//这个地方是不是有问题？ 应该是n2++
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {

      //  Horizontal transform.水平方向变换
      for (j=0; j < BLOCK_SIZE; j++)
      {
        mb_y=n2+j;//??和上面不一样
        for (i=0; i < 2; i++)
        {
          i1=3-i;
          m5[i]=img->m7[i+n1][mb_y]+img->m7[i1+n1][mb_y];
          m5[i1]=img->m7[i+n1][mb_y]-img->m7[i1+n1][mb_y];
        }
        img->m7[n1][mb_y]  =(m5[0]+m5[1]);
        img->m7[n1+2][mb_y]=(m5[0]-m5[1]);
        img->m7[n1+1][mb_y]=m5[3]*2+m5[2];
        img->m7[n1+3][mb_y]=m5[3]-m5[2]*2;
      }

      //  Vertical transform.

      for (i=0; i < BLOCK_SIZE; i++)
      {
        j1=n1+i;
        for (j=0; j < 2; j++)
        {
          j2=3-j;
          m5[j]=img->m7[j1][n2+j]+img->m7[j1][n2+j2];
          m5[j2]=img->m7[j1][n2+j]-img->m7[j1][n2+j2];
        }
        img->m7[j1][n2+0]=(m5[0]+m5[1]);
        img->m7[j1][n2+2]=(m5[0]-m5[1]);
        img->m7[j1][n2+1]=m5[3]*2+m5[2];
        img->m7[j1][n2+3]=m5[3]-m5[2]*2;
      }
    }
  }

  //     2X2 transform of DC coeffs.色度哈达玛变换
  m1[0]=(img->m7[0][0]+img->m7[4][0]+img->m7[0][4]+img->m7[4][4]);
  m1[1]=(img->m7[0][0]-img->m7[4][0]+img->m7[0][4]-img->m7[4][4]);
  m1[2]=(img->m7[0][0]+img->m7[4][0]-img->m7[0][4]-img->m7[4][4]);
  m1[3]=(img->m7[0][0]-img->m7[4][0]-img->m7[0][4]+img->m7[4][4]);

  //     Quant of chroma 2X2 coeffs.
  run=-1;
  scan_pos=0;

  for (coeff_ctr=0; coeff_ctr < 4; coeff_ctr++)
  {
    run++;
    ilev=0;
	//量化
    level =(abs(m1[coeff_ctr]) * quant_coef[qp_rem][0][0] + 2*qp_const) >> (q_bits+1);

    if (level  != 0)
    {
      currMB->cbp_blk |= 0xf0000 << (uv << 2) ;    // if one of the 2x2-DC levels is != 0 set the ？不懂
      cr_cbp=max(1,cr_cbp);                     // coded-bit all 4 4x4 blocks (bit 16-19 or 20-23)
      DCcoded = 1 ;//标志位
      DCLevel[scan_pos] = sign(level ,m1[coeff_ctr]);
      DCRun  [scan_pos] = run;
      scan_pos++;
      run=-1;
      ilev=level*dequant_coef[qp_rem][0][0]<<qp_per;//反量化
    }
    m1[coeff_ctr]=sign(ilev,m1[coeff_ctr]);
  }
  DCLevel[scan_pos] = 0;

  //  Invers transform of 2x2 DC levels DC反变换

  img->m7[0][0]=(m1[0]+m1[1]+m1[2]+m1[3])>>1;
  img->m7[4][0]=(m1[0]-m1[1]+m1[2]-m1[3])>>1;
  img->m7[0][4]=(m1[0]+m1[1]-m1[2]-m1[3])>>1;
  img->m7[4][4]=(m1[0]-m1[1]-m1[2]+m1[3])>>1;

  //     Quant of chroma AC-coeffs. AC量化
  coeff_cost=0;
  cr_cbp_tmp=0;

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      b4      = 2*(n2/4) + (n1/4);
      ACLevel = img->cofAC[uv+4][b4][0];
      ACRun   = img->cofAC[uv+4][b4][1];
      run=-1;
      scan_pos=0;

      for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// start change rd_quant
      {

        if (img->field_picture || ( img->MbaffFrameFlag && currMB->mb_field )) 
        {  // Alternate scan for field coding
          i=FIELD_SCAN[coeff_ctr][0];
          j=FIELD_SCAN[coeff_ctr][1];
        }
        else 
        {
          i=SNGL_SCAN[coeff_ctr][0];
          j=SNGL_SCAN[coeff_ctr][1];
        }
        ++run;
        ilev=0;

        level=(abs(img->m7[n1+i][n2+j])*quant_coef[qp_rem][i][j]+qp_const)>>q_bits;

        if (level  != 0)
        {
          currMB->cbp_blk |= 1 << (16 + (uv << 2) + ((n2 >> 1) + (n1 >> 2))) ;
          if (level > 1)
            coeff_cost += MAX_VALUE;                // set high cost, shall not be discarded
          else
            coeff_cost += COEFF_COST[run];

          cr_cbp_tmp=2;
          ACLevel[scan_pos] = sign(level,img->m7[n1+i][n2+j]);
          ACRun  [scan_pos] = run;
          ++scan_pos;
          run=-1;
          ilev=level*dequant_coef[qp_rem][i][j]<<qp_per;//AC反量化
        }
        img->m7[n1+i][n2+j]=sign(ilev,img->m7[n1+i][n2+j]); // for use in IDCT
      }
      ACLevel[scan_pos] = 0;
    }
  }

  // * reset chroma coeffs AC DCT反变换
  if(coeff_cost < _CHROMA_COEFF_COST_)
  {
    cr_cbp_tmp = 0 ;
    for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
    {
      for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
      {
        b4      = 2*(n2/4) + (n1/4);
        ACLevel = img->cofAC[uv+4][b4][0];
        ACRun   = img->cofAC[uv+4][b4][1];
        if( DCcoded == 0) currMB->cbp_blk &= ~(0xf0000 << (uv << 2));  // if no chroma DC's: then reset coded-bits of this chroma subblock
        nn0 = (n1>>2) + (uv<<1);
        nn1 = 4 + (n2>>2) ;
        ACLevel[0] = 0;
        for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// ac coeff
        {

          if (img->field_picture || ( img->MbaffFrameFlag && currMB->mb_field )) 
          {  // Alternate scan for field coding
            i=FIELD_SCAN[coeff_ctr][0];
            j=FIELD_SCAN[coeff_ctr][1];
          }
          else 
          {
            i=SNGL_SCAN[coeff_ctr][0];
            j=SNGL_SCAN[coeff_ctr][1];
          }
          img->m7[n1+i][n2+j]=0;
          ACLevel[coeff_ctr] = 0;
        }
      }
    }
  }
  if(cr_cbp_tmp==2)
    cr_cbp = 2;
  //     IDCT.

      //     Horizontal.
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      for (j=0; j < BLOCK_SIZE; j++)
      {
        for (i=0; i < BLOCK_SIZE; i++)
        {
          m5[i]=img->m7[n1+i][n2+j];
        }
        m6[0]=(m5[0]+m5[2]);
        m6[1]=(m5[0]-m5[2]);
        m6[2]=(m5[1]>>1)-m5[3];
        m6[3]=m5[1]+(m5[3]>>1);

        for (i=0; i < 2; i++)
        {
          i1=3-i;
          img->m7[n1+i][n2+j]=m6[i]+m6[i1];
          img->m7[n1+i1][n2+j]=m6[i]-m6[i1];
        }
      }

      //     Vertical.
      for (i=0; i < BLOCK_SIZE; i++)
      {
        for (j=0; j < BLOCK_SIZE; j++)
        {
          m5[j]=img->m7[n1+i][n2+j];
        }
        m6[0]=(m5[0]+m5[2]);
        m6[1]=(m5[0]-m5[2]);
        m6[2]=(m5[1]>>1)-m5[3];
        m6[3]=m5[1]+(m5[3]>>1);

        for (j=0; j < 2; j++)
        {
          j2=3-j;
          img->m7[n1+i][n2+j] =min(255,max(0,(m6[j]+m6[j2]+(img->mpr[n1+i][n2+j] <<DQ_BITS)+DQ_ROUND)>>DQ_BITS));
          img->m7[n1+i][n2+j2]=min(255,max(0,(m6[j]-m6[j2]+(img->mpr[n1+i][n2+j2]<<DQ_BITS)+DQ_ROUND)>>DQ_BITS));
        }
      }
    }
  }

  //  Decoded block moved to memory u,v分量分别重构存储 8*8
  for (j=0; j < BLOCK_SIZE*2; j++)
    for (i=0; i < BLOCK_SIZE*2; i++)
      enc_picture->imgUV[uv][img->pix_c_y+j][img->pix_c_x+i]= img->m7[i][j];

  return cr_cbp;
}


/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.              \n
 *    coeff_cost: Counter for nonzero coefficients, used to discard expencive levels.
 *
 *
 ************************************************************************
 */
int dct_luma_sp(int block_x,int block_y,int *coeff_cost)
{
  int sign(int a,int b);

  int i,j,i1,j1,ilev,m5[4],m6[4],coeff_ctr;
  int qp_const,level,scan_pos,run;
  int nonzero;

  int predicted_block[BLOCK_SIZE][BLOCK_SIZE],c_err,qp_const2;
  int qp_per,qp_rem,q_bits;
  int qp_per_sp,qp_rem_sp,q_bits_sp;

  int   pos_x   = block_x/BLOCK_SIZE;
  int   pos_y   = block_y/BLOCK_SIZE;
  int   b8      = 2*(pos_y/2) + (pos_x/2);
  int   b4      = 2*(pos_y%2) + (pos_x%2);
  int*  ACLevel = img->cofAC[b8][b4][0];
  int*  ACRun   = img->cofAC[b8][b4][1];
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  // For encoding optimization
  int c_err1, c_err2, level1, level2;
  double D_dis1, D_dis2;
  int len, info;
  double lambda_mode   = 0.85 * pow (2, (currMB->qp - SHIFT_QP)/3.0) * 4; 

  qp_per    = (currMB->qp-MIN_QP)/6;
  qp_rem    = (currMB->qp-MIN_QP)%6;
  q_bits    = Q_BITS+qp_per;
  qp_per_sp    = (currMB->qpsp-MIN_QP)/6;
  qp_rem_sp    = (currMB->qpsp-MIN_QP)%6;
  q_bits_sp    = Q_BITS+qp_per_sp;

  qp_const=(1<<q_bits)/6;    // inter
  qp_const2=(1<<q_bits_sp)/2;  //sp_pred

  //  Horizontal transform
  for (j=0; j< BLOCK_SIZE; j++)
    for (i=0; i< BLOCK_SIZE; i++)
    {
      img->m7[i][j]+=img->mpr[i+block_x][j+block_y];
      predicted_block[i][j]=img->mpr[i+block_x][j+block_y];
    }

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < 2; i++)
    {
      i1=3-i;
      m5[i]=img->m7[i][j]+img->m7[i1][j];
      m5[i1]=img->m7[i][j]-img->m7[i1][j];
    }
    img->m7[0][j]=(m5[0]+m5[1]);
    img->m7[2][j]=(m5[0]-m5[1]);
    img->m7[1][j]=m5[3]*2+m5[2];
    img->m7[3][j]=m5[3]-m5[2]*2;
  }

  //  Vertival transform

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      m5[j]=img->m7[i][j]+img->m7[i][j1];
      m5[j1]=img->m7[i][j]-img->m7[i][j1];
    }
    img->m7[i][0]=(m5[0]+m5[1]);
    img->m7[i][2]=(m5[0]-m5[1]);
    img->m7[i][1]=m5[3]*2+m5[2];
    img->m7[i][3]=m5[3]-m5[2]*2;
  }

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < 2; i++)
    {
      i1=3-i;
      m5[i]=predicted_block[i][j]+predicted_block[i1][j];
      m5[i1]=predicted_block[i][j]-predicted_block[i1][j];
    }
    predicted_block[0][j]=(m5[0]+m5[1]);
    predicted_block[2][j]=(m5[0]-m5[1]);
    predicted_block[1][j]=m5[3]*2+m5[2];
    predicted_block[3][j]=m5[3]-m5[2]*2;
  }

  //  Vertival transform

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      m5[j]=predicted_block[i][j]+predicted_block[i][j1];
      m5[j1]=predicted_block[i][j]-predicted_block[i][j1];
    }
    predicted_block[i][0]=(m5[0]+m5[1]);
    predicted_block[i][2]=(m5[0]-m5[1]);
    predicted_block[i][1]=m5[3]*2+m5[2];
    predicted_block[i][3]=m5[3]-m5[2]*2;
  }

  // Quant
  nonzero=FALSE;

  run=-1;
  scan_pos=0;
  
  for (coeff_ctr=0;coeff_ctr < 16;coeff_ctr++)     // 8 times if double scan, 16 normal scan
  {

    if (img->field_picture || ( mb_adaptive && img->field_mode )) 
    {  // Alternate scan for field coding
        i=FIELD_SCAN[coeff_ctr][0];
        j=FIELD_SCAN[coeff_ctr][1];
    }
    else 
    {
        i=SNGL_SCAN[coeff_ctr][0];
        j=SNGL_SCAN[coeff_ctr][1];
    }
    
    run++;
    ilev=0;
    
    // decide prediction
    
    // case 1
    level1 = (abs (predicted_block[i][j]) * quant_coef[qp_rem_sp][i][j] + qp_const2) >> q_bits_sp; 
    level1 = (level1 << q_bits_sp) / quant_coef[qp_rem_sp][i][j];                 
    c_err1 = img->m7[i][j]-sign(level1, predicted_block[i][j]);                   
    level1 = (abs (c_err1) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;
    
    // case 2
    c_err2=img->m7[i][j]-predicted_block[i][j];
    level2 = (abs (c_err2) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;
    
    // select prediction
    if ((level1 != level2) && (level1 != 0) && (level2 != 0))
    {
      D_dis1 = img->m7[i][j] - ((sign(level1,c_err1)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6) - predicted_block[i][j]; 
      levrun_linfo_inter(level1, run, &len, &info);
      D_dis1 = D_dis1*D_dis1 + lambda_mode * len;
      
      D_dis2 = img->m7[i][j] - ((sign(level2,c_err2)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6) - predicted_block[i][j]; 
      levrun_linfo_inter(level2, run, &len, &info);
      D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;
      
      if (D_dis1 == D_dis2)
        level = (abs(level1) < abs(level2)) ? level1 : level2;
      else
      {
        if (D_dis1 < D_dis2)
          level = level1;
        else
          level = level2;
      }
      c_err = (level == level1) ? c_err1 : c_err2;
    }
    else if (level1 == level2)
    {
      level = level1;
      c_err = c_err1;
    }
    else
    {
      level = (level1 == 0) ? level1 : level2;
      c_err = (level1 == 0) ? c_err1 : c_err2;
    }
    
    if (level != 0)
    {
      nonzero=TRUE;
      if (level > 1)
        *coeff_cost += MAX_VALUE;                // set high cost, shall not be discarded
      else
        *coeff_cost += COEFF_COST[run];
      ACLevel[scan_pos] = sign(level,c_err);
      ACRun  [scan_pos] = run;
      ++scan_pos;
      run=-1;                     // reset zero level counter
      ilev=((sign(level,c_err)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6);
    }
    ilev+=predicted_block[i][j] ; 
    img->m7[i][j] = sign((abs(ilev) * quant_coef[qp_rem_sp][i][j] + qp_const2)>> q_bits_sp, ilev) * dequant_coef[qp_rem_sp][i][j] << qp_per_sp;
  }
  ACLevel[scan_pos] = 0;
  
    
  //     IDCT.
  //     horizontal

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < BLOCK_SIZE; i++)
    {
      m5[i]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (i=0; i < 2; i++)
    {
      i1=3-i;
      img->m7[i][j]=m6[i]+m6[i1];
      img->m7[i1][j]=m6[i]-m6[i1];
    }
  }

  //  vertical

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < BLOCK_SIZE; j++)
    {
      m5[j]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (j=0; j < 2; j++)
    {
      j1=3-j;
      img->m7[i][j] =min(255,max(0,(m6[j]+m6[j1]+DQ_ROUND)>>DQ_BITS));
      img->m7[i][j1]=min(255,max(0,(m6[j]-m6[j1]+DQ_ROUND)>>DQ_BITS));
    }
  }

  //  Decoded block moved to frame memory

  for (j=0; j < BLOCK_SIZE; j++)
  for (i=0; i < BLOCK_SIZE; i++)
    enc_picture->imgY[img->pix_y+block_y+j][img->pix_x+block_x+i]=img->m7[i][j];

  return nonzero;
}

/*!
 ************************************************************************
 * \brief
 *    Transform,quantization,inverse transform for chroma.
 *    The main reason why this is done in a separate routine is the
 *    additional 2x2 transform of DC-coeffs. This routine is called
 *    ones for each of the chroma components.
 *
 * \par Input:
 *    uv    : Make difference between the U and V chroma component               \n
 *    cr_cbp: chroma coded block pattern
 *
 * \par Output:
 *    cr_cbp: Updated chroma coded block pattern.
 ************************************************************************
 */
int dct_chroma_sp(int uv,int cr_cbp)
{
  int i,j,i1,j2,ilev,n2,n1,j1,mb_y,coeff_ctr,qp_const,c_err,level ,scan_pos,run;
  int m1[BLOCK_SIZE],m5[BLOCK_SIZE],m6[BLOCK_SIZE];
  int coeff_cost;
  int cr_cbp_tmp;
  int predicted_chroma_block[MB_BLOCK_SIZE/2][MB_BLOCK_SIZE/2],qp_const2,mp1[BLOCK_SIZE];
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  int qp_per,qp_rem,q_bits;
  int qp_per_sp,qp_rem_sp,q_bits_sp;

  int   b4;
  int*  DCLevel = img->cofDC[uv+1][0];
  int*  DCRun   = img->cofDC[uv+1][1];
  int*  ACLevel;
  int*  ACRun;

  int c_err1, c_err2, level1, level2;
  int len, info;
  double D_dis1, D_dis2;
  double lambda_mode   = 0.85 * pow (2, (currMB->qp -SHIFT_QP)/3.0) * 4; 


  int qpChroma=Clip3(0, 51, currMB->qp + active_pps->chroma_qp_index_offset);
  int qpChromaSP=Clip3(0, 51, currMB->qpsp + active_pps->chroma_qp_index_offset);

  qp_per    = ((qpChroma<0?qpChroma:QP_SCALE_CR[qpChroma])-MIN_QP)/6;
  qp_rem    = ((qpChroma<0?qpChroma:QP_SCALE_CR[qpChroma])-MIN_QP)%6;
  q_bits    = Q_BITS+qp_per;
  qp_const=(1<<q_bits)/6;    // inter
  qp_per_sp    = ((qpChromaSP<0?currMB->qpsp:QP_SCALE_CR[qpChromaSP])-MIN_QP)/6;
  qp_rem_sp    = ((qpChromaSP<0?currMB->qpsp:QP_SCALE_CR[qpChromaSP])-MIN_QP)%6;
  q_bits_sp    = Q_BITS+qp_per_sp;
  qp_const2    = (1<<q_bits_sp)/2;  //sp_pred
  

  for (j=0; j < MB_BLOCK_SIZE/2; j++)
    for (i=0; i < MB_BLOCK_SIZE/2; i++)
    {
      img->m7[i][j]+=img->mpr[i][j];
      predicted_chroma_block[i][j]=img->mpr[i][j];
    }

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {

      //  Horizontal transform.
      for (j=0; j < BLOCK_SIZE; j++)
      {
        mb_y=n2+j;
        for (i=0; i < 2; i++)
        {
          i1=3-i;
          m5[i]=img->m7[i+n1][mb_y]+img->m7[i1+n1][mb_y];
          m5[i1]=img->m7[i+n1][mb_y]-img->m7[i1+n1][mb_y];
        }
        img->m7[n1][mb_y]  =(m5[0]+m5[1]);
        img->m7[n1+2][mb_y]=(m5[0]-m5[1]);
        img->m7[n1+1][mb_y]=m5[3]*2+m5[2];
        img->m7[n1+3][mb_y]=m5[3]-m5[2]*2;
      }

      //  Vertical transform.

      for (i=0; i < BLOCK_SIZE; i++)
      {
        j1=n1+i;
        for (j=0; j < 2; j++)
        {
          j2=3-j;
          m5[j]=img->m7[j1][n2+j]+img->m7[j1][n2+j2];
          m5[j2]=img->m7[j1][n2+j]-img->m7[j1][n2+j2];
        }
        img->m7[j1][n2+0]=(m5[0]+m5[1]);
        img->m7[j1][n2+2]=(m5[0]-m5[1]);
        img->m7[j1][n2+1]=m5[3]*2+m5[2];
        img->m7[j1][n2+3]=m5[3]-m5[2]*2;
      }
    }
  }
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {

      //  Horizontal transform.
      for (j=0; j < BLOCK_SIZE; j++)
      {
        mb_y=n2+j;
        for (i=0; i < 2; i++)
        {
          i1=3-i;
          m5[i]=predicted_chroma_block[i+n1][mb_y]+predicted_chroma_block[i1+n1][mb_y];
          m5[i1]=predicted_chroma_block[i+n1][mb_y]-predicted_chroma_block[i1+n1][mb_y];
        }
        predicted_chroma_block[n1][mb_y]  =(m5[0]+m5[1]);
        predicted_chroma_block[n1+2][mb_y]=(m5[0]-m5[1]);
        predicted_chroma_block[n1+1][mb_y]=m5[3]*2+m5[2];
        predicted_chroma_block[n1+3][mb_y]=m5[3]-m5[2]*2;
      }

      //  Vertical transform.

      for (i=0; i < BLOCK_SIZE; i++)
      {
        j1=n1+i;
        for (j=0; j < 2; j++)
        {
          j2=3-j;
          m5[j]=predicted_chroma_block[j1][n2+j]+predicted_chroma_block[j1][n2+j2];
          m5[j2]=predicted_chroma_block[j1][n2+j]-predicted_chroma_block[j1][n2+j2];
        }
        predicted_chroma_block[j1][n2+0]=(m5[0]+m5[1]);
        predicted_chroma_block[j1][n2+2]=(m5[0]-m5[1]);
        predicted_chroma_block[j1][n2+1]=m5[3]*2+m5[2];
        predicted_chroma_block[j1][n2+3]=m5[3]-m5[2]*2;
      }
    }
  }

  //     2X2 transform of DC coeffs.
  m1[0]=(img->m7[0][0]+img->m7[4][0]+img->m7[0][4]+img->m7[4][4]);
  m1[1]=(img->m7[0][0]-img->m7[4][0]+img->m7[0][4]-img->m7[4][4]);
  m1[2]=(img->m7[0][0]+img->m7[4][0]-img->m7[0][4]-img->m7[4][4]);
  m1[3]=(img->m7[0][0]-img->m7[4][0]-img->m7[0][4]+img->m7[4][4]);

  //     2X2 transform of DC coeffs.
  mp1[0]=(predicted_chroma_block[0][0]+predicted_chroma_block[4][0]+predicted_chroma_block[0][4]+predicted_chroma_block[4][4]);
  mp1[1]=(predicted_chroma_block[0][0]-predicted_chroma_block[4][0]+predicted_chroma_block[0][4]-predicted_chroma_block[4][4]);
  mp1[2]=(predicted_chroma_block[0][0]+predicted_chroma_block[4][0]-predicted_chroma_block[0][4]-predicted_chroma_block[4][4]);
  mp1[3]=(predicted_chroma_block[0][0]-predicted_chroma_block[4][0]-predicted_chroma_block[0][4]+predicted_chroma_block[4][4]);

  run=-1;
  scan_pos=0;

  for (coeff_ctr=0; coeff_ctr < 4; coeff_ctr++)
  {
    run++;
    ilev=0;

  // case 1
    c_err1 = (abs (mp1[coeff_ctr]) * quant_coef[qp_rem_sp][0][0] + 2 * qp_const2) >> (q_bits_sp + 1);
    c_err1 = (c_err1 << (q_bits_sp + 1)) / quant_coef[qp_rem_sp][0][0];
    c_err1 = m1[coeff_ctr] - sign(c_err1, mp1[coeff_ctr]);
    level1 = (abs(c_err1) * quant_coef[qp_rem][0][0] + 2 * qp_const) >> (q_bits+1);

  // case 2
    c_err2 = m1[coeff_ctr] - mp1[coeff_ctr];
    level2 = (abs(c_err2) * quant_coef[qp_rem][0][0] + 2 * qp_const) >> (q_bits+1);

  if (level1 != level2 && level1 != 0 && level2 != 0)
  {
    D_dis1 = m1[coeff_ctr] - ((sign(level1,c_err1)*dequant_coef[qp_rem][0][0]*A[0][0]<< qp_per) >>5)- mp1[coeff_ctr];
    levrun_linfo_c2x2(level1, run, &len, &info);
    D_dis1 = D_dis1 * D_dis1 + lambda_mode * len;

    D_dis2 = m1[coeff_ctr] - ((sign(level2,c_err2)*dequant_coef[qp_rem][0][0]*A[0][0]<< qp_per) >>5)- mp1[coeff_ctr];
    levrun_linfo_c2x2(level2, run, &len, &info);
    D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;

    if (D_dis1 == D_dis2)
      level = (abs(level1) < abs(level2)) ? level1 : level2;
    else
    {
      if (D_dis1 < D_dis2)
        level = level1;
      else
        level = level2;
    }
    c_err = (level == level1) ? c_err1 : c_err2;
  }
  else if (level1 == level2)
  {
    level = level1;
    c_err = c_err1;
  }
  else
  {
    level = (level1 == 0) ? level1 : level2;
    c_err = (level1 == 0) ? c_err1 : c_err2;
  }

    if (level  != 0)
    {
      currMB->cbp_blk |= 0xf0000 << (uv << 2) ;  // if one of the 2x2-DC levels is != 0 the coded-bit
      cr_cbp=max(1,cr_cbp);
      DCLevel[scan_pos] = sign(level ,c_err);
      DCRun  [scan_pos] = run;
      scan_pos++;
      run=-1;
      ilev=((sign(level,c_err)*dequant_coef[qp_rem][0][0]*A[0][0]<< qp_per) >>5);
    }
    ilev+= mp1[coeff_ctr];
    m1[coeff_ctr]=sign((abs(ilev)  * quant_coef[qp_rem_sp][0][0] + 2 * qp_const2) >> (q_bits_sp+1), ilev) * dequant_coef[qp_rem_sp][0][0] << qp_per_sp;
  }
  DCLevel[scan_pos] = 0;

  //  Invers transform of 2x2 DC levels

  img->m7[0][0]=(m1[0]+m1[1]+m1[2]+m1[3])/2;
  img->m7[4][0]=(m1[0]-m1[1]+m1[2]-m1[3])/2;
  img->m7[0][4]=(m1[0]+m1[1]-m1[2]-m1[3])/2;
  img->m7[4][4]=(m1[0]-m1[1]-m1[2]+m1[3])/2;

  //     Quant of chroma AC-coeffs.
  coeff_cost=0;
  cr_cbp_tmp=0;

  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      b4      = 2*(n2/4) + (n1/4);
      ACLevel = img->cofAC[uv+4][b4][0];
      ACRun   = img->cofAC[uv+4][b4][1];

      run      = -1;
      scan_pos =  0;

      for (coeff_ctr=1; coeff_ctr < 16; coeff_ctr++)// start change rd_quant
      {

        if (img->field_picture || ( mb_adaptive && img->field_mode )) 
        {  // Alternate scan for field coding
          i=FIELD_SCAN[coeff_ctr][0];
          j=FIELD_SCAN[coeff_ctr][1];
        }
        else 
        {
          i=SNGL_SCAN[coeff_ctr][0];
          j=SNGL_SCAN[coeff_ctr][1];
        }
        ++run;
        ilev=0;

    // quantization on prediction
    c_err1 = (abs(predicted_chroma_block[n1+i][n2+j]) * quant_coef[qp_rem_sp][i][j] + qp_const2) >> q_bits_sp;
    c_err1 = (c_err1 << q_bits_sp) / quant_coef[qp_rem_sp][i][j];
    c_err1 = img->m7[n1+i][n2+j] - sign(c_err1, predicted_chroma_block[n1+i][n2+j]);
    level1 = (abs(c_err1) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;

    // no quantization on prediction
    c_err2 = img->m7[n1+i][n2+j] - predicted_chroma_block[n1+i][n2+j];
    level2 = (abs(c_err2) * quant_coef[qp_rem][i][j] + qp_const) >> q_bits;

    if (level1 != level2 && level1 != 0 && level2 != 0)
    {
      D_dis1 = img->m7[n1+i][n2+j] - ((sign(level1,c_err1)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6) - predicted_chroma_block[n1+i][n2+j]; 

      levrun_linfo_inter(level1, run, &len, &info);
      D_dis1 = D_dis1 * D_dis1 + lambda_mode * len;

      D_dis2 = img->m7[n1+i][n2+j] - ((sign(level2,c_err2)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6) - predicted_chroma_block[n1+i][n2+j]; 
      levrun_linfo_inter(level2, run, &len, &info);
      D_dis2 = D_dis2 * D_dis2 + lambda_mode * len;
      
      if (D_dis1 == D_dis2)
        level = (abs(level1) < abs(level2)) ? level1 : level2;
      else
      {
        if (D_dis1 < D_dis2)
          level = level1;
        else
          level = level2;
      }
      c_err = (level == level1) ? c_err1 : c_err2;
    }
    else if (level1 == level2)
    {
      level = level1;
      c_err = c_err1;
    }
    else
    {
      level = (level1 == 0) ? level1 : level2;
      c_err = (level1 == 0) ? c_err1 : c_err2;
    }

        if (level  != 0)
        {
          currMB->cbp_blk |=  1 << (16 + (uv << 2) + ((n2 >> 1) + (n1 >> 2))) ;
          if (level > 1)
            coeff_cost += MAX_VALUE;                // set high cost, shall not be discarded
          else
            coeff_cost += COEFF_COST[run];

          cr_cbp_tmp=2;
          ACLevel[scan_pos] = sign(level,c_err);
          ACRun  [scan_pos] = run;
          ++scan_pos;
          run=-1;
          ilev=((sign(level,c_err)*dequant_coef[qp_rem][i][j]*A[i][j]<< qp_per) >>6);
        }
        ilev+=predicted_chroma_block[n1+i][n2+j];
        img->m7[n1+i][n2+j] = sign((abs(ilev) * quant_coef[qp_rem_sp][i][j] + qp_const2) >> q_bits_sp,ilev) * dequant_coef[qp_rem_sp][i][j] << qp_per_sp;
      }
      ACLevel[scan_pos] = 0;
    }
  }

  // * reset chroma coeffs

  if(cr_cbp_tmp==2)
      cr_cbp=2;
  //     IDCT.

      //     Horizontal.
  for (n2=0; n2 <= BLOCK_SIZE; n2 += BLOCK_SIZE)
  {
    for (n1=0; n1 <= BLOCK_SIZE; n1 += BLOCK_SIZE)
    {
      for (j=0; j < BLOCK_SIZE; j++)
      {
        for (i=0; i < BLOCK_SIZE; i++)
        {
          m5[i]=img->m7[n1+i][n2+j];
        }
        m6[0]=(m5[0]+m5[2]);
        m6[1]=(m5[0]-m5[2]);
        m6[2]=(m5[1]>>1)-m5[3];
        m6[3]=m5[1]+(m5[3]>>1);

        for (i=0; i < 2; i++)
        {
          i1=3-i;
          img->m7[n1+i][n2+j]=m6[i]+m6[i1];
          img->m7[n1+i1][n2+j]=m6[i]-m6[i1];
        }
      }

      //     Vertical.
      for (i=0; i < BLOCK_SIZE; i++)
      {
        for (j=0; j < BLOCK_SIZE; j++)
        {
          m5[j]=img->m7[n1+i][n2+j];
        }
        m6[0]=(m5[0]+m5[2]);
        m6[1]=(m5[0]-m5[2]);
        m6[2]=(m5[1]>>1)-m5[3];
        m6[3]=m5[1]+(m5[3]>>1);

        for (j=0; j < 2; j++)
        {
          j2=3-j;
          img->m7[n1+i][n2+j] =min(255,max(0,(m6[j]+m6[j2]+DQ_ROUND)>>DQ_BITS));
          img->m7[n1+i][n2+j2]=min(255,max(0,(m6[j]-m6[j2]+DQ_ROUND)>>DQ_BITS));
        }
      }
    }
  }

  //  Decoded block moved to memory
  for (j=0; j < BLOCK_SIZE*2; j++)
    for (i=0; i < BLOCK_SIZE*2; i++)
    {
      enc_picture->imgUV[uv][img->pix_c_y+j][img->pix_c_x+i]= img->m7[i][j];
    }

  return cr_cbp;
}

/*!
 ************************************************************************
 * \brief
 *    The routine performs transform,quantization,inverse transform, adds the diff.
 *    to the prediction and writes the result to the decoded luma frame. Includes the
 *    RD constrained quantization also.
 *
 * \par Input:
 *    block_x,block_y: Block position inside a macro block (0,4,8,12).
 *
 * \par Output:
 *    nonzero: 0 if no levels are nonzero.  1 if there are nonzero levels.            \n
 *    coeff_cost: Counter for nonzero coefficients, used to discard expencive levels.
 ************************************************************************
 */
void copyblock_sp(int block_x,int block_y)
{
  int sign(int a,int b);

  int i,j,i1,j1,m5[4],m6[4];

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  int predicted_block[BLOCK_SIZE][BLOCK_SIZE];
  int qp_per = (currMB->qpsp-MIN_QP)/6;
  int qp_rem = (currMB->qpsp-MIN_QP)%6;
  int q_bits    = Q_BITS+qp_per;
  int qp_const2 = (1<<q_bits)/2;  //sp_pred

  //  Horizontal transform
  for (j=0; j< BLOCK_SIZE; j++)
    for (i=0; i< BLOCK_SIZE; i++)
    {
      predicted_block[i][j]=img->mpr[i+block_x][j+block_y];
    }

  for (j=0; j < BLOCK_SIZE; j++)
  {
    for (i=0; i < 2; i++)
    {
      i1=3-i;
      m5[i]=predicted_block[i][j]+predicted_block[i1][j];
      m5[i1]=predicted_block[i][j]-predicted_block[i1][j];
    }
    predicted_block[0][j]=(m5[0]+m5[1]);
    predicted_block[2][j]=(m5[0]-m5[1]);
    predicted_block[1][j]=m5[3]*2+m5[2];
    predicted_block[3][j]=m5[3]-m5[2]*2;
  }

  //  Vertival transform

  for (i=0; i < BLOCK_SIZE; i++)
  {
    for (j=0; j < 2; j++)
    {
      j1=3-j;
      m5[j]=predicted_block[i][j]+predicted_block[i][j1];
      m5[j1]=predicted_block[i][j]-predicted_block[i][j1];
    }
    predicted_block[i][0]=(m5[0]+m5[1]);
    predicted_block[i][2]=(m5[0]-m5[1]);
    predicted_block[i][1]=m5[3]*2+m5[2];
    predicted_block[i][3]=m5[3]-m5[2]*2;
  }

  // Quant
  for (j=0;j < BLOCK_SIZE; j++)
    for (i=0; i < BLOCK_SIZE; i++)
       img->m7[i][j]=sign((abs(predicted_block[i][j])* quant_coef[qp_rem][i][j]+qp_const2)>> q_bits,predicted_block[i][j])*dequant_coef[qp_rem][i][j]<<qp_per;

  //     IDCT.
  //     horizontal

  for (j=0;j<BLOCK_SIZE;j++)
  {
    for (i=0;i<BLOCK_SIZE;i++)
    {
      m5[i]=img->m7[i][j];
    }
    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (i=0;i<2;i++)
    {
      i1=3-i;
      img->m7[i][j]=m6[i]+m6[i1];
      img->m7[i1][j]=m6[i]-m6[i1];
    }
  }
  // vertical
  for (i=0;i<BLOCK_SIZE;i++)
  {
    for (j=0;j<BLOCK_SIZE;j++)
      m5[j]=img->m7[i][j];

    m6[0]=(m5[0]+m5[2]);
    m6[1]=(m5[0]-m5[2]);
    m6[2]=(m5[1]>>1)-m5[3];
    m6[3]=m5[1]+(m5[3]>>1);

    for (j=0;j<2;j++)
    {
      j1=3-j;
      img->m7[i][j] =min(255,max(0,(m6[j]+m6[j1]+DQ_ROUND)>>DQ_BITS));
      img->m7[i][j1]=min(255,max(0,(m6[j]-m6[j1]+DQ_ROUND)>>DQ_BITS));
    }
  }

  //  Decoded block moved to frame memory

  for (j=0; j < BLOCK_SIZE; j++)
    for (i=0; i < BLOCK_SIZE; i++)
      enc_picture->imgY[img->pix_y+block_y+j][img->pix_x+block_x+i]=img->m7[i][j];
}
