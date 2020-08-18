
/*!
 *************************************************************************************
 * \file loopFilter.c
 *
 * \brief
 *    Filter to reduce blocking artifacts(手工制品) on a macroblock level.
 *    The filter strengh is QP dependent.
 *
 * \author
 *    Contributors:
 *    - Peter List       Peter.List@t-systems.de:  Original code                                 (13-Aug-2001)
 *    - Jani Lainema     Jani.Lainema@nokia.com:   Some bug fixing, removal of recusiveness      (16-Aug-2001)
 *    - Peter List       Peter.List@t-systems.de:  inplace filtering and various simplifications (10-Jan-2002)
 *    - Anthony Joch     anthony@ubvideo.com:      Simplified switching between filters and 
 *                                                 non-recursive default filter.                 (08-Jul-2002)
 *    - Cristina Gomila  cristina.gomila@thomson.net: Simplification of the chroma deblocking
 *                                                    from JVT-E089                              (21-Nov-2002)
 *************************************************************************************
 */
#include <stdlib.h>
#include <string.h>
#include "global.h"
#include "image.h"
#include "mb_access.h"

extern const byte QP_SCALE_CR[52] ;//色度QP数组

byte mixedModeEdgeFlag, fieldModeFilteringFlag;

/*********************************************************************************************************/

#define  IClip( Min, Max, Val) (((Val)<(Min))? (Min):(((Val)>(Max))? (Max):(Val)))

// NOTE: to change the tables below for instance when the QP doubling is changed from 6 to 8 values 
//       send an e-mail to Peter.List@t-systems.com to get a little programm that calculates them automatically 

byte ALPHA_TABLE[52]  = {0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,4,4,5,6,  7,8,9,10,12,13,15,17,  20,22,25,28,32,36,40,45,  50,56,63,71,80,90,101,113,  127,144,162,182,203,226,255,255} ;
byte  BETA_TABLE[52]  = {0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,2,2,2,3,  3,3,3, 4, 4, 4, 6, 6,   7, 7, 8, 8, 9, 9,10,10,  11,11,12,12,13,13, 14, 14,   15, 15, 16, 16, 17, 17, 18, 18} ;
//BS=1 CLIP_TAB[52][1]    BS=2 CLIP_TAB[52][2]   BS=3 CLIP_TAB[52][3]
byte CLIP_TAB[52][5]  =
{
  { 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 0},{ 0, 0, 0, 1, 1},{ 0, 0, 0, 1, 1},{ 0, 0, 0, 1, 1},{ 0, 0, 0, 1, 1},{ 0, 0, 1, 1, 1},{ 0, 0, 1, 1, 1},{ 0, 1, 1, 1, 1},
  { 0, 1, 1, 1, 1},{ 0, 1, 1, 1, 1},{ 0, 1, 1, 1, 1},{ 0, 1, 1, 2, 2},{ 0, 1, 1, 2, 2},{ 0, 1, 1, 2, 2},{ 0, 1, 1, 2, 2},{ 0, 1, 2, 3, 3},
  { 0, 1, 2, 3, 3},{ 0, 2, 2, 3, 3},{ 0, 2, 2, 4, 4},{ 0, 2, 3, 4, 4},{ 0, 2, 3, 4, 4},{ 0, 3, 3, 5, 5},{ 0, 3, 4, 6, 6},{ 0, 3, 4, 6, 6},
  { 0, 4, 5, 7, 7},{ 0, 4, 5, 8, 8},{ 0, 4, 6, 9, 9},{ 0, 5, 7,10,10},{ 0, 6, 8,11,11},{ 0, 6, 8,13,13},{ 0, 7,10,14,14},{ 0, 8,11,16,16},
  { 0, 9,12,18,18},{ 0,10,13,20,20},{ 0,11,15,23,23},{ 0,13,17,25,25}
} ;

void GetStrength(byte Strength[16],ImageParameters *img,int MbQAddr,int dir,int edge, int mvlimit);
void EdgeLoop(byte** Img, byte Strength[16],ImageParameters *img, int MbQAddr, int AlphaC0Offset, int BetaOffset, int dir, int edge, int width, int yuv);
void DeblockMb(ImageParameters *img, byte **imgY, byte ***imgUV, int MbQAddr) ;

/*!
 *****************************************************************************************
 * \brief
 *    Filter all macroblocks in order of increasing macroblock address.
 *****************************************************************************************
 */
void DeblockFrame(ImageParameters *img, byte **imgY, byte ***imgUV)
{
  unsigned i;

  for (i=0; i<img->PicSizeInMbs; i++)//对一帧图像滤波是对图像里面每一个宏块进行滤波
  {
    DeblockMb( img, imgY, imgUV, i ) ;
  }
} 


/*!
 *****************************************************************************************
 * \brief
 *    Deblocking filter for one macroblock.
 *****************************************************************************************
 */

void DeblockMb(ImageParameters *img, byte **imgY, byte ***imgUV, int MbQAddr)
{
  int           EdgeCondition;
  int           dir,edge;
  byte          Strength[16];
  int           mb_x, mb_y;

  int           filterLeftMbEdgeFlag;
  int           filterTopMbEdgeFlag;
  int           fieldModeMbFlag;
  int           mvlimit=4;
  int           i, StrengthSum;
  Macroblock    *MbQ;
  
  img->DeblockCall = 1;// 0非滤波状态 1正常滤波状态 2 extra horizontal edge状态
  get_mb_pos (MbQAddr, &mb_x, &mb_y);//得到该块左上角的像素坐标
  filterLeftMbEdgeFlag  = (mb_x != 0);
  filterTopMbEdgeFlag   = (mb_y != 0);//图像边界判断

  MbQ  = &(img->mb_data[MbQAddr]) ; // current Mb Q块为当前块

  if (img->MbaffFrameFlag && mb_y==16 && MbQ->mb_field)//??
    filterTopMbEdgeFlag = 0;

  fieldModeMbFlag       = (img->structure!=FRAME) || (img->MbaffFrameFlag && MbQ->mb_field);//场模式
  if (fieldModeMbFlag)
    mvlimit = 2;

  // return, if filter is disabled
  //LFDisableIdc=0 时slice边界也要滤波
  //LFDisableIdc=1 时不滤波
  //LFDisableIdc=2 时slice边界不用滤波
  if (MbQ->LFDisableIdc==1) {
    img->DeblockCall = 0;
    return;
  }

  if (MbQ->LFDisableIdc==2)
  {
    // don't filter at slice boundaries
    filterLeftMbEdgeFlag = MbQ->mbAvailA;//已经检查过Q 和 A是否同一个slice
    filterTopMbEdgeFlag  = MbQ->mbAvailB;
  }

  img->current_mb_nr = MbQAddr;
  CheckAvailabilityOfNeighbors();

  for( dir=0 ; dir<2 ; dir++ ) //方向 0:V  1:H                               // vertical edges, than horicontal edges
  {
    //水平方向滤波的时候需要上边界有效 垂直方向滤波的时候需要左边界有效 
    EdgeCondition = (dir && filterTopMbEdgeFlag) || (!dir && filterLeftMbEdgeFlag); // can not filter beyond picture boundaries
    for( edge=0 ; edge<4 ; edge++ ) //亮度宏块垂直和水平方向分别有4条边需要滤波                                            // first 4 vertical strips of 16 pel
    {                                                                                         // then  4 horicontal
      if( edge || EdgeCondition )//?? 为什么判断里面要加edge 
      {

        GetStrength(Strength,img,MbQAddr,dir,edge, mvlimit); // Strength for 4 blks in 1 stripe
        StrengthSum = Strength[0];
        for (i = 1; i < 16; i++) StrengthSum += Strength[i];//StrengthSum 边缘强度和
        if( StrengthSum )                      // only if one of the 16 Strength bytes is != 0
        {
          EdgeLoop( imgY, Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, edge, img->width, 0) ; 
          if( (imgUV != NULL) && !(edge & 1) )//edge:0 1 2 3 !(edge & 1):1 0 1 0  色度宏块垂直和水平方向分别有2条边需要滤波
          {
            EdgeLoop( imgUV[0], Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, edge/2, img->width_cr, 1 ) ; 
            EdgeLoop( imgUV[1], Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, edge/2, img->width_cr, 1 ) ; 
          }
        }

		//不是很懂？水平方向最上面一条边，Q为帧模式，P为场模式 p295 c,d
        if (dir && !edge && !MbQ->mb_field && mixedModeEdgeFlag) {
          // this is the extra horizontal edge between a frame macroblock pair and a field above it
          img->DeblockCall = 2;
          GetStrength(Strength,img,MbQAddr,dir,4, mvlimit); // Strength for 4 blks in 1 stripe
          if( *((int*)Strength) )                      // only if one of the 4 Strength bytes is != 0
          {
            EdgeLoop( imgY, Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, 4, img->width, 0) ; 
            if( (imgUV != NULL) && !(edge & 1) )
            {
              EdgeLoop( imgUV[0], Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, 4, img->width_cr, 1 ) ; 
              EdgeLoop( imgUV[1], Strength, img, MbQAddr, MbQ->LFAlphaC0Offset, MbQ->LFBetaOffset, dir, 4, img->width_cr, 1 ) ; 
            }
          }
          img->DeblockCall = 1;
        }

      }
    }//end edge
  }//end loop dir
  img->DeblockCall = 0;

}

  /*!
 *********************************************************************************************
 * \brief
 *    returns a buffer of 16 Strength values for one stripe in a mb (for different Frame types)
 *********************************************************************************************
 */

int  ININT_STRENGTH[4] = {0x04040404, 0x03030303, 0x03030303, 0x03030303} ; 
byte BLK_NUM[2][4][4]  = {{{0,4,8,12},{1,5,9,13},{2,6,10,14},{3,7,11,15}},{{0,1,2,3},{4,5,6,7},{8,9,10,11},{12,13,14,15}}} ;
byte BLK_4_TO_8[16]    = {0,0,1,1,0,0,1,1,2,2,3,3,2,2,3,3} ;
#define ANY_INTRA (MbP->mb_type==I4MB||MbP->mb_type==I16MB||MbP->mb_type==IPCM||MbQ->mb_type==I4MB||MbQ->mb_type==I16MB||MbQ->mb_type==IPCM)

//每一条边上有16个像素 就有16个边界强度描述
void GetStrength(byte Strength[16],ImageParameters *img,int MbQAddr,int dir,int edge, int mvlimit)
{
  int    blkP, blkQ, idx;
  int    blk_x, blk_x2, blk_y, blk_y2 ;
  int    ***list0_mv = enc_picture->mv[LIST_0];//已经编码图像 已经得到了当前宏块前向参考的运动向量mv
  int    ***list1_mv = enc_picture->mv[LIST_1];//后向参考mv
  int    **list0_refIdxArr = enc_picture->ref_idx[LIST_0];//参考帧号
  int    **list1_refIdxArr = enc_picture->ref_idx[LIST_1];
  int64    **list0_refPicIdArr = enc_picture->ref_pic_id[LIST_0];//参考图像的ID ?
  int64    **list1_refPicIdArr = enc_picture->ref_pic_id[LIST_1];
  int    xQ, xP, yQ, yP;
  int    mb_x, mb_y;
  Macroblock    *MbQ;
  Macroblock    *MbP;
  PixelPos pixP;

  MbQ = &(img->mb_data[MbQAddr]);//得到当前宏块数据

  for( idx=0 ; idx<16 ; idx++ )//dir 0:V 1:H  16*16 每一条边有16个边界强度，垂直加水平有8条边
  {
  /*
	dir=0 idx=0 edge=0 xQ=0 yQ=0 xQ-(1-dir)=-1 yQ-dir=0
	dir=0 idx=1 edge=0 xQ=0 yQ=1 xQ-(1-dir)=-1 yQ-dir=1
	dir=0 idx=2 edge=0 xQ=0 yQ=2 xQ-(1-dir)=-1 yQ-dir=2
	dir=0 idx=3 edge=0 xQ=0 yQ=3 xQ-(1-dir)=-1 yQ-dir=3
	dir=0 idx=4 edge=0 xQ=0 yQ=4 xQ-(1-dir)=-1 yQ-dir=4
	dir=0 idx=5 edge=0 xQ=0 yQ=5 xQ-(1-dir)=-1 yQ-dir=5
	...
	dir=0 idx=0 edge=1 xQ=4 yQ=0 xQ-(1-dir)=3  yQ-dir=0
	dir=0 idx=1 edge=1 xQ=4 yQ=1 xQ-(1-dir)=3  yQ-dir=1
	dir=0 idx=2 edge=1 xQ=4 yQ=2 xQ-(1-dir)=3  yQ-dir=2
	dir=0 idx=3 edge=1 xQ=4 yQ=3 xQ-(1-dir)=3  yQ-dir=3
	dir=0 idx=4 edge=1 xQ=4 yQ=4 xQ-(1-dir)=3  yQ-dir=4
	dir=0 idx=5 edge=1 xQ=4 yQ=5 xQ-(1-dir)=3  yQ-dir=5	

	p3 p2 p1 p0 q0 q1 q2 q3
  */
  	//q0和p0可能是同一宏块
    xQ = dir ? idx : edge << 2;//当前宏块左上角为原点 (xQ,yQ)是待滤波、当前块边界上的像素坐标 相当于q0
    yQ = dir ? (edge < 4 ? edge << 2 : 1) : idx;
    getNeighbour(MbQAddr, xQ - (1 - dir), yQ - dir, 1, &pixP);//(xQ - (1 - dir), yQ - dir)是偏移，pixP是待滤波、当前块的相邻块边界上的像素 
    xP = pixP.x;//图像左上角为原点 像素点坐标 (xP,yP)相当于p0
    yP = pixP.y;
    MbP = &(img->mb_data[pixP.mb_addr]);//当边界是宏块内边界时，P宏块和Q宏块为同一个宏块
    mixedModeEdgeFlag = MbQ->mb_field != MbP->mb_field;//p 和 Q一个是帧 一个是场就是混合模式
	/*
		blkQ 
				0 0 0 0 1
				1 1 1 1 2 
				2 2 2 2 3
				3 3 3 3 4
	*/
    blkQ = ((yQ>>2)<<2) + (xQ>>2);//当前宏块左上角为原点  转换成为4*4块坐标
    blkP = ((yP>>2)<<2) + (xP>>2);//图像左上角为原点 转换成为4*4块坐标

    if ((img->type==SP_SLICE)||(img->type==SI_SLICE) )
    {
    	//
      Strength[idx] = (edge == 0 && (((!img->MbaffFrameFlag && (img->structure==FRAME)) ||
      (img->MbaffFrameFlag && !MbP->mb_field && !MbQ->mb_field)) ||
      ((img->MbaffFrameFlag || (img->structure!=FRAME)) && !dir))) ? 4 : 3;
    }
    else
    {
      // Start with Strength=3. or Strength=4 for Mb-edge
      Strength[idx] = (edge == 0 && (((!img->MbaffFrameFlag && (img->structure==FRAME)) ||
        (img->MbaffFrameFlag && !MbP->mb_field && !MbQ->mb_field)) ||
        ((img->MbaffFrameFlag || (img->structure!=FRAME)) && !dir))) ? 4 : 3;

      if(  !(MbP->mb_type==I4MB || MbP->mb_type==I16MB || MbP->mb_type==IPCM)
        && !(MbQ->mb_type==I4MB || MbQ->mb_type==I16MB || MbQ->mb_type==IPCM) )
      {
        if( ((MbQ->cbp_blk &  (1 << blkQ )) != 0) || ((MbP->cbp_blk &  (1 << blkP)) != 0) )
          Strength[idx] = 2 ;//残差系数不为0
        else
        {                                                     // if no coefs, but vector difference >= 1 set Strength=1 
          // if this is a mixed mode edge then one set of reference pictures will be frame and the
          // other will be field
          if (mixedModeEdgeFlag)
          {
            (Strength[idx] = 1);
          }
          else
          {
            get_mb_block_pos (MbQAddr, &mb_x, &mb_y);
			//????
            blk_y  = (mb_y<<2) + (blkQ >> 2) ;//像素坐标
            blk_x  = (mb_x<<2) + (blkQ  & 3) ;
            blk_y2 = pixP.pos_y >> 2;//块坐标
            blk_x2 = pixP.pos_x >> 2;
            if( (img->type == B_SLICE) )
            {
              int64 ref_p0,ref_p1,ref_q0,ref_q1;      
              ref_p0 = list0_refIdxArr[blk_x][blk_y]<0 ? -1 : list0_refPicIdArr[blk_x][blk_y];
              ref_q0 = list0_refIdxArr[blk_x2][blk_y2]<0 ? -1 : list0_refPicIdArr[blk_x2][blk_y2];
              ref_p1 = list1_refIdxArr[blk_x][blk_y]<0 ? -1 : list1_refPicIdArr[blk_x][blk_y];
              ref_q1 = list1_refIdxArr[blk_x2][blk_y2]<0 ? -1 : list1_refPicIdArr[blk_x2][blk_y2];
              if ( ((ref_p0==ref_q0) && (ref_p1==ref_q1)) ||
                ((ref_p0==ref_q1) && (ref_p1==ref_q0))) //前向或者后向对应像素的参考帧相同
              {
                Strength[idx]=0;//参考帧相同
                // L0 and L1 reference pictures of p0 are different; q0 as well
                if (ref_p0 != ref_p1) 
                { 
                  // compare MV for the same reference picture
                  if (ref_p0==ref_q0) 
                  {
                   //运动向量
                    Strength[idx] =  (abs( list0_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4) |
                      (abs( list0_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit) |
                      (abs( list1_mv[blk_x][blk_y][0] - list1_mv[blk_x2][blk_y2][0]) >= 4) |
                      (abs( list1_mv[blk_x][blk_y][1] - list1_mv[blk_x2][blk_y2][1]) >= mvlimit);
                  }
                  else 
                  {
                    Strength[idx] =  (abs( list0_mv[blk_x][blk_y][0] - list1_mv[blk_x2][blk_y2][0]) >= 4) |
                      (abs( list0_mv[blk_x][blk_y][1] - list1_mv[blk_x2][blk_y2][1]) >= mvlimit) |
                      (abs( list1_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4) |
                      (abs( list1_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit);
                  } 
                }
                else 
                { // L0 and L1 reference pictures of p0 are the same; q0 as well

                  Strength[idx] =  ((abs( list0_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4) |
                    (abs( list0_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit ) |
                    (abs( list1_mv[blk_x][blk_y][0] - list1_mv[blk_x2][blk_y2][0]) >= 4) |
                    (abs( list1_mv[blk_x][blk_y][1] - list1_mv[blk_x2][blk_y2][1]) >= mvlimit))
                    &&
                    ((abs( list0_mv[blk_x][blk_y][0] - list1_mv[blk_x2][blk_y2][0]) >= 4) |
                    (abs( list0_mv[blk_x][blk_y][1] - list1_mv[blk_x2][blk_y2][1]) >= mvlimit) |
                    (abs( list1_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4) |
                    (abs( list1_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit));
                }       
              }
              else 
              {
                Strength[idx] = 1;        
              } 
            }
            else  
            { // P slice
              int64 ref_p0,ref_q0;      
              ref_p0 = list0_refIdxArr[blk_x][blk_y]<0 ? -1 : list0_refPicIdArr[blk_x][blk_y];
              ref_q0 = list0_refIdxArr[blk_x2][blk_y2]<0 ? -1 : list0_refPicIdArr[blk_x2][blk_y2];
              Strength[idx] =  (ref_p0 != ref_q0 ) |
                (abs( list0_mv[blk_x][blk_y][0] - list0_mv[blk_x2][blk_y2][0]) >= 4 ) |
                (abs( list0_mv[blk_x][blk_y][1] - list0_mv[blk_x2][blk_y2][1]) >= mvlimit );
            }
          }
        }
      }
    }
  }
}

#define CQPOF(qp) (Clip3(0, 51, qp + active_pps->chroma_qp_index_offset))

/*!
 *****************************************************************************************
 * \brief
 *    Filters one edge of 16 (luma) or 8 (chroma) pel
 *****************************************************************************************
 */
void EdgeLoop(byte** Img, byte Strength[16],ImageParameters *img, int MbQAddr, int AlphaC0Offset, int BetaOffset,
              int dir, int edge, int width, int yuv)
{
  int      pel, ap = 0, aq = 0, Strng ;
  int      incP, incQ;
  int      C0, c0, Delta, dif, AbsDelta ;
  int      L2 = 0, L1, L0, R0, R1, R2 = 0, RL0, L3, R3 ;
  int      Alpha = 0, Beta = 0 ;
  byte*    ClipTab = NULL;   
  int      small_gap;
  int      indexA, indexB;
  int      PelNum;
  int      StrengthIdx;
  byte     *SrcPtrP, *SrcPtrQ;
  int      QP;
  int      xP, xQ, yP, yQ;
  Macroblock *MbQ, *MbP;
  PixelPos pixP, pixQ;
  
  PelNum = yuv ? 8 : 16 ;

  for( pel=0 ; pel<PelNum ; pel++ )
  {
    xQ = dir ? pel : edge << 2;
    yQ = dir ? (edge < 4 ? edge << 2 : 1) : pel;
	//(xQ, yQ)和(xQ - (1 - dir), yQ - dir)是以宏块左上角为原点 的偏移坐标
    getNeighbour(MbQAddr, xQ, yQ, 1-yuv, &pixQ);//pixQ是以图像左上角为原点 的像素坐标
    getNeighbour(MbQAddr, xQ - (1 - dir), yQ - dir, 1-yuv, &pixP);//pixP是以图像左上角为原点 的像素坐标
    xP = pixP.x;
    yP = pixP.y;
    MbQ = &(img->mb_data[MbQAddr]);
    MbP = &(img->mb_data[pixP.mb_addr]);
    fieldModeFilteringFlag = MbQ->mb_field || MbP->mb_field;
	//?? yuv=1色度    yuv=0亮度
    StrengthIdx = yuv ? ((MbQ->mb_field && !MbP->mb_field) ? pel<<1 : ((pel>>1)<<2)+(pel%2)) : pel ;
	//MbQ->LFDisableIdc== 0表示slice也要滤波
    if (pixP.available || (MbQ->LFDisableIdc== 0)) {
		//??水平方向      dir方向 0:V   1:H 
		/*
			垂直方向滤波(水平边界) dir=0 
										帧场自适应且Q块为帧模式            incQ = 2*width
									  非帧场自适应且Q块为场模式             incQ = width
			水平方向滤波(垂直边界) dir=1
									incQ = 1
				
		*/
      incQ = dir ? ((fieldModeFilteringFlag && !MbQ->mb_field) ? 2 * width : width) : 1;
      incP = dir ? ((fieldModeFilteringFlag && !MbP->mb_field) ? 2 * width : width) : 1;
      SrcPtrQ = &(Img[pixQ.pos_y][pixQ.pos_x]);//pixQ对应像素值的内存地址
      SrcPtrP = &(Img[pixP.pos_y][pixP.pos_x]);

      // Average QP of the two blocks
      QP  = yuv ? (QP_SCALE_CR[CQPOF(MbP->qp)] + QP_SCALE_CR[CQPOF(MbQ->qp)] + 1) >> 1 : (MbP->qp + MbQ->qp + 1) >> 1;

      indexA = IClip(0, MAX_QP, QP + AlphaC0Offset);
      indexB = IClip(0, MAX_QP, QP + BetaOffset);
    
      Alpha=ALPHA_TABLE[indexA];//indexA是边界两边是否平坦的刻画
      Beta=BETA_TABLE[indexB];  //indexB是边界相邻像素差距大小的描述
      ClipTab=CLIP_TAB[indexA];//对应indexA查表得BS不同时候的偏移 其实只用到了BS=1,2,3
	/*
		水平边界(垂直方向)
			L3 L2 L1 L0 R0 R1 R2 R3
		垂直边界(水平方向)	
					L3
					L2
					L1
					L0
					R0
					R1
					R2
					R3
				
	  */
      L0  = SrcPtrP[0] ;
      R0  = SrcPtrQ[0] ;
      L1  = SrcPtrP[-incP] ;
      R1  = SrcPtrQ[ incQ] ;
      L2  = SrcPtrP[-incP*2] ;
      R2  = SrcPtrQ[ incQ*2] ;
      L3  = SrcPtrP[-incP*3] ;
      R3  = SrcPtrQ[ incQ*3] ;
      if( (Strng = Strength[StrengthIdx]) )//Strng=0,1,2,3,4
      {
        AbsDelta  = abs( Delta = R0 - L0 )  ;
      
        if( AbsDelta < Alpha )
        {
          C0  = ClipTab[ Strng ] ;//后面限幅用 通过具体的BS值查表得限幅值
          if( ((abs( R0 - R1) - Beta )  & (abs(L0 - L1) - Beta )) < 0  ) 
          {
            if( !yuv)
            {
              aq  = (abs( R0 - R2) - Beta ) < 0  ;//R1处需要修改
              ap  = (abs( L0 - L2) - Beta ) < 0  ;//L1处需要修改
            }
          
            RL0             = L0 + R0 ;
          
            if(Strng == 4 )    // INTRA strong filtering
            {
              if( yuv)  // Chroma
              {
                SrcPtrQ[0] = ((R1 << 1) + R0 + L1 + 2) >> 2; 
                SrcPtrP[0] = ((L1 << 1) + L0 + R1 + 2) >> 2;                                           
              }
              else  // Luma
              {
                small_gap = (AbsDelta < ((Alpha >> 2) + 2));
              
                aq &= small_gap;
                ap &= small_gap;
                //强滤波或者弱滤波
                SrcPtrQ[0]   = aq ? ( L1 + ((R1 + RL0) << 1) +  R2 + 4) >> 3 : ((R1 << 1) + R0 + L1 + 2) >> 2 ;
                SrcPtrP[0]   = ap ? ( R1 + ((L1 + RL0) << 1) +  L2 + 4) >> 3 : ((L1 << 1) + L0 + R1 + 2) >> 2 ;
              
                SrcPtrQ[ incQ] =   aq  ? ( R2 + R0 + R1 + L0 + 2) >> 2 : R1;
                SrcPtrP[-incP] =   ap  ? ( L2 + L1 + L0 + R0 + 2) >> 2 : L1;
              
                SrcPtrQ[ incQ*2] = aq ? (((R3 + R2) <<1) + R2 + R1 + RL0 + 4) >> 3 : R2;
                SrcPtrP[-incP*2] = ap ? (((L3 + L2) <<1) + L2 + L1 + RL0 + 4) >> 3 : L2;
              }
            }
            else                                                                                   // normal filtering
            {
              c0               = yuv? (C0+1):(C0 + ap + aq) ;//c0才是最终的限幅值
              dif              = IClip( -c0, c0, ( (Delta << 2) + (L1 - R1) + 4) >> 3 ) ;//限幅
              SrcPtrP[0]  = IClip(0, 255, L0 + dif) ;
              SrcPtrQ[0]  = IClip(0, 255, R0 - dif) ;
            
              if( !yuv )
              {
                if( ap )
                  SrcPtrP[-incP] += IClip( -C0,  C0, ( L2 + ((RL0 + 1) >> 1) - (L1<<1)) >> 1 ) ;
                if( aq  )
                  SrcPtrQ[ incQ] += IClip( -C0,  C0, ( R2 + ((RL0 + 1) >> 1) - (R1<<1)) >> 1 ) ;
              } ;
            } ;
          } ; 
        } ;
      } ;
    } ;
  }
}

