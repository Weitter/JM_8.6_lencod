
/*!
 *************************************************************************************
 * \file mb_access.c
 *
 * \brief
 *    Functions for macroblock neighborhoods
 *
 *  \author
 *      Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Karsten S黨ring          <suehring@hhi.de>
 *************************************************************************************
 */

#include "global.h"
#include "mbuffer.h"

extern StorablePicture *dec_picture;

/*!
 ************************************************************************
 * \brief
 *    returns 1 if the macroblock at the given address is available
 ************************************************************************
 */
 //mbAddr是待检测宏块地址          currMbAddr是当前宏块地址
int mb_is_available(int mbAddr, int currMbAddr)
{
  if ((mbAddr < 0) || (mbAddr > ((int)dec_picture->PicSizeInMbs - 1)))//地址在有效范围内
    return 0;

  // the following line checks both: slice number and if the mb has been decoded
  if (!img->DeblockCall)//回调函数在判断该快是否在用于去块滤波
  {
    if (img->mb_data[mbAddr].slice_nr != img->mb_data[currMbAddr].slice_nr)//是否在同一片中
      return 0;
  }
  
  return 1;
}


/*!
 ************************************************************************
 * \brief
 *    Checks the availability of neighboring macroblocks of
 *    the current macroblock for prediction and context determination;
 ************************************************************************
 */
void CheckAvailabilityOfNeighbors()
{
  const int mb_nr = img->current_mb_nr;//当前宏块地址序号
  Macroblock *currMB = &img->mb_data[mb_nr];//得到当前宏块数据

  // mark all neighbors as unavailable
  currMB->mb_available_up   = NULL;//初始化
  currMB->mb_available_left = NULL;
/*
	DBC
	AX
  */
  if (dec_picture->MbaffFrameFlag)
  {
    currMB->mbAddrA = 2 * (mb_nr/2 - 1);
    currMB->mbAddrB = 2 * (mb_nr/2 - dec_picture->PicWidthInMbs);
    currMB->mbAddrC = 2 * (mb_nr/2 - dec_picture->PicWidthInMbs + 1);
    currMB->mbAddrD = 2 * (mb_nr/2 - dec_picture->PicWidthInMbs - 1);
    
    currMB->mbAvailA = mb_is_available(currMB->mbAddrA, mb_nr) && (((mb_nr/2) % dec_picture->PicWidthInMbs)!=0);
    currMB->mbAvailB = mb_is_available(currMB->mbAddrB, mb_nr);
    currMB->mbAvailC = mb_is_available(currMB->mbAddrC, mb_nr) && (((mb_nr/2 +1) % dec_picture->PicWidthInMbs)!=0);
    currMB->mbAvailD = mb_is_available(currMB->mbAddrD, mb_nr) && (((mb_nr/2) % dec_picture->PicWidthInMbs)!=0);
  }
  else
  {
    currMB->mbAddrA = mb_nr - 1;
    currMB->mbAddrB = mb_nr - dec_picture->PicWidthInMbs;
    currMB->mbAddrC = mb_nr - dec_picture->PicWidthInMbs + 1;
    currMB->mbAddrD = mb_nr - dec_picture->PicWidthInMbs - 1;

    currMB->mbAvailA = mb_is_available(currMB->mbAddrA, mb_nr) && ((mb_nr % dec_picture->PicWidthInMbs)!=0);//当前宏块不是最左边的块
    currMB->mbAvailB = mb_is_available(currMB->mbAddrB, mb_nr);
    currMB->mbAvailC = mb_is_available(currMB->mbAddrC, mb_nr) && (((mb_nr+1) % dec_picture->PicWidthInMbs)!=0);//当前宏块不是最右边的块
    currMB->mbAvailD = mb_is_available(currMB->mbAddrD, mb_nr) && ((mb_nr % dec_picture->PicWidthInMbs)!=0);//当前宏块不是最左边的块
  }
}


/*!
 ************************************************************************
 * \brief
 *    returns the x and y macroblock coordinates for a given MbAddress
 ************************************************************************
 */
void get_mb_block_pos (int mb_addr, int *x, int*y)//得到以图像左上角为原点  该宏块左上角的16*16像素块坐标
{

  if (dec_picture->MbaffFrameFlag)
  {
    *x = ((mb_addr/2) % dec_picture->PicWidthInMbs);
    *y = ( ((mb_addr/2) / dec_picture->PicWidthInMbs)  * 2 + (mb_addr%2));
  }
  else
  {
    *x = (mb_addr % dec_picture->PicWidthInMbs);
    *y = (mb_addr / dec_picture->PicWidthInMbs);
  }
}


/*!
 ************************************************************************
 * \brief
 *    returns the x and y sample coordinates for a given MbAddress
 ************************************************************************
 */
void get_mb_pos (int mb_addr, int *x, int*y)//得到以图像左上角为原点  该宏块左上角的像素坐标
{
  get_mb_block_pos(mb_addr, x, y);
  
  (*x) *= MB_BLOCK_SIZE;
  (*y) *= MB_BLOCK_SIZE;
}


/*!
 ************************************************************************
 * \brief
 *    get neighbouring positions for non-aff coding
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
 //当前宏块左上角为原点 xN，yN表示整像素偏移
void getNonAffNeighbour(unsigned int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)
{
  Macroblock *currMb = &img->mb_data[curr_mb_nr];
  int maxWH;

  if (luma)
    maxWH = 16;
  else
    maxWH = 8;
/*
	DBC
	AX
  */
  if ((xN<0)&&(yN<0))
  {
    pix->mb_addr   = currMb->mbAddrD;//当前宏块左上角为原点           往右上方偏移后的像素pix pix的信息有宏块D来描述 
    pix->available = currMb->mbAvailD;
  }
  else
  if ((xN<0)&&((yN>=0)&&(yN<maxWH)))
  {
    pix->mb_addr  = currMb->mbAddrA;
    pix->available = currMb->mbAvailA;
  }
  else
  if (((xN>=0)&&(xN<maxWH))&&(yN<0))
  {
    pix->mb_addr  = currMb->mbAddrB;
    pix->available = currMb->mbAvailB;
  }
  else
  if (((xN>=0)&&(xN<maxWH))&&((yN>=0)&&(yN<maxWH)))
  {
    pix->mb_addr  = curr_mb_nr;
    pix->available = 1;
  }
  else
  if ((xN>=maxWH)&&(yN<0))
  {
    pix->mb_addr  = currMb->mbAddrC;
    pix->available = currMb->mbAvailC;
  }
  else 
  {
    pix->available = 0;
  }
  if (pix->available || img->DeblockCall)
  {
    pix->x = (xN + maxWH) % maxWH;//xN作为偏移值应该限制在宏块大小范围内
    pix->y = (yN + maxWH) % maxWH;
    get_mb_pos(pix->mb_addr, &(pix->pos_x), &(pix->pos_y));//以图像左上角为原点    	     当前宏块左上角的像素坐标
    if (luma)
    {
      pix->pos_x += pix->x;//宏块左上角的像素坐标加上偏移坐标 得到宏块具体相邻像素的像素坐标(图像左上角为原点)
      pix->pos_y += pix->y;
    }
    else
    {
      pix->pos_x = (pix->pos_x/2) + pix->x;//u v 
      pix->pos_y = (pix->pos_y/2) + pix->y;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    get neighbouring positions for aff coding
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getAffNeighbour(unsigned int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)//帧场自适应
{
  Macroblock *currMb = &img->mb_data[curr_mb_nr];
  int maxWH;
  int yM = -1;

  if (luma)
    maxWH = 16;
  else
    maxWH = 8;

  // initialize to "not available"
  pix->available = 0;

  if(yN > (maxWH - 1))
  {
    return;
  }
  if ((xN > (maxWH -1)) && ((yN >= 0)&&(yN < (maxWH ))))
  {
    return;
  }

  if (xN < 0)
  {
    if (yN < 0)
    {
      if(!currMb->mb_field)
      {
        // frame
        if (curr_mb_nr%2 == 0)
        {
          // top
          pix->mb_addr  = currMb->mbAddrD  + 1;
          pix->available = currMb->mbAvailD;
           yM      = yN;
        }
        else
        {
          // bottom
          pix->mb_addr  = currMb->mbAddrA;
          pix->available = currMb->mbAvailA;
          if (currMb->mbAvailA)
          {
            if(!img->mb_data[currMb->mbAddrA].mb_field)
            {
               yM = yN;
            }
            else
            {
              (pix->mb_addr)++;
               yM = (yN + maxWH) >> 1;
            }
          }
        }
      }
      else
      {
        // field
        if(curr_mb_nr % 2 == 0)
        {
          // top
          pix->mb_addr  = currMb->mbAddrD;
          pix->available = currMb->mbAvailD;
          if (currMb->mbAvailD)
          {
            if(!img->mb_data[currMb->mbAddrD].mb_field)
            {
              (pix->mb_addr)++;
               yM = 2 * yN;
            }
            else
            {
               yM = yN;
            }
          }
        }
        else
        {
          // bottom
          pix->mb_addr  = currMb->mbAddrD+1;
          pix->available = currMb->mbAvailD;
           yM      = yN;
        }
      }
    }
    else
    { // xN < 0 && yN >= 0
      if ((yN >= 0) && (yN <maxWH))
      {
        if (!currMb->mb_field)
        {
          // frame
          if(curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrA;
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field)
              {
                 yM = yN;
              }
              else
              {
                if (yN %2 == 0)
                {
                   yM = yN>> 1;
                }
                else
                {
                  (pix->mb_addr)++;
                   yM = yN>> 1;
                }
              }
            }
          }
          else
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrA;
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field)
              {
                (pix->mb_addr)++;
                 yM = yN;
              }
              else
              {
                if (yN %2 == 0)
                {
                   yM = (yN + maxWH) >> 1;
                }
                else
                {
                  (pix->mb_addr)++;
                   yM = (yN + maxWH) >> 1;
                }
              }
            }
          }
        }
        else
        {
          // field
          if (curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrA;
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field)
              {
                if (yN < (maxWH / 2))
                {
                   yM = yN << 1;
                }
                else
                {
                  (pix->mb_addr)++;
                   yM = (yN << 1 ) - maxWH;
                }
              }
              else
              {
                 yM = yN;
              }
            }
          }
          else
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrA;
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field)
              {
                if (yN < (maxWH / 2))
                {
                   yM = (yN << 1) + 1;
                }
                else
                {
                  (pix->mb_addr)++;
                   yM = (yN << 1 ) + 1 - maxWH;
                }
              }
              else
              {
                 (pix->mb_addr)++;
                 yM = yN;
              }
            }
          }
        }
      }
    }
  }
  else
  {
    // xN >= 0
    if ((xN >= 0)&&(xN <maxWH))
    {
      if (yN<0)
      {
        if (!currMb->mb_field)
        {
          //frame
          if (curr_mb_nr % 2 == 0)
          {
            //top
            pix->mb_addr  = currMb->mbAddrB;
            // for the deblocker if the current MB is a frame and the one above is a field
            // then the neighbor is the top MB of the pair
            if (currMb->mbAvailB)
            {
              if (!(img->DeblockCall == 1 && (img->mb_data[currMb->mbAddrB]).mb_field))
                pix->mb_addr  += 1;
            }

            pix->available = currMb->mbAvailB;
             yM      = yN;
          }
          else
          {
            // bottom
            pix->mb_addr  = curr_mb_nr - 1;
            pix->available = 1;
             yM      = yN;
          }
        }
        else
        {
          // field
          if (curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrB;
            pix->available = currMb->mbAvailB;
            if (currMb->mbAvailB)
            {
              if(!img->mb_data[currMb->mbAddrB].mb_field)
              {
                (pix->mb_addr)++;
                 yM = 2* yN;
              }
              else
              {
                 yM = yN;
              }
            }
          }
          else
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrB + 1;
            pix->available = currMb->mbAvailB;
             yM      = yN;
          }
        }
      }
      else
      {
        // yN >=0
        // for the deblocker if this is the extra edge then do this special stuff
        if (yN == 0 && img->DeblockCall == 2)
        {
          pix->mb_addr  = currMb->mbAddrB + 1;
          pix->available = 1;
           yM      = yN - 1;
        }

        else if ((yN >= 0) && (yN <maxWH))
        {
          pix->mb_addr  = curr_mb_nr;
          pix->available = 1;
           yM      = yN;
        }
      }
    }
    else
    {
      // xN >= maxWH
      if(yN < 0)
      {
        if (!currMb->mb_field)
        {
          // frame
          if (curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrC + 1;
            pix->available = currMb->mbAvailC;
             yM      = yN;
          }
          else
          {
            // bottom
            pix->available = 0;
          }
        }
        else
        {
          // field
          if (curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrC;
            pix->available = currMb->mbAvailC;
            if (currMb->mbAvailC)
            {
              if(!img->mb_data[currMb->mbAddrC].mb_field)
              {
                (pix->mb_addr)++;
                 yM = 2* yN;
              }
              else
              {
                 yM = yN;
              }
            }
          }
          else
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrC + 1;
            pix->available = currMb->mbAvailC;
             yM      = yN;
          }
        }
      }
    }
  }
  if (pix->available || img->DeblockCall)
  {
    pix->x = (xN + maxWH) % maxWH;
    pix->y = (yM + maxWH) % maxWH;
    get_mb_pos(pix->mb_addr, &(pix->pos_x), &(pix->pos_y));
    if (luma)
    {
      pix->pos_x += pix->x;
      pix->pos_y += pix->y;
    }
    else
    {
      pix->pos_x = (pix->pos_x/2) + pix->x;
      pix->pos_y = (pix->pos_y/2) + pix->y;
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    get neighbouring positions. MB AFF is automatically used from img structure
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
 // xN yN 是以当前块左上角为原点的像素坐标  偏移坐标
void getNeighbour(int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)
{
  if (curr_mb_nr<0)
    error ("getNeighbour: invalid macroblock number", 100);

  if (dec_picture->MbaffFrameFlag)
    getAffNeighbour(curr_mb_nr, xN, yN, luma, pix);
  else
    getNonAffNeighbour(curr_mb_nr, xN, yN, luma, pix);
}


/*!
 ************************************************************************
 * \brief
 *    get neighbouring  get neighbouring 4x4 luma block
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param block_x
 *    input x block position
 * \param block_y
 *    input y block position
 * \param rel_x
 *    relative x position of neighbor
 * \param rel_y
 *    relative y position of neighbor
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getLuma4x4Neighbour (int curr_mb_nr, int block_x, int block_y, int rel_x, int rel_y, PixelPos *pix)
{
/*
	curr_mb_nr:当前16*16宏块的地址标号
	block_x：以当前16*16宏块左上角为原点，以4*4单位的块坐标(分块左上角)
	rel_x:以当前分块左上角为原点，以像素单位的相对偏移坐标，是相邻像素的相对坐标
	pix:(x,y)坐标对应像素的信息
*/
  int x = 4* block_x + rel_x;//以当前16*16宏块左上角为原点，对应像素点的像素坐标，和rel_x的坐标原点不同
  int y = 4* block_y + rel_y;

  getNeighbour(curr_mb_nr, x, y, 1, pix);

  if (pix->available)
  {
    pix->x /= 4;//除以4应该是转换成4*4块坐标
    pix->y /= 4;
    pix->pos_x /= 4;// pix->pos_x是以原点为坐标 的像素坐标? 为什么要除以4
    pix->pos_y /= 4;
  }
}


/*!
 ************************************************************************
 * \brief
 *    get neighbouring 4x4 chroma block
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param block_x
 *    input x block position
 * \param block_y
 *    input y block position
 * \param rel_x
 *    relative x position of neighbor
 * \param rel_y
 *    relative y position of neighbor
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getChroma4x4Neighbour (int curr_mb_nr, int block_x, int block_y, int rel_x, int rel_y, PixelPos *pix)
{
  int x = 4* block_x + rel_x;
  int y = 4* block_y + rel_y;

  getNeighbour(curr_mb_nr, x, y, 0, pix);

  if (pix->available)
  {
    pix->x /= 4;
    pix->y /= 4;
    pix->pos_x /= 4;
    pix->pos_y /= 4;
  }
}
