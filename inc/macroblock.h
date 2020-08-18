
/*!
 ************************************************************************
 * \file
 *    macroblock.h
 *
 * \author
 *    Inge Lille-Lang�y               <inge.lille-langoy@telenor.com>     \n
 *    Telenor Satellite Services                                          \n
 *    P.O.Box 6914 St.Olavs plass                                         \n
 *    N-0130 Oslo, Norway
 *
 ************************************************************************/
#ifndef _MACROBLOCK_H_
#define _MACROBLOCK_H_


//! just to make new temp intra mode table
const int  MODTAB[3][2]=
{
  { 0, 4},
  {16,12},
  { 8,20}
};

//! gives codeword number from CBP value, both for intra and inter
const int NCBP[48][2]=
{
  { 3, 0},{29, 2},{30, 3},{17, 7},{31, 4},{18, 8},{37,17},{ 8,13},{32, 5},{38,18},{19, 9},{ 9,14},
  {20,10},{10,15},{11,16},{ 2,11},{16, 1},{33,32},{34,33},{21,36},{35,34},{22,37},{39,44},{ 4,40},
  {36,35},{40,45},{23,38},{ 5,41},{24,39},{ 6,42},{ 7,43},{ 1,19},{41, 6},{42,24},{43,25},{25,20},
  {44,26},{26,21},{46,46},{12,28},{45,27},{47,47},{27,22},{13,29},{28,23},{14,30},{15,31},{ 0,12},
};


extern int QP2QUANT[40];

#endif

