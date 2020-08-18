
/*!
 **************************************************************************************
 * \file
 *    parset.h
 * \brief
 *    Picture and Sequence Parameter Sets, encoder operations
 *    This code reflects JVT version xxx
 *  \date 25 November 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details) 
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */


#ifndef _PARSET_H_
#define _PARSET_H_

#include "parsetcommon.h"
#include "nalu.h"

void GenerateParameterSets ();
void FreeParameterSets ();

NALU_t *GenerateSeq_parameter_set_NALU ();
NALU_t *GeneratePic_parameter_set_NALU ();

// The following are local helpers, but may come handy in the future, hence public
void FillParameterSetStructures (seq_parameter_set_rbsp_t *sps, pic_parameter_set_rbsp_t *pps);
int GenerateSeq_parameter_set_rbsp (seq_parameter_set_rbsp_t *sps, char *buf);
int GeneratePic_parameter_set_rbsp (pic_parameter_set_rbsp_t *pps, char *buf);
void FreeSPS (seq_parameter_set_rbsp_t *sps);
void FreePPS (pic_parameter_set_rbsp_t *pps);
pic_parameter_set_rbsp_t *AllocPPS ();
seq_parameter_set_rbsp_t *AllocSPS ();


#endif
