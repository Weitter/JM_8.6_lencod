
/*!
 **************************************************************************************
 * \file
 *    parset.c
 * \brief
 *    Picture and Sequence Parameter set generation and handling
 *  \date 25 November 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details) 
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 *
 **************************************************************************************
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "parsetcommon.h"
#include "memalloc.h"
/*! 
 *************************************************************************************
 * \brief
 *    Allocates memory for a pps
 *
 * \return
 *    pointer to a pps
 *************************************************************************************
 */
 
pic_parameter_set_rbsp_t *AllocPPS ()
 {
   pic_parameter_set_rbsp_t *p;

   if ((p=calloc (sizeof (pic_parameter_set_rbsp_t), 1)) == NULL)
     no_mem_exit ("AllocPPS: PPS");
   if ((p->slice_group_id = calloc (SIZEslice_group_id, 1)) == NULL)
     no_mem_exit ("AllocPPS: slice_group_id");
   return p;
 }

 
/*! 
 *************************************************************************************
 * \brief
 *    Allocates memory for am sps
 *
 * \return
 *    pointer to a sps
 *************************************************************************************
 */
 
seq_parameter_set_rbsp_t *AllocSPS ()
 {
   seq_parameter_set_rbsp_t *p;

   if ((p=calloc (sizeof (seq_parameter_set_rbsp_t), 1)) == NULL)
     no_mem_exit ("AllocSPS: SPS");
   return p;
 }

 
/*! 
 *************************************************************************************
 * \brief
 *    Frees a pps
 *
 * \param pps
 *     pps to be freed
 *
 * \return
 *    none
 *************************************************************************************
 */
 
 void FreePPS (pic_parameter_set_rbsp_t *pps)
 {
   assert (pps != NULL);
   if (pps->slice_group_id != NULL) free (pps->slice_group_id);
   free (pps);
 }

 
 /*! 
 *************************************************************************************
 * \brief
 *    Frees a sps
 *
 * \param sps
 *     sps to be freed
 *
 * \return
 *    none
 *************************************************************************************
 */
 
 void FreeSPS (seq_parameter_set_rbsp_t *sps)
 {
   assert (sps != NULL);
   free (sps); 
 }
