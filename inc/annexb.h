
/*!
 **************************************************************************************
 * \file
 *    annexb.h
 * \brief
 *    Byte stream operations support
 *    This code reflects JVT version xxx
 *  \date 7 December 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details) 
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */

#include <stdio.h>
#include "nalucommon.h"

int WriteAnnexbNALU (NALU_t *n);
void CloseAnnexbFile();
void OpenAnnexbFile (char *Filename);
