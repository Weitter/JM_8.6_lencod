
/*!
 **************************************************************************************
 * \file
 *    filehandle.c
 * \brief
 *    Start and terminate sequences
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Thomas Stockhammer            <stockhammer@ei.tum.de>
 *      - Detlev Marpe                  <marpe@hhi.de>
 ***************************************************************************************
 */

#include "contributors.h"

#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "global.h"
#include "header.h"
#include "rtp.h"
#include "nalu.h"
#include "annexb.h"
#include "parset.h"
#include "mbuffer.h"


/*!
 ************************************************************************
 * \brief
 *    Error handling procedure. Print error message to stderr and exit
 *    with supplied code.
 * \param text
 *    Error message
 * \param code
 *    Exit code
 ************************************************************************
 */
void error(char *text, int code)
{
  fprintf(stderr, "%s\n", text);
  flush_dpb();
  exit(code);
}

/*!
 ************************************************************************
 * \brief
 *    This function opens the output files and generates the
 *    appropriate sequence header
 ************************************************************************
 */
int start_sequence()
{
  int len=0;
  NALU_t *nalu;

  switch(input->of_mode)
  {
    case PAR_OF_ANNEXB:
      OpenAnnexbFile (input->outfile);
      WriteNALU = WriteAnnexbNALU;
      break;
    case PAR_OF_RTP:
      OpenRTPFile (input->outfile);
      WriteNALU = WriteRTPNALU;
      break;
    default:
      snprintf(errortext, ET_SIZE, "Output File Mode %d not supported", input->of_mode);
      error(errortext,1);
      return 1;
  }

  //! As a sequence header, here we write the both sequence and picture
  //! parameter sets.  As soon as IDR is implemented, this should go to the
  //! IDR part, as both parsets have to be transmitted as part of an IDR.
  //! An alterbative may be to consider this function the IDR start function.
  
  nalu = NULL;
  nalu = GenerateSeq_parameter_set_NALU ();
  len += WriteNALU (nalu);
  FreeNALU (nalu);
  nalu = NULL;
  nalu = GeneratePic_parameter_set_NALU ();
  len += WriteNALU (nalu);
  FreeNALU (nalu);

//  stat->bit_ctr_parametersets = len;
    stat->bit_ctr_parametersets_n = len;
  return 0;
}

/*!
 ************************************************************************
 * \brief
 *     This function terminates the sequence and closes the
 *     output files
 ************************************************************************
 */
int terminate_sequence()
{
//  Bitstream *currStream;

  // Mainly flushing of everything
  // Add termination symbol, etc.

  switch(input->of_mode)
  {
    case PAR_OF_ANNEXB:
      CloseAnnexbFile();
      break;
    case PAR_OF_RTP:
      CloseRTPFile();
      return 0;
    default:
      snprintf(errortext, ET_SIZE, "Output File Mode %d not supported", input->of_mode);
      error(errortext,1);
      return 1;
  }
  return 1;   // make lint happy
}

