
/*!
 ************************************************************************
 * \file image.h
 *
 * \author
 *  Inge Lille-Lang�y               <inge.lille-langoy@telenor.com>
 *  Copyright (C) 1999  Telenor Satellite Services, Norway
 ************************************************************************
 */
#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "mbuffer.h"

extern StorablePicture *enc_picture;
extern StorablePicture *enc_frame_picture;
extern StorablePicture *enc_top_picture;
extern StorablePicture *enc_bottom_picture;

int encode_one_frame ();
Boolean dummy_slice_too_big(int bits_slice);
void copy_rdopt_data (int field_type);    //!< For MB level field/frame coding tools

void UnifiedOneForthPix (StorablePicture *s);

#endif

