
/*!
 **************************************************************************************
 * \file
 *    parsetcommon.h
 * \brief
 *    Picture and Sequence Parameter Sets, structures common to encoder and decoder
 *    This code reflects JVT version xxx
 *  \date 25 November 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details) 
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */



// In the JVT syntax, frequently flags are used that indicate the presence of
// certain pieces of information in the NALU.  Here, these flags are also
// present.  In the encoder, those bits indicate that the values signalled to
// be present are meaningful and that this part of the syntax should be
// written to the NALU.  In the decoder, the flag indicates that information
// was received from the decoded NALU and should be used henceforth.
// The structure names were chosen as indicated in the JVT syntax

#ifndef _PARSETCOMMON_H_
#define _PARSETCOMMON_H_

#define MAXIMUMPARSETRBSPSIZE   1500
#define MAXIMUMPARSETNALUSIZE   1500
#define SIZEslice_group_id      (sizeof (int) * 60000)    // should be sufficient for HUGE pictures, need one int per MB in a picture

#define MAXSPS  32
#define MAXPPS  256

//! Boolean Type
typedef enum {
  FALSE,
  TRUE
} Boolean;

#define MAXIMUMVALUEOFcpb_cnt   32
typedef struct
{
  unsigned  cpb_cnt;                                          // ue(v)
  unsigned  bit_rate_scale;                                   // u(4)
  unsigned  cpb_size_scale;                                   // u(4)
    unsigned  bit_rate_value [MAXIMUMVALUEOFcpb_cnt];         // ue(v)
    unsigned  cpb_size_value[MAXIMUMVALUEOFcpb_cnt];          // ue(v)
    unsigned  vbr_cbr_flag[MAXIMUMVALUEOFcpb_cnt];            // u(1)
  unsigned  initial_cpb_removal_delay_length_minus1;          // u(5)
  unsigned  cpb_removal_delay_length_minus1;                  // u(5)
  unsigned  dpb_output_delay_length_minus1;                   // u(5)
  unsigned  time_offset_length;                               // u(5)
} hrd_parameters_t;


typedef struct
{
  Boolean      aspect_ratio_info_present_flag;                   // u(1)
    unsigned  aspect_ratio_idc;                               // u(8)
      unsigned  sar_width;                                    // u(16)
      unsigned  sar_height;                                   // u(16)
  Boolean      overscan_info_present_flag;                       // u(1)
    Boolean      overscan_appropriate_flag;                      // u(1)
  Boolean      video_signal_type_present_flag;                   // u(1)
    unsigned  video_format;                                   // u(3)
    Boolean      video_full_range_flag;                          // u(1)
    Boolean      colour_description_present_flag;                // u(1)
      unsigned  colour_primaries;                             // u(8)
      unsigned  transfer_characteristics;                     // u(8)
      unsigned  matrix_coefficients;                          // u(8)
  Boolean      chroma_location_info_present_flag;                // u(1)
    unsigned  chroma_location_frame;                          // ue(v)
    unsigned  chroma_location_field;                          // ue(v)
  Boolean      timing_info_present_flag;                         // u(1)
    unsigned  num_units_in_tick;                              // u(32)
    unsigned  time_scale;                                     // u(32)
    Boolean      fixed_frame_rate_flag;                          // u(1)
  Boolean      nal_hrd_parameters_present_flag;                  // u(1)
    hrd_parameters_t nal_hrd_parameters;                      // hrd_paramters_t
  Boolean      vcl_hrd_parameters_present_flag;                  // u(1)
    hrd_parameters_t vcl_hrd_parameters;                      // hrd_paramters_t
  // if ((nal_hrd_parameters_present_flag || (vcl_hrd_parameters_present_flag))
    Boolean      low_delay_hrd_flag;                             // u(1)
  Boolean      bitstream_restriction_flag;                       // u(1)
    Boolean      motion_vectors_over_pic_boundaries_flag;        // u(1)
    unsigned  max_bytes_per_pic_denom;                        // ue(v)
    unsigned  max_bits_per_mb_denom;                          // ue(v)
    unsigned  log2_max_mv_length_vertical;                    // ue(v)
    unsigned  log2_max_mv_length_horizontal;                  // ue(v)
    unsigned  max_dec_frame_reordering;                       // ue(v)
    unsigned  max_dec_frame_buffering;                        // ue(v)
} vui_seq_parameters_t;


#define MAXnum_slice_groups_minus1  8
typedef struct
{
  Boolean   Valid;                  // indicates the parameter set is valid
  unsigned  pic_parameter_set_id;                             // ue(v)
  unsigned  seq_parameter_set_id;                             // ue(v)
  Boolean   entropy_coding_mode_flag;                         // u(1)
  // if( pic_order_cnt_type < 2 )  in the sequence parameter set
  Boolean      pic_order_present_flag;                           // u(1)
  unsigned  num_slice_groups_minus1;                          // ue(v)
    unsigned  slice_group_map_type;                        // ue(v)
    // if( slice_group_map_type = = 0 )
      unsigned  run_length_minus1[MAXnum_slice_groups_minus1]; // ue(v)
    // else if( slice_group_map_type = = 2 )
      unsigned  top_left[MAXnum_slice_groups_minus1];         // ue(v)
      unsigned  bottom_right[MAXnum_slice_groups_minus1];     // ue(v)
    // else if( slice_group_map_type = = 3 || 4 || 5
      Boolean   slice_group_change_direction_flag;            // u(1)
      unsigned  slice_group_change_rate_minus1;               // ue(v)
    // else if( slice_group_map_type = = 6 )
      unsigned  pic_size_in_map_units_minus1;	                // ue(v)
      unsigned  *slice_group_id;                              // complete MBAmap u(v)
			
  int       num_ref_idx_l0_active_minus1;                     // ue(v)
  int       num_ref_idx_l1_active_minus1;                     // ue(v)
  Boolean   weighted_pred_flag;                               // u(1)
  Boolean   weighted_bipred_idc;                              // u(2)
  int       pic_init_qp_minus26;                              // se(v)
  int       pic_init_qs_minus26;                              // se(v)
  int       chroma_qp_index_offset;                           // se(v)
  Boolean   deblocking_filter_control_present_flag;           // u(1)
  Boolean   constrained_intra_pred_flag;                      // u(1)
  Boolean   redundant_pic_cnt_present_flag;                   // u(1)
  Boolean   vui_pic_parameters_flag;                          // u(1)
} pic_parameter_set_rbsp_t;


#define MAXnum_ref_frames_in_pic_order_cnt_cycle  256
typedef struct
{
  Boolean   Valid;                  // indicates the parameter set is valid

  unsigned  profile_idc;                                      // u(8)
  Boolean   constrained_set0_flag;                            // u(1)
  Boolean   constrained_set1_flag;                            // u(1)
  Boolean   constrained_set2_flag;                            // u(1)
  unsigned  level_idc;                                        // u(8)
  unsigned  seq_parameter_set_id;                             // ue(v)
  unsigned  log2_max_frame_num_minus4;                        // ue(v)
  unsigned pic_order_cnt_type;
  // if( pic_order_cnt_type == 0 ) 
  unsigned log2_max_pic_order_cnt_lsb_minus4;                 // ue(v)
  // else if( pic_order_cnt_type == 1 )
    Boolean delta_pic_order_always_zero_flag;               // u(1)
    int     offset_for_non_ref_pic;                         // se(v)
    int     offset_for_top_to_bottom_field;                 // se(v)
    unsigned  num_ref_frames_in_pic_order_cnt_cycle;          // ue(v)
    // for( i = 0; i < num_ref_frames_in_pic_order_cnt_cycle; i++ )
      int   offset_for_ref_frame[MAXnum_ref_frames_in_pic_order_cnt_cycle];   // se(v)
  unsigned  num_ref_frames;                                   // ue(v)
  Boolean   gaps_in_frame_num_value_allowed_flag;             // u(1)
  unsigned  pic_width_in_mbs_minus1;                          // ue(v)
  unsigned  pic_height_in_map_units_minus1;                   // ue(v)
  Boolean   frame_mbs_only_flag;                              // u(1)
  // if( !frame_mbs_only_flag ) 
    Boolean   mb_adaptive_frame_field_flag;                   // u(1)
  Boolean   direct_8x8_inference_flag;                        // u(1)
  Boolean   frame_cropping_flag;                              // u(1)
    unsigned  frame_cropping_rect_left_offset;                // ue(v)
    unsigned  frame_cropping_rect_right_offset;               // ue(v)
    unsigned  frame_cropping_rect_top_offset;                 // ue(v)
    unsigned  frame_cropping_rect_bottom_offset;              // ue(v)
  Boolean   vui_parameters_present_flag;                      // u(1)
    vui_seq_parameters_t vui_seq_parameters;                  // vui_seq_parameters_t
} seq_parameter_set_rbsp_t;

pic_parameter_set_rbsp_t *AllocPPS ();
seq_parameter_set_rbsp_t *AllocSPS ();
void FreePPS (pic_parameter_set_rbsp_t *pps);
void FreeSPS (seq_parameter_set_rbsp_t *sps);

#endif
