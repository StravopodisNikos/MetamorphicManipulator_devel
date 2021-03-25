void setup_libraries_packets()
{
  // UPDATE ROBOT CONFIGURATION PACKET
  dxl_pp_packet.Dxl_ids       = dxl_id;
  dxl_pp_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pp_packet.dxl_pp        = dxl_present_position;
  dxl_pp_packet.SR_pp         = sr_data_array_pp;
  dxl_pp_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pp_packet = &dxl_pp_packet;
  
  // UPDATE ROBOT ANGULAR VELOCITY PACKET
  dxl_pv_packet.Dxl_ids       = dxl_id;
  dxl_pv_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pv_packet.dxl_pv        = dxl_present_velocity;
  dxl_pv_packet.SR_pv         = sr_data_array_pv;
  dxl_pv_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pv_packet = &dxl_pv_packet;

  // UPDATE DXL MOVING PACKET
  dxl_mov_packet.Dxl_ids      = dxl_id;
  dxl_mov_packet.Dxl_ids_size = sizeof(dxl_id);
  dxl_mov_packet.dxl_mov      = dxl_moving;
  dxl_mov_packet.SR_mov       = sr_data_array_mov;
  dxl_mov_packet.dxl2ard_obj  = dxl;

  PTR_2_dxl_mov_packet = &dxl_mov_packet;

  // UPDATE DXL CURRENT MEASUREMENT
  dxl_pc_packet.Dxl_ids       = dxl_id;
  dxl_pc_packet.Dxl_ids_size  = sizeof(dxl_id);
  dxl_pc_packet.dxl_pc        = dxl_present_current;
  dxl_pc_packet.SR_pc         = sr_data_array_pc;
  dxl_pc_packet.dxl2ard_obj   = dxl;

  PTR_2_dxl_pc_packet = &dxl_pc_packet;
  
  return;
}
