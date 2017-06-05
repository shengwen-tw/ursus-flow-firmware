#ifndef __FCB_LINK_H__
#define __FCB_LINK_H__

void send_flow_to_fcb(uint16_t _lidar_distance, float _flow_vx,
                      float _flow_vy, float _time, float period, float _frequency);

#endif
