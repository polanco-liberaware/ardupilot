#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_VisualOdom_config.h"

#define LOG_IDS_FROM_VISUALODOM \
    LOG_VISUALODOM_MSG, \
    LOG_VISUALPOS_MSG, \
    LOG_VISUALVEL_MSG, \
    LOG_VISBODYVEL_MSG

// @LoggerMessage: VISO
// @Description: Visual Odometry
// @Field: TimeUS: System time
// @Field: dt: Time period this data covers
// @Field: AngDX: Angular change for body-frame roll axis
// @Field: AngDY: Angular change for body-frame pitch axis
// @Field: AngDZ: Angular change for body-frame z axis
// @Field: PosDX: Position change for body-frame X axis (Forward-Back)
// @Field: PosDY: Position change for body-frame Y axis (Right-Left)
// @Field: PosDZ: Position change for body-frame Z axis (Down-Up)
// @Field: conf: Confidence
struct PACKED log_VisualOdom {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float time_delta;
    float angle_delta_x;
    float angle_delta_y;
    float angle_delta_z;
    float position_delta_x;
    float position_delta_y;
    float position_delta_z;
    float confidence;
};

// @LoggerMessage: VISP
// @Description: Vision Position
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: PX: Position X-axis (North-South)
// @Field: PY: Position Y-axis (East-West)
// @Field: PZ: Position Z-axis (Down-Up)
// @Field: R: Roll lean angle
// @Field: P: Pitch lean angle
// @Field: Y: Yaw angle
// @Field: PErr: Position estimate error
// @Field: AErr: Attitude estimate error
// @Field: Rst: Position reset counter
// @Field: Ign: Ignored
// @Field: Q: Quality
struct PACKED log_VisualPosition {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float pos_x;
    float pos_y;
    float pos_z;
    float roll;     // degrees
    float pitch;    // degrees
    float yaw;      // degrees
    float pos_err;  // meters
    float ang_err;  // radians
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: VISV
// @Description: Vision Velocity
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: VX: Velocity X-axis (North-South)
// @Field: VY: Velocity Y-axis (East-West)
// @Field: VZ: Velocity Z-axis (Down-Up)
// @Field: VErr: Velocity estimate error
// @Field: Rst: Velocity reset counter
// @Field: Ign: Ignored
// @Field: Q: Quality
struct PACKED log_VisualVelocity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float vel_x;
    float vel_y;
    float vel_z;
    float vel_err;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: VISBV
// @Description: Vision Body-Frame Velocity (radar RIO route)
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: VX: Body forward velocity (m/s)
// @Field: VY: Body right velocity (m/s)
// @Field: VZ: Body down velocity (m/s)
// @Field: WX: Body roll rate (rad/s)
// @Field: WY: Body pitch rate (rad/s)
// @Field: WZ: Body yaw rate (rad/s)
// @Field: VErr: Velocity estimate error (m/s)
// @Field: Rst: Reset counter
// @Field: Ign: Ignored (quality below threshold)
// @Field: Q: Quality
struct PACKED log_VisualBodyVelocity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float vel_x;
    float vel_y;
    float vel_z;
    float ang_x;
    float ang_y;
    float ang_z;
    float vel_err;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

#if HAL_VISUALODOM_ENABLED
#define LOG_STRUCTURE_FROM_VISUALODOM \
    { LOG_VISUALODOM_MSG, sizeof(log_VisualOdom), \
      "VISO", "Qffffffff", "TimeUS,dt,AngDX,AngDY,AngDZ,PosDX,PosDY,PosDZ,conf", "ssrrrmmm-", "FF000000-" }, \
    { LOG_VISUALPOS_MSG, sizeof(log_VisualPosition), \
      "VISP", "QQIffffffffBBb", "TimeUS,RTimeUS,CTimeMS,PX,PY,PZ,R,P,Y,PErr,AErr,Rst,Ign,Q", "sssmmmddhmd--%", "FFC00000000--0" }, \
    { LOG_VISUALVEL_MSG, sizeof(log_VisualVelocity), \
      "VISV", "QQIffffBBb", "TimeUS,RTimeUS,CTimeMS,VX,VY,VZ,VErr,Rst,Ign,Q", "sssnnnn--%", "FFC0000--0" }, \
    { LOG_VISBODYVEL_MSG, sizeof(log_VisualBodyVelocity), \
      "VISBV", "QQIfffffffBBb", "TimeUS,RTimeUS,CTimeMS,VX,VY,VZ,WX,WY,WZ,VErr,Rst,Ign,Q", "sssnnnnnn---%", "FFC0000000--0" },
#else
#define LOG_STRUCTURE_FROM_VISUALODOM
#endif
