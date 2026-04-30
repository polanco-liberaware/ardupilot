#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_VisualOdom_config.h"

#define LOG_IDS_FROM_VISUALODOM \
    LOG_VISUALODOM_MSG, \
    LOG_VISUALPOS_MSG, \
    LOG_VISUALVEL_MSG, \
    LOG_RIOPOS_MSG, \
    LOG_RIOATT_MSG, \
    LOG_RIOVEL_MSG, \
    LOG_RIOPOSCOV_MSG, \
    LOG_RIOVELCOV_MSG, \
    LOG_RIOATTCOV_MSG, \
    LOG_RIOSTATUS_MSG

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

// @LoggerMessage: RIOP
// @Description: Radar inertial navigation position from the raw CC packet in MAV_FRAME_LOCAL_FRD
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: FTimeMS: FC boot-time measurement timestamp derived directly from the packet time_usec field
// @Field: PX: Raw CC position X in local FRD
// @Field: PY: Raw CC position Y in local FRD
// @Field: PZ: Raw CC position Z in local FRD
// @Field: Rst: Packet reset counter
// @Field: Ign: Packet ignored by quality gate
// @Field: Q: Quality
struct PACKED log_RIOPosition {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float pos_x;
    float pos_y;
    float pos_z;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: RIOA
// @Description: Radar inertial navigation attitude from the raw CC packet as a body-to-local-FRD quaternion
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: FTimeMS: FC boot-time measurement timestamp derived directly from the packet time_usec field
// @Field: QW: Raw CC quaternion W for body->local FRD
// @Field: QX: Raw CC quaternion X for body->local FRD
// @Field: QY: Raw CC quaternion Y for body->local FRD
// @Field: QZ: Raw CC quaternion Z for body->local FRD
// @Field: Rst: Packet reset counter
// @Field: Ign: Packet ignored by quality gate
// @Field: Q: Quality
struct PACKED log_RIOAttitude {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: RIOPV
// @Description: Radar inertial navigation position covariance from the raw CC packet in MAV_FRAME_LOCAL_FRD
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: FTimeMS: FC boot-time measurement timestamp derived directly from the packet time_usec field
// @Field: Pxx: Raw CC position covariance XX in local FRD
// @Field: Pxy: Raw CC position covariance XY in local FRD
// @Field: Pxz: Raw CC position covariance XZ in local FRD
// @Field: Pyy: Raw CC position covariance YY in local FRD
// @Field: Pyz: Raw CC position covariance YZ in local FRD
// @Field: Pzz: Raw CC position covariance ZZ in local FRD
// @Field: Rst: Packet reset counter
// @Field: Ign: Packet ignored by quality gate
// @Field: Q: Quality
struct PACKED log_RIOPosCov {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float pos_cov_xx;
    float pos_cov_xy;
    float pos_cov_xz;
    float pos_cov_yy;
    float pos_cov_yz;
    float pos_cov_zz;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: RIOV
// @Description: Radar inertial navigation velocity from the raw CC packet in MAV_FRAME_BODY_FRD
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: FTimeMS: FC boot-time measurement timestamp derived directly from the packet time_usec field
// @Field: VX: Raw CC velocity X in body FRD
// @Field: VY: Raw CC velocity Y in body FRD
// @Field: VZ: Raw CC velocity Z in body FRD
// @Field: Rst: Packet reset counter
// @Field: Ign: Packet ignored by quality gate
// @Field: Q: Quality
struct PACKED log_RIOVelocity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float vel_x;
    float vel_y;
    float vel_z;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: RIOVV
// @Description: Radar inertial navigation velocity covariance from the raw CC packet in MAV_FRAME_BODY_FRD
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: FTimeMS: FC boot-time measurement timestamp derived directly from the packet time_usec field
// @Field: Vxx: Raw CC velocity covariance XX in body FRD
// @Field: Vxy: Raw CC velocity covariance XY in body FRD
// @Field: Vxz: Raw CC velocity covariance XZ in body FRD
// @Field: Vyy: Raw CC velocity covariance YY in body FRD
// @Field: Vyz: Raw CC velocity covariance YZ in body FRD
// @Field: Vzz: Raw CC velocity covariance ZZ in body FRD
// @Field: Rst: Packet reset counter
// @Field: Ign: Packet ignored by quality gate
// @Field: Q: Quality
struct PACKED log_RIOVelCov {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float vel_cov_xx;
    float vel_cov_xy;
    float vel_cov_xz;
    float vel_cov_yy;
    float vel_cov_yz;
    float vel_cov_zz;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: RIOAV
// @Description: Radar inertial navigation body-frame attitude covariance from the raw CC packet
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: FTimeMS: FC boot-time measurement timestamp derived directly from the packet time_usec field
// @Field: Axx: Raw CC body-frame attitude covariance XX
// @Field: Axy: Raw CC body-frame attitude covariance XY
// @Field: Axz: Raw CC body-frame attitude covariance XZ
// @Field: Ayy: Raw CC body-frame attitude covariance YY
// @Field: Ayz: Raw CC body-frame attitude covariance YZ
// @Field: Azz: Raw CC body-frame attitude covariance ZZ
// @Field: Rst: Packet reset counter
// @Field: Ign: Packet ignored by quality gate
// @Field: Q: Quality
struct PACKED log_RIOAttCov {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float att_cov_xx;
    float att_cov_xy;
    float att_cov_xz;
    float att_cov_yy;
    float att_cov_yz;
    float att_cov_zz;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};

// @LoggerMessage: RIOS
// @Description: Radar inertial navigation status/debug; delay is local FC-derived and all other fields come from the raw CC packet
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: FTimeMS: FC boot-time measurement timestamp derived directly from the packet time_usec field
// @Field: DlyMS: Estimated receive-side delay computed on the FC as now_ms - packet_time_ms
// @Field: Rst: Packet reset counter
// @Field: Q: Quality
// @Field: HS: Health state
// @Field: HR: Health reason
// @Field: Flags: Machine-readable status bitmask
// @Field: RJ: Last radar-estimator rejection reason
// @Field: Raw: Raw radar detections
// @Field: Valid: Detections accepted by the solver
// @Field: Cond: Solver geometry quality
// @Field: SigR0: Radar ego-velocity 1-sigma X
// @Field: SigR1: Radar ego-velocity 1-sigma Y
// @Field: SigR2: Radar ego-velocity 1-sigma Z
// @Field: RAge: Age of last accepted radar update
struct PACKED log_RIOStatus {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    uint16_t delay_ms;
    uint8_t reset_counter;
    int8_t quality;
    uint8_t health_state;
    uint8_t health_reason;
    uint32_t status_flags;
    uint8_t rejection_reason;
    uint16_t raw_point_count;
    uint16_t valid_point_count;
    float condition_number;
    float sigma_radar_x;
    float sigma_radar_y;
    float sigma_radar_z;
    float radar_age_sec;
};

#if HAL_VISUALODOM_ENABLED
#define LOG_STRUCTURE_FROM_VISUALODOM \
    { LOG_VISUALODOM_MSG, sizeof(log_VisualOdom), \
      "VISO", "Qffffffff", "TimeUS,dt,AngDX,AngDY,AngDZ,PosDX,PosDY,PosDZ,conf", "ssrrrmmm-", "FF000000-" }, \
    { LOG_VISUALPOS_MSG, sizeof(log_VisualPosition), \
      "VISP", "QQIffffffffBBb", "TimeUS,RTimeUS,CTimeMS,PX,PY,PZ,R,P,Y,PErr,AErr,Rst,Ign,Q", "sssmmmddhmd--%", "FFC00000000--0" }, \
    { LOG_VISUALVEL_MSG, sizeof(log_VisualVelocity), \
      "VISV", "QQIffffBBb", "TimeUS,RTimeUS,CTimeMS,VX,VY,VZ,VErr,Rst,Ign,Q", "sssnnnn--%", "FFC0000--0" }, \
    { LOG_RIOPOS_MSG, sizeof(log_RIOPosition), \
      "RIOP", "QQIfffBBb", "TimeUS,RTimeUS,FTimeMS,PX,PY,PZ,Rst,Ign,Q", "sssmmm--%", "FFC000--0" }, \
    { LOG_RIOATT_MSG, sizeof(log_RIOAttitude), \
      "RIOA", "QQIffffBBb", "TimeUS,RTimeUS,FTimeMS,QW,QX,QY,QZ,Rst,Ign,Q", "sss------%", "FFC-----0" }, \
    { LOG_RIOVEL_MSG, sizeof(log_RIOVelocity), \
      "RIOV", "QQIfffBBb", "TimeUS,RTimeUS,FTimeMS,VX,VY,VZ,Rst,Ign,Q", "sssnnn--%", "FFC000--0" }, \
    { LOG_RIOPOSCOV_MSG, sizeof(log_RIOPosCov), \
      "RIOPV", "QQIffffffBBb", "TimeUS,RTimeUS,FTimeMS,Pxx,Pxy,Pxz,Pyy,Pyz,Pzz,Rst,Ign,Q", "sss---------", "FFC---------" }, \
    { LOG_RIOVELCOV_MSG, sizeof(log_RIOVelCov), \
      "RIOVV", "QQIffffffBBb", "TimeUS,RTimeUS,FTimeMS,Vxx,Vxy,Vxz,Vyy,Vyz,Vzz,Rst,Ign,Q", "sss---------", "FFC---------" }, \
    { LOG_RIOATTCOV_MSG, sizeof(log_RIOAttCov), \
      "RIOAV", "QQIffffffBBb", "TimeUS,RTimeUS,FTimeMS,Axx,Axy,Axz,Ayy,Ayz,Azz,Rst,Ign,Q", "sss---------", "FFC---------" }, \
    { LOG_RIOSTATUS_MSG, sizeof(log_RIOStatus), \
      "RIOS", "QQIHBbBBIBHHfffff", "TimeUS,RTimeUS,FTimeMS,DlyMS,Rst,Q,HS,HR,Flags,RJ,Raw,Vld,Cond,S0,S1,S2,Age", "sss--------------", "FFCC-------------", true },
#else
#define LOG_STRUCTURE_FROM_VISUALODOM
#endif
