#include <node.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <kmti/gimbal/MotorCommand.hpp>
#include <uavcan/equipment/camera_gimbal/AngularCommand.hpp>
#include <uavcan/equipment/camera_gimbal/GEOPOICommand.hpp>
#include <uavcan/equipment/camera_gimbal/Mode.hpp>
#include <kmti/gimbal/MotorStatus.hpp>

#include <uavcan/equipment/ahrs/Solution.hpp>
#include <uavcan/equipment/gnss/Fix2.hpp>

#include <ch.hpp>
#include <hal.h>
#include <pwmio.h>
#include <attitude.h>
#include <misc.h>
#include <misc.hpp>

binary_semaphore_t motor_data;

namespace Node {

  uavcan_stm32::CanInitHelper<> can;

  uavcan::Node<NodePoolSize>& getNode() {
    static uavcan::Node<NodePoolSize> node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
  }

  void uavcanNodeThread::main() {

    setName("uavcan_node");
    uavcan::uint32_t bitrate = 1000000;
    can.init(bitrate);

    getNode().setName("org.kmti.gcm_controler");
    getNode().setNodeID(15);

    if (getNode().start() < 0) {
      chSysHalt("UAVCAN init fail");
    }

    systime_t last_command = 0;

    uavcan::Subscriber<uavcan::equipment::camera_gimbal::AngularCommand> ang_sub(getNode());
    const int mot_sub_start_res = ang_sub.start(
        [&](const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand>& msg)
        {
            last_command = chVTGetSystemTime();
            float cmd[3];
            uint8_t mode = msg.mode.command_mode;
            float q[4];
            //Here we mix and invert axis to adhere to UAVCAN rotation represantation
            q[0] = msg.quaternion_xyzw[3]; // w
            q[1] = msg.quaternion_xyzw[1]; // gimbal pitch axis (x)
            q[2] = msg.quaternion_xyzw[0]; // gimbal roll axis (y)
            q[3] = -msg.quaternion_xyzw[2]; // gimbal yaw axis (z)
            Quaternion2RPY(q, cmd);
            if(mode == uavcan::equipment::camera_gimbal::Mode::COMMAND_MODE_ANGULAR_VELOCITY) {
                for(uint8_t i = 0; i < 3; i++) {
                    g_canInput[i].mode = INPUT_MODE_SPEED;
                    g_canInput[i].cmd = cmd[i];
                }
            } else if(mode == uavcan::equipment::camera_gimbal::Mode::COMMAND_MODE_ORIENTATION_FIXED_FRAME) {
                for(uint8_t i = 0; i < 3; i++) {
                    g_canInput[i].mode = INPUT_MODE_ANGLE;
                    g_canInput[i].cmd = cmd[i];
                }
            } else if(mode == uavcan::equipment::camera_gimbal::Mode::COMMAND_MODE_ORIENTATION_BODY_FRAME) {
                for(uint8_t i = 0; i < 3; i++) {
                    g_canInput[i].mode = INPUT_MODE_BODYFRAME;
                    g_canInput[i].cmd = cmd[i];
                }

            } else {
                for(uint8_t i = 0; i < 3; i++) {
                    g_canInput[i].mode = INPUT_MODE_NONE;
                }
            }
        });

    uavcan::Subscriber<uavcan::equipment::camera_gimbal::GEOPOICommand> geo_poi_sub(getNode());
    geo_poi_sub.start(
            [&](const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::GEOPOICommand>& msg)
            {
                Location target(msg.latitude_deg_1e7/1e7f, msg.longitude_deg_1e7/1e7f, msg.height_cm/100.0f);
                g_target_loc = target;
                for(int i = 0; i < 3; i++) {
                    g_canInput[i].mode = INPUT_MODE_GPS_COORD;
                }
            });

    uavcan::Subscriber<kmti::gimbal::MotorStatus> mot_stat_sub(getNode());
    const int mot_stat_sub_res = mot_stat_sub.start(
            [&](const uavcan::ReceivedDataStructure<kmti::gimbal::MotorStatus>& msg)
            {
                if(msg.axis_id >= 0 && msg.axis_id < 3) {
                    g_motorOffset[msg.axis_id] = msg.motor_pos_rad;
                }
            });

    uavcan::Subscriber<uavcan::equipment::ahrs::Solution> ardupilot_ahrs(getNode());
    const int ardupilot_ahrs_sub_res = ardupilot_ahrs.start(
            [&](const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::Solution>& msg)
            {
                float cmd[3];
                Quaterion autopilot_orient(msg.orientation_xyzw[3], msg.orientation_xyzw[0], msg.orientation_xyzw[1],
                        msg.orientation_xyzw[2]);
                g_autopilot_attitude = autopilot_orient;
            });

    uavcan::Subscriber<uavcan::equipment::gnss::Fix2> ardupilot_fix(getNode());
    ardupilot_fix.start(
            [&](const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix2>& msg)
            {
                Location loc(msg.latitude_deg_1e8 / 1e8, msg.longitude_deg_1e8 / 1e8, msg.height_msl_mm / 1e3);
                g_location = loc;
            });

    uavcan::Publisher<kmti::gimbal::MotorCommand> mot_pub(getNode());
    mot_pub.init();
    kmti::gimbal::MotorCommand mot_msg;

    getNode().setModeOperational();

    while(true) {
      if (getNode().spin(uavcan::MonotonicDuration::fromMSec(10)) < 0) {
        chSysHalt("UAVCAN spin fail");
      }
      if(last_command != 0 && last_command + MS2ST(200) < chVTGetSystemTime()) {
          last_command = 0;
          for(uint8_t i = 0; i < 3; i++) {
              if(g_canInput[i].mode == INPUT_MODE_SPEED) {
                  g_canInput[i].cmd = 0.0f;
              }
          }
      }
      for(int i = 0; i < 3; i++) {
          mot_msg.cmd[i] = g_pwmCmd[i].phase;
      }
        mot_pub.broadcast(mot_msg);
    }
  }

}

