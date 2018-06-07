#include <node.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <kmti/gimbal/MotorCommand.hpp>
#include <uavcan/equipment/camera_gimbal/AngularCommand.hpp>
#include <uavcan/equipment/camera_gimbal/Mode.hpp>
#include <kmti/gimbal/MotorStatus.hpp>

#include <ch.hpp>
#include <hal.h>
#include <pwmio.h>
#include <attitude.h>
#include <misc.h>

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
            q[0] = msg.quaternion_xyzw[3];
            q[1] = msg.quaternion_xyzw[0];
            q[2] = msg.quaternion_xyzw[1];
            q[3] = msg.quaternion_xyzw[2];
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
            } else {
                for(uint8_t i = 0; i < 3; i++) {
                    g_canInput[i].mode = INPUT_MODE_NONE;
                }
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

    if(mot_sub_start_res < 0) {

    }

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

