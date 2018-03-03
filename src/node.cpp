#include <node.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <kmti/gimbal/MotorCommand.hpp>
#include <uavcan/equipment/camera_gimbal/AngularCommand.hpp>
#include <uavcan/equipment/camera_gimbal/Mode.hpp>

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

    getNode().setName("org.kmti.gmm_controler");
    getNode().setNodeID(10);

    if (getNode().start() < 0) {
      chSysHalt("UAVCAN init fail");
    }

    uavcan::Subscriber<uavcan::equipment::camera_gimbal::AngularCommand> ang_sub(getNode());
    const int mot_sub_start_res = ang_sub.start(
        [&](const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand>& msg)
        {
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

    if(mot_sub_start_res < 0) {

    }

    uavcan::Publisher<kmti::gimbal::MotorCommand> mot_pub(getNode());
    mot_pub.init();
    kmti::gimbal::MotorCommand mot_msg;
    mot_msg.power[0] = 0.5f;

    getNode().setModeOperational();

    while(true) {
      if (getNode().spin(uavcan::MonotonicDuration::fromMSec(2)) < 0) {
        chSysHalt("UAVCAN spin fail");
      }
      for(int i = 0; i < 3; i++) {
          mot_msg.cmd[i] = g_pwmCmd[i].phase;
          mot_msg.power[i] = g_pwmCmd[i].power/100.0f;
      }
        mot_pub.broadcast(mot_msg);
    }
  }

}

