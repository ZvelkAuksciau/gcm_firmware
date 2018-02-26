#include <node.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <kmti/gimbal/MotorCommand.hpp>

#include <ch.hpp>
#include <pwmio.h>

binary_semaphore_t motor_data;

namespace Node {

  uavcan_stm32::CanInitHelper<> can;

  uavcan::Node<NodePoolSize>& getNode() {
    static uavcan::Node<NodePoolSize> node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
  }

  void uavcanNodeThread::main() {


    uavcan::uint32_t bitrate = 1000000;
    can.init(bitrate);

    getNode().setName("org.kmti.gmm_controler");
    getNode().setNodeID(10);

    if (getNode().start() < 0) {
      chSysHalt("UAVCAN init fail");
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

