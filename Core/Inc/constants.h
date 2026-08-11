#ifndef PROTO_TURRET_CONSTANTS_H
#define PROTO_TURRET_CONSTANTS_H

#define PID_NODE_NAME "proto_turret_node"
#define PID_TOPIC_CMD "/proto_turret_cmd"
#define PID_TOPIC_STATUS "/proto_turret_stm32_publisher"
#define PID_TOPIC_AS5600 "/proto_turret_as5600"  // сырой угол AS5600 (0..4095, -1 = нет ответа)
#define PID_QOS_DEPTH 10

#endif
