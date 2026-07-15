#include "control.h"
#include "getdata.h"
#include "ankle_can_config.h"

struct Motors motor[6];

void motor_stop_init(void)
{
    /* CAN 总线上现在只使用踝关节电机 ID=1，
     * 不再向 0/2/3 等不存在的 CAN 节点发送停机命令。
     */
    motor_init_stop(ANKLE_CAN_NODE_ID);
}

void motor_init(uint8_t node_id, uint8_t mode)
{
    if (mode == 1U) {
        Filtering_location(node_id);
    }
    else if (mode == 2U) {
        Direct_Speed(node_id);
    }
    else if (mode == 3U) {
        Direct_torque(node_id);
    }
}

void Calibrate_motor_init(void)
{
    /* 只记录 CAN ID=1 的当前位置作为零点。 */
    if (motor_fb_valid[ANKLE_CAN_NODE_ID] != 0U) {
        motor[ANKLE_CAN_NODE_ID].init_pos = motor[ANKLE_CAN_NODE_ID].pos;
    }
}
