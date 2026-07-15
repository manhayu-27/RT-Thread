#ifndef __RIS_PROTOCOL_H
#define __RIS_PROTOCOL_H

#include <stdint.h>

#pragma pack(push, 1)

typedef struct
{
    uint8_t id     : 4;
    uint8_t status : 3;
    uint8_t none   : 1;
} RIS_Mode_t;

typedef struct
{
    int16_t tor_des;     /* q8  */
    int16_t spd_des;     /* q8  */
    int32_t pos_des;     /* q15 */
    int16_t k_pos;       /* q15 */
    int16_t k_spd;       /* q15 */
} RIS_Comd_t;

typedef struct
{
    int16_t  torque;     /* q8  */
    int16_t  speed;      /* q8  */
    int32_t  pos;        /* q15 */
    int8_t   temp;
    uint16_t MError : 3;
    uint16_t force  : 12;
    uint16_t none   : 1;
} RIS_Fbk_t;

#pragma pack(pop)

#endif
