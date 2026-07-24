#ifndef CAR_ROLE_H
#define CAR_ROLE_H

/*
 * 两辆车使用同一套工程，只需在烧录前切换这里的角色：
 *   CAR_ROLE_MASTER：主车，PA9接串口屏，并通过蓝牙通知从车。
 *   CAR_ROLE_SLAVE ：从车，只等待蓝牙圈数命令。
 */
#define CAR_ROLE_MASTER    (1U)
#define CAR_ROLE_SLAVE     (2U)

#ifndef CAR_ROLE
#define CAR_ROLE           CAR_ROLE_MASTER
#endif

#endif
