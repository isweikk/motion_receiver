
#ifndef __MOTION_MOTOR_CTRL_H__
#define __MOTION_MOTOR_CTRL_H__

#include "eos_type.h"
#include "pthread.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MC_DBG_LOG  0

#define MOT_USART_NAME "/dev/ttyS1"

#define MOT_OBJ_LIST_MAX     0x77
#define MOT_CMD_LEN         13
#define MOT_CMD_RXBUF_MAX   (13*64)

#define MOT_USART_BAUD      115200

#define MOT_NUM_TOTAL       8

enum EnumMCJointAck{
    EmJointAckError = 0,
    EmJointAckOk,
    EmJointAckArrived
};
    
enum EnumMCJoint{
    EmJointFootLine = 0,
    EmJointFootRotate,
    EmJointHead,
    EmJointArmLeft,
    EmJointArmRight,
    EmJointWristLeft,
    EmJointWristRight,
    EmJointHandRotate,
    EmJointHand,
    EmJointEnd,
    EmJointAll
};

enum EnumMotCfgItem{
    EmMotCfgPmin = 0,
    EmMotCfgPmax,
    EmMotCfgOffset
};

#define MC_SPEED_MAX 5
#define FOOT_SPEED_MAX 20
#define AUTO_CTRL_WAIT 2000

//M1=Head, M2=Arm, M3=Wrist, M5=Hand
//#define MOT_M1_LIMIT_MID    1370     //  18/du
//#define MOT_M2_LIMIT_MID    1517    //   18/du
//#define MOT_M3_LIMIT_MID    1705    //  19/du
//#define MOT_M4_LIMIT_MID    1000
//#define MOT_M5_LIMIT_MID    300    //open=288,close=1000
//#define MOT_M6_LIMIT_MID    1000
//
#define MOTOR_HEAD          (4)
#define MOTOR_ARM_L         (3)
#define MOTOR_ARM_R         (0)
#define MOTOR_WRIST_L       (2)
#define MOTOR_WRIST_R       (0)
#define MOTOR_HAND_ROTATE   (0)
#define MOTOR_HAND          (1)
#define MOTOR_FOOT_L        (0)
#define MOTOR_FOOT_R        (0)
#define MOTOR_FOOT_LINE     (8)
#define MOTOR_FOOT_ROTATE   (9)
#define MOTOR_ALL           (100)


#define MOTOR_AUTO_CTRL     (0)
#define MOTOR_FREE_CTRL     (1)


//motion parameter
typedef struct {
    unsigned char  valid;       //need to send.0=none, 1=valid
    unsigned char  rxvalid;    //if return ok, set 1. check over,set 0
    unsigned char  type;        //01-0x36
    unsigned short value1;     //send value
    unsigned short value2;     //
    unsigned short value3;     //
    unsigned short rxval1;     //return value
    unsigned short rxval2;     //
    unsigned short rxval3;     //
}StMotorMsg;

typedef struct {
    int  lock;
    char txbuf[MOT_CMD_LEN];
    char rxbuf[MOT_CMD_RXBUF_MAX];
    int  rxlen;       //13/cmd
    int  rxreadlen;    
    unsigned int timeout;    //timeout, ms,100
}StMotorCmdHdl;


typedef struct {
    int run_en;
    int uart_fd;
    int mode;   //0=auto , 1=normal.
    int stopflag;   //stop force when waiting for being arrived

    StMotorCmdHdl   cmd_hdl;
    StMotorMsg*  obj_list;
}StMotionHandle;


int MCSendMotorCmdEx(char* cmd_out);
//extern StMotionHandle kMotHdl;

void MCSetMode(int mode);
int MCGetMode(void);
int MCSendObjCmd(unsigned char object, unsigned short val1, unsigned short val2, unsigned short val3);
int MCSendObjCmdWithAck(unsigned char object, unsigned short val1, unsigned short val2, unsigned short val3);
int MCReadObjWithValRet(unsigned char object, unsigned short *val1, unsigned short *val2, unsigned short *val3);
int MCSendMotorCmd(int mode, int motor, unsigned short val1, unsigned short val2, unsigned short val3);
int MCReadObjectAck(unsigned char object, unsigned short* val1, unsigned short* val2, unsigned short* val3);
int MCWaitMotorArrived(int motor, unsigned int time_ms);
int MCClearMotorCmd(int motor);
int MCStopBlockingMotor(void);

//free control mode, need check timeout of the stop cmd.
void MCRefreshUnctrlTime(int motor);
void MCClearUnctrlTime(int motor);

void* MCCheckUnctrlThread(void* prm);
void* MCRecvUartMsgThread(void* param);
void MCModuleEnable(void);
void MCModuleDisable(void);
int MCGetModuleStatus(void);

int MotionCtrlStartup(void);
void MotionCtrlStop(void);
int MotionGetCtrlStat(void);


#ifdef __cplusplus
}
#endif

#endif
