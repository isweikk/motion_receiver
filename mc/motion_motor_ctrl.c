
#include "motion_motor_ctrl.h"

#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>  
#include <setjmp.h>  
#include <math.h>  
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <pthread.h>

#include "eos_type.h"
#include "websocket_chat.h"
#include "ut_usart.h"

#ifdef __cplusplus
extern "C" {
#endif

StMotionHandle kMotHdl;
unsigned int kMtUnctrlTimeRec[10];
//timeout = ms,when we have not recieve motor command for 500 ms,we need to send stop cmd to the MCS.
#define MC_UNCTRL_TIMEOUT  500



/*************************************************************************************************/
/*pack and unpack
| head       | type   | value1   | value2    | value3  | end     |
| 4 bytes    | 1 byte | 2 bytes  | 2 bytes   | 2 bytes | 2 bytes |
| 0xAAAAAAAA | 1-n    |          |           |         | 0xBBBB  |
*/

void MCCmdPack(StMotorMsg* cmd_info, char* cmd_out)
{
    cmd_out[0] = 0xAA;
    cmd_out[1] = 0xAA;
    cmd_out[2] = 0xAA;
    cmd_out[3] = 0xAA;
    cmd_out[4] = cmd_info->type;
    cmd_out[5] = cmd_info->value1 >> 8;
    cmd_out[6] = cmd_info->value1 & 0xff;
    cmd_out[7] = cmd_info->value2 >> 8;
    cmd_out[8] = cmd_info->value2 & 0xff;
    cmd_out[9] = cmd_info->value3 >> 8;
    cmd_out[10] = cmd_info->value3 & 0xff;
    cmd_out[11] = 0xBB;
    cmd_out[12] = 0XBB;
    cmd_info->rxvalid = 0;
}

int MCCmdUnpack(char* cmd_in)
{
    if((cmd_in[0] != 0xAA) || (cmd_in[1] != 0xAA) || (cmd_in[2] != 0xAA) || (cmd_in[3] != 0xAA)){
        return EOS_ERROR;
    }
    if(cmd_in[4] >= MOT_OBJ_LIST_MAX){
        return EOS_ERROR;
    }
    //don't care the ack data in freely-control-mode.
    if (cmd_in[4] < 0x10) {
        return EOS_OK;
    }
    send_msg_to_client(cmd_in);
    
    int obj_num = cmd_in[4];
    StMotorMsg* p_cmd_info = &kMotHdl.obj_list[obj_num];

    p_cmd_info->type = obj_num;
    p_cmd_info->rxval1 = cmd_in[5];
    p_cmd_info->rxval1 <<= 8;
    p_cmd_info->rxval1 |= cmd_in[6];

    p_cmd_info->rxval2 = cmd_in[7];
    p_cmd_info->rxval2 <<= 8;
    p_cmd_info->rxval2 |= cmd_in[8];

    p_cmd_info->rxval3 = cmd_in[9];
    p_cmd_info->rxval3 <<= 8;
    p_cmd_info->rxval3 |= cmd_in[10];
    
    p_cmd_info->rxvalid = 1;

    #if MC_DBG_LOG
    LogInfo("[MC] Motor recv cmd: %02x, %04x, %04x, %04x\n", p_cmd_info->type, p_cmd_info->rxval1, 
        p_cmd_info->rxval2, p_cmd_info->rxval3);
    #endif
    return EOS_OK;
}

void MCSetMode(int mode)
{
    kMotHdl.mode = mode;
}

int MCGetMode(void)
{
    return kMotHdl.mode;
}

//
int MCSendObjCmd(unsigned char object, unsigned short val1, unsigned short val2, unsigned short val3)
{
    if(object < MOT_OBJ_LIST_MAX){
        StMotorMsg* p_cmd_info =  &kMotHdl.obj_list[object];
        p_cmd_info->type   = object;
        p_cmd_info->value1 = val1;
        p_cmd_info->value2 = val2;
        p_cmd_info->value3 = val3;
        p_cmd_info->rxvalid = 0;
        p_cmd_info->valid = 1;
        MCCmdPack(p_cmd_info, kMotHdl.cmd_hdl.txbuf);
        if(Ut_UsartWrite(kMotHdl.uart_fd, kMotHdl.cmd_hdl.txbuf, MOT_CMD_LEN) < 0){
            LogError("[MC] Send obj faild!\n");
            return EOS_ERROR;
        }else{
            #if MC_DBG_LOG
            LogInfo("[MC] Send obj: type-%02x, value1-%04x, value2-%04x, value3-%04x\n", 
                p_cmd_info->type, p_cmd_info->value1, p_cmd_info->value2, p_cmd_info->value3);
            #endif
        }
    }
    return EOS_OK;
}

int MCSendObjCmdWithAck(unsigned char object, unsigned short val1, unsigned short val2, unsigned short val3)
{
    if(object < MOT_OBJ_LIST_MAX){
        StMotorMsg* p_cmd_info =  &kMotHdl.obj_list[object];
        p_cmd_info->type   = object;
        p_cmd_info->value1 = val1;
        p_cmd_info->value2 = val2;
        p_cmd_info->value3 = val3;
        p_cmd_info->rxvalid = 0;
        MCCmdPack(p_cmd_info, kMotHdl.cmd_hdl.txbuf);
        if(Ut_UsartWrite(kMotHdl.uart_fd, kMotHdl.cmd_hdl.txbuf, MOT_CMD_LEN) < 0){
            LogError("[MC] Send obj faild!\n");
            return EmJointAckError;
        }else{
            LogDbg("[MC] Send obj: type-%02x, value1-%04x, value2-%04x, value3-%04x\n", 
                p_cmd_info->type, p_cmd_info->value1, p_cmd_info->value2, p_cmd_info->value3);
        }
        unsigned int timeout = 0;
        while(p_cmd_info->rxvalid == 0){
            UtDelayMs(10);
            timeout += 10;
            if(timeout > 100){
                return EmJointAckError;
            }
        }
        p_cmd_info->rxvalid = 0;
        if ((p_cmd_info->rxval1 == EmJointAckOk) 
            || (p_cmd_info->rxval1 == EmJointAckArrived)){
            return EmJointAckOk;
        }else{
            return EmJointAckError;
        }
    }
    return EmJointAckError;
}

int MCReadObjWithValRet(unsigned char object, unsigned short *val1, unsigned short *val2, unsigned short *val3)
{
    if(object < MOT_OBJ_LIST_MAX){
        StMotorMsg* p_cmd_info =  &kMotHdl.obj_list[object];
        p_cmd_info->type   = object;
        p_cmd_info->value1 = 0;
        p_cmd_info->value2 = 0;
        p_cmd_info->value3 = 0;
        p_cmd_info->rxvalid = 0;
        MCCmdPack(p_cmd_info, kMotHdl.cmd_hdl.txbuf);
        if(Ut_UsartWrite(kMotHdl.uart_fd, kMotHdl.cmd_hdl.txbuf, MOT_CMD_LEN) < 0){
            LogError("[MC] Send obj faild!\n");
            return EOS_ERROR;
        }else{
            LogDbg("[MC] Send obj: type-%02x, value1-%04x, value2-%04x, value3-%04x\n", 
                p_cmd_info->type, p_cmd_info->value1, p_cmd_info->value2, p_cmd_info->value3);
        }
        unsigned int timeout = 0;
        while(p_cmd_info->rxvalid == 0){
            UtDelayMs(10);
            timeout += 10;
            if(timeout > 200){
                return EOS_ERROR;
            }
        }
        if(val1) *val1 = p_cmd_info->rxval1;
        if(val2) *val2 = p_cmd_info->rxval2;
        if(val3) *val3 = p_cmd_info->rxval3;
        p_cmd_info->rxvalid = 0;
        LogDbg("[MC] Read obj: type-%02x, v1-%04x, v2-%04x, v3-%04x\n", 
                p_cmd_info->type, p_cmd_info->rxval1, p_cmd_info->rxval2, p_cmd_info->rxval3);
        return EOS_OK;
    }
    return EOS_ERROR;
}

//

int MCSendMotorCmd(int mode, int motor, unsigned short val1, unsigned short val2, unsigned short val3)
{
    int ret = EOS_ERROR;
    unsigned char object = 0;
    
    if(kMotHdl.cmd_hdl.lock == 0){
        kMotHdl.cmd_hdl.lock = 1;
        if((mode == MOTOR_AUTO_CTRL) || (mode == MOTOR_FREE_CTRL)){
            kMotHdl.mode = mode;
        }else{
            kMotHdl.cmd_hdl.lock = 0;
            return ret;
        }
        if(kMotHdl.mode == MOTOR_AUTO_CTRL){
            object = (motor & 0xff) + 0x40;
        }else{
            object = motor & 0xff;
        }
        if((motor >= 1) && (motor <= 9)){
            ret = MCSendObjCmd(object, val1, val2, val3);
        }   
        kMotHdl.cmd_hdl.lock = 0;
    }
    return ret;
}


int MCReadObjectAck(unsigned char object, unsigned short* val1, unsigned short* val2, unsigned short* val3)
{
    StMotorMsg* p_cmd_info = NULL;

    if(object < MOT_OBJ_LIST_MAX){
        p_cmd_info =  &kMotHdl.obj_list[object];
        if(p_cmd_info->rxvalid  > 0){
            if(val1) *val1 = p_cmd_info->rxval1;
            if(val2) *val2 = p_cmd_info->rxval2;
            if(val3) *val3 = p_cmd_info->rxval3;
            p_cmd_info->rxvalid = 0;
            return EOS_OK;
        }
    }
    return EOS_ERROR;
}

int MCWaitMotorArrived(int motor, unsigned int time_ms)
{
    int  ret = EOS_ERROR;
    unsigned int  timeout = 0;
    unsigned char  object = 0, start_obj = 0, end_obj = 0;
    unsigned short val1;
    StMotorMsg* p_cmd_info = NULL;
    
    if(motor == MOTOR_ALL){
        start_obj = 0x41;
        end_obj = 0x49;
        object = start_obj;
    }else if((motor >= 1) && (motor <= 9)){
        object = motor + 0x40;
        start_obj = end_obj = object;
    }else{
        return EOS_ERROR;
    }
    kMotHdl.stopflag = 0;
    while(1){
        for(; object <= end_obj; object++){
            p_cmd_info = &kMotHdl.obj_list[object];
            if (p_cmd_info->valid) {
                if (MCReadObjectAck(object, &val1, NULL, NULL) == EOS_ERROR) {
                    ret = EOS_ERROR;
                    break;
                } else {
                    ret = val1; //if one motor isnot arrived, break,check again after another 10 ms.
                    if (ret != EmJointAckArrived) {
                        break;
                    } else {
                        p_cmd_info->valid = 0;
                    }
                }
            } else {
                ret = EmJointAckArrived;
            }
        }
        if(ret == EmJointAckArrived){
            break;
        }
        if (time_ms == 0)
            break;
        if(kMotHdl.stopflag || timeout >= time_ms){
            MCClearMotorCmd(motor);  //timeout, clear the "valid"
            break;
        }else{
            UtDelayMs(10);
            timeout += 10;
        }
    }
    kMotHdl.stopflag = 0;
    return ret;
}

//motor = 0;clear all motor.
int MCClearMotorCmd(int motor)
{
    StMotorMsg* cmd = NULL;
    unsigned char  object = 0;

    if(motor == MOTOR_ALL){
        for(motor = 1; motor <= 9; motor++){
            object = motor + 0x40;
            cmd = kMotHdl.obj_list + object;
            cmd->valid = 0;
            cmd->rxvalid = 0;
        }
    }else if((motor >= 1) && (motor <= 9)){
        object = motor + 0x40;
        cmd = kMotHdl.obj_list + object;
        cmd->valid = 0;
        cmd->rxvalid = 0;
    }
    return EOS_OK;
}

int MCStopBlockingMotor(void)
{
    kMotHdl.stopflag = 1;
    return 0;
}

void MCRefreshUnctrlTime(int motor)
{
    if((motor > 0) && (motor < 10))
        kMtUnctrlTimeRec[motor] = UtGetRunTimeMs();
}

void MCClearUnctrlTime(int motor)
{
    if((motor > 0) && (motor < 10))
        kMtUnctrlTimeRec[motor] = 0;
}

void* MCCheckUnctrlThread(void* prm)
{
    int motor = 1;
    unsigned int cur_time = 0;

    while(1){
        if(kMotHdl.run_en == 0)
            break;
        
        cur_time = UtGetRunTimeMs();
        for(motor =1; motor <= 9; motor++){
            if(kMtUnctrlTimeRec[motor] == 0){
                continue;
            }else if((cur_time - kMtUnctrlTimeRec[motor]) > MC_UNCTRL_TIMEOUT){
                kMtUnctrlTimeRec[motor] = 0;
                MCSendMotorCmd(MOTOR_FREE_CTRL, motor, 0, 0, 0);
            }
        }
        UtDelayMs(50);
    }
    LogInfo( "[MC] MCCheckUnctrlThread finish!\n");
    return 0;
}


void* MCRecvUartMsgThread(void* param)
{
    StMotorCmdHdl* prm = &kMotHdl.cmd_hdl;
    int tmp_len = 0;
    int tmp_valid_len = 0;
    char tmp[MOT_CMD_LEN];
    int tailbyte = 0;
    pthread_t  check_thread_id;

    prm->lock = 0;
    prm->rxlen = 0;
    prm->rxreadlen = 0;
    
    if ((kMotHdl.uart_fd = Ut_UsartSpecialInit(MOT_USART_NAME, MOT_USART_BAUD)) < 0) {
        goto RecvThreadEnd;
    }
    
    if ((kMotHdl.obj_list = (StMotorMsg*)malloc(sizeof(StMotorMsg) * MOT_OBJ_LIST_MAX)) == NULL) {
        LogError( "[MC] MotionCtrlStartup malloc faild!\n");
        goto RecvThreadEnd;
    }
    memset(kMotHdl.obj_list, 0, sizeof(StMotorMsg) * MOT_OBJ_LIST_MAX);
    kMotHdl.run_en = 1;
    pthread_create(&check_thread_id, NULL, MCCheckUnctrlThread, NULL);
    pthread_detach(check_thread_id);
    LogInfo("[MC] MCRecvUartMsgThread Init OK!\n");
    while(1)
    {
        if(kMotHdl.run_en == 0)
            break;
        
        if((tmp_len = Ut_UsartRead(kMotHdl.uart_fd, &prm->rxbuf[prm->rxlen], 
            MOT_CMD_RXBUF_MAX - prm->rxlen)) > 0){
            //LogDbg("[MC] ++++ now recv_len=%d, total=%d.rxlen=%d.\n", tmp_len, prm->rxlen, prm->rxreadlen);
            prm->rxlen += tmp_len;
            if(prm->rxlen >= prm->rxreadlen) {
                tmp_valid_len = prm->rxlen - prm->rxreadlen;
            } else {
                tmp_valid_len = MOT_CMD_RXBUF_MAX + prm->rxlen - prm->rxreadlen;
            }
            while (tmp_valid_len >= MOT_CMD_LEN){
                //when we check cmd,if head is not right,discard the first byte.
                if(prm->rxlen >= prm->rxreadlen) {
                    if(MCCmdUnpack(&prm->rxbuf[prm->rxreadlen]) < 0){
                        prm->rxreadlen++;
                        tmp_valid_len--;
                    }else{
                        prm->rxreadlen += MOT_CMD_LEN;
                        tmp_valid_len -= MOT_CMD_LEN;
                    }
                }else if(prm->rxlen < prm->rxreadlen) {
                    if ((MOT_CMD_RXBUF_MAX - prm->rxreadlen) >= MOT_CMD_LEN){
                        if(MCCmdUnpack(&prm->rxbuf[prm->rxreadlen]) < 0){
                            prm->rxreadlen++;
                            tmp_valid_len--;
                        }else{
                            prm->rxreadlen += MOT_CMD_LEN;
                            tmp_valid_len -= MOT_CMD_LEN;
                        }
                    }else {
                        tailbyte = MOT_CMD_RXBUF_MAX - prm->rxreadlen;
                        memcpy(tmp, &prm->rxbuf[prm->rxreadlen], tailbyte);
                        memcpy(&tmp[tailbyte], prm->rxbuf, MOT_CMD_LEN - tailbyte);
                        if(MCCmdUnpack(tmp) < 0){
                            prm->rxreadlen++;
                            tmp_valid_len--;
                        }else{
                            prm->rxreadlen = MOT_CMD_LEN - tailbyte;
                            tmp_valid_len -= MOT_CMD_LEN;
                        }
                    }
                }
                if(prm->rxreadlen >= MOT_CMD_RXBUF_MAX) {
                    prm->rxreadlen = 0;
                }
            }
            
            if(prm->rxlen >= MOT_CMD_RXBUF_MAX) {
                prm->rxlen = 0;
            }
        }
        UtDelayMs(10);
    }

    RecvThreadEnd:
    if(kMotHdl.uart_fd)
        Ut_UsartExit(kMotHdl.uart_fd);
    if(kMotHdl.obj_list != NULL){
        free(kMotHdl.obj_list);
        kMotHdl.obj_list = NULL;
    }
    kMotHdl.run_en = 0;
    LogInfo( "[MC] MCRecvUartMsgThread finish!\n");
    return EOS_OK;
}

void MCModuleEnable(void)
{
    kMotHdl.run_en = 1;
}

void MCModuleDisable(void)
{
    kMotHdl.run_en = 0;
}

int MCGetModuleStatus(void)
{
    return kMotHdl.run_en;
}





int MotionCtrlStartup(void)
{
    pthread_t recv_msg_thread_id;
    pthread_create(&recv_msg_thread_id, NULL, MCRecvUartMsgThread, NULL);
    pthread_detach(recv_msg_thread_id);
    while(MCGetModuleStatus() == 0){
        UtDelayMs(10);
    }
    return EOS_OK;
}

void MotionCtrlStop(void)
{
    MCModuleDisable();
}

int MotionGetCtrlStat(void)
{
    return MCGetModuleStatus();
}

int MCSendMotorCmdEx(char* cmd_out)
{
    memcpy(kMotHdl.cmd_hdl.txbuf, cmd_out, 13);
    if(Ut_UsartWrite(kMotHdl.uart_fd, kMotHdl.cmd_hdl.txbuf, MOT_CMD_LEN) < 0){
        LogError("[MC] Send obj faild!\n");
        return EOS_ERROR;
    }else{
        LogDbg("[MC] Send obj: type-%02x, value1-%04x, value2-%04x, value3-%04x\n", 
            cmd_out[4], cmd_out[5]*255+cmd_out[6], cmd_out[7]*255+cmd_out[8], cmd_out[9]*255+cmd_out[10]);
    }
    return EOS_OK;
}

#ifdef __cplusplus
}
#endif

