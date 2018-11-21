
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/stat.h>
#include <fcntl.h> 
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/mman.h>
#include <linux/serial.h>

#include "ut_usart.h"
#include "ut_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_PRINT_UART_DATA 0


static int Ut_InUsartOpen(char* ttySn);
static void Ut_InUsartClose(int fd);
static int Ut_InUsartInit(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity);
static int Ut_InUsartSetBaud(int fd, int baud);

struct __UsartManage
{
    unsigned int num;
    char   name[16];
    int  fd;
    unsigned int lock;
};

static struct __UsartManage UsartMng[5];


//time delay
void UtDelayMs(unsigned int msval)
{
    usleep(msval*1000);
}

unsigned int UtGetRunTimeMs(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

int Ut_InUsartCheck(char* name)
{
    int i;
    for(i=0; i<5; i++){
        if(strcmp(UsartMng[i].name, name) == 0){
            if(UsartMng[i].lock == 1)
                return EOS_TRUE;
        }
    }
    return EOS_FALSE;
}

int Ut_InUsartLock(char* name)
{
    int i;
    for(i=0; i<5; i++){
        if(UsartMng[i].name[0] == '\0'){
            strcpy(UsartMng[i].name, name);
            UsartMng[i].lock = 1;
            return EOS_OK;
        }
    }
    return EOS_ERROR;
}

int Ut_InUsartUnlock(char* name)
{
    int i;
    for(i=0; i<5; i++){
        if(strcmp(UsartMng[i].name, name) == 0){
            strcpy(UsartMng[i].name, "");
            UsartMng[i].lock = 0;
            return EOS_OK;
        }
    }
    return EOS_ERROR;
}

int Ut_UsartDefaultInit(char* devName)
{
    int devfd = -1;

    if((devfd = Ut_InUsartOpen(devName)) < 0)
        return EOS_ERROR;
    if(Ut_InUsartInit(devfd, 115200, 0, 8, 1, 'N') < 0)
        return EOS_ERROR;
    return devfd;
}

int Ut_UsartInit(char* devName, unsigned int baud)
{
    int devfd = -1;

    if(baud != 9600 && baud != 57600 && baud != 115200){
        LogError("[UTL] not support band %d\n", baud);
        return EOS_ERROR;
    }
    if((devfd = Ut_InUsartOpen(devName)) < 0)
        return EOS_ERROR;
    if(Ut_InUsartInit(devfd, baud, 0, 8, 1, 'N') < 0){
        Ut_InUsartClose(devfd);
        return EOS_ERROR;
    }
    return devfd;
}

int Ut_UsartSpecialInit(char* devName, unsigned int baud)
{
    int devfd = -1;

    if(baud < 9600 || baud > 115200){
        LogError("[UTL] not support band %d (9600-115200)\n", baud);
        return EOS_ERROR;
    }
    if((devfd = Ut_InUsartOpen(devName)) < 0)
        return EOS_ERROR;
    if(Ut_InUsartInit(devfd, 9600, 0, 8, 1, 'N') < 0){
        Ut_InUsartClose(devfd);
        return EOS_ERROR;
    }
    if(Ut_InUsartSetBaud(devfd, baud) < 0){
        Ut_InUsartClose(devfd);
        return EOS_ERROR;
    }
    return devfd;
}

int Ut_UsartExit(int devfd)
{
    Ut_InUsartClose(devfd);
    return EOS_OK;
}

/*read the data from uart
 *pData  :the point variable for saving the data
 *length  :the size of the pData.
 */
int Ut_UsartRead(int devfd, char* pData, unsigned int size)
{
    int length = 0;

    if(devfd <= 0){
        return 0;
    }
    if(size > 255){
        size = 255;
    }
    ioctl(devfd, FIONREAD, &length);
    if(length > 0){
        length = read(devfd, pData, size);
#if DEBUG_PRINT_UART_DATA
        LogDbg("[UTL] RXD(devfd=%d)(%d)\n", devfd, length);
#endif        
    }
    
    return length;
}


/*send data from uart
 *fd:the device file describe
 *length:the length of "pData"
 */
int Ut_UsartWrite(int devfd, char* pData, unsigned int length)
{
    int ret = 0;

    if(devfd <= 0)
        return EOS_ERROR;

    if((ret = write(devfd, pData, length)) < 0){
        extern int errno;
        if(errno != 0){
            LogError("[UTL] TXD:(devfd=%d)(pre tx=%d)(err:%s)\r\n", devfd, length, strerror(errno)); 
        }
        tcflush(devfd,TCOFLUSH);
        return EOS_ERROR;
    }else{
#if DEBUG_PRINT_UART_DATA
        LogDbg("[UTL] TXD:(devfd=%d)(pre tx=%d; true tx=%d)\r\n", devfd, length, ret);
#endif        
        return ret;
    }    
}

/*********************Local Function***********************************************/

static int Ut_InUsartOpen(char* ttySn)
{
    int fd = -1;
	
	fd = open( ttySn, O_RDWR|O_NOCTTY|O_NDELAY);
	if(fd < 0){
		LogError("[UTL] Cann't Open %s\r\n", ttySn);
		return EOS_ERROR;
	}

	//fcntl(fd, F_SETFL, FNDELAY);  //阻塞
    fcntl(fd, F_SETFL, 0);      //非阻塞
	
	if(0 == isatty(STDIN_FILENO)){
		LogError("[UTL] Standard input is not a terminal device.\n");
		//return EOS_ERROR;
	}
	return fd;
}

static void Ut_InUsartClose(int fd)
{
    close(fd);
}

static int Ut_InUsartInit(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity)
{
	unsigned int   i;
	int   speed_arr[] = { B115200, B57600, B19200, B9600};
	int   name_arr[] = {115200, 57600, 19200, 9600};
         
	struct termios options;
   
	/*tcgetattr(fd,&options),this function gets the paramer of the uart,and saves int the options,
	if it runs successfully,returns 0,or else returns 1*/
	if(tcgetattr(fd, &options)  !=  0){
		LogError("[UTL] Getting the paramer of the uart!\n");    
		return EOS_ERROR; 
	}
	//set the baud rate
	for (i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++){
		if  (speed == name_arr[i]){             
			cfsetispeed(&options, speed_arr[i]); 
			cfsetospeed(&options, speed_arr[i]);  
		}
	}     
   
	//modify the mode.set the serial port free.
	options.c_cflag |= CLOCAL;
	//modify the mode, open the function for reading.
	options.c_cflag |= CREAD;
	
	//set data stream mode
	switch(flow_ctrl){
    	case 0 ://none
    	      options.c_cflag &= ~CRTSCTS;
    	      break;   
    	case 1 ://hardware data stream mode
    	      options.c_cflag |= CRTSCTS;
    	      break;
    	case 2 ://software data stream mode
    	      options.c_cflag |= IXON | IXOFF | IXANY;
    	      break;
	}
	//set the bit of the data.
	options.c_cflag &= ~CSIZE;
	switch (databits){  
    	case 5    :
    		options.c_cflag |= CS5;
    		break;
    	case 6    :
    		options.c_cflag |= CS6;
    		break;
    	case 7    :    
    		options.c_cflag |= CS7;
    		break;
    	case 8:    
    		options.c_cflag |= CS8;
    		break;  
    	default:   
    		LogError("[UTL] Unsupported data size.\n");
    		return EOS_ERROR; 
	}
	//set the verify bit.
	switch (parity){  
    	case 'n':
    	case 'N': //none
    		options.c_cflag &= ~PARENB; 
    		options.c_iflag &= ~INPCK;    
    		break; 
    	case 'o':  
    	case 'O'://odd parity   
    		options.c_cflag |= (PARODD | PARENB); 
    		options.c_iflag |= INPCK;             
    		break; 
    	case 'e': 
    	case 'E'://even parity
    		options.c_cflag |= PARENB;       
    		options.c_cflag &= ~PARODD;       
    		options.c_iflag |= INPCK;      
    		break;
    	case 's':
    	case 'S': //set the space
    		options.c_cflag &= ~PARENB;
    		options.c_cflag &= ~CSTOPB;
    		break; 
    	default:  
    		LogError("[UTL] Unsupported parity.\n");    
    		return EOS_ERROR; 
	} 
	//set stop bit
	switch (stopbits){  
    	case 1:   
    		options.c_cflag &= ~CSTOPB;
    		break; 
    	case 2:   
    		options.c_cflag |= CSTOPB;
    		break;
    	default:   
    		LogError("[UTL] Unsupported stop bits.\n"); 
    		return EOS_ERROR;
	}
   
	//modify the mode to output. original data.
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	//options.c_lflag &= ~(ISIG | ICANON);	
	//other char for control.
	options.c_iflag &= ~(IXON | IXOFF | IXANY);	//
	options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
	options.c_oflag &= ~(ONLCR | OCRNL);
   
	//set the waiting time and the least size for reading and writing.
	options.c_cc[VTIME] = 1; /* wait 1*(1/100)s */  
	options.c_cc[VMIN] = 1;
   
	//if the data is overflowing,clean the old data.
	tcflush(fd, TCIFLUSH);
   
	//activate the option. 
	if (tcsetattr(fd,TCSANOW,&options) != 0){
		//perror("wrluartInit: com set error!");  
		return EOS_ERROR; 
	}

	LogDbg("[UTL] Usart (fd=%d) set param ok!\n", fd);
	return EOS_OK;; 
}

/*set the special baud rate.
 */
static int Ut_InUsartSetBaud(int fd, int baud)
{
    int   status;
    struct termios   Opt;
    struct serial_struct Serial;

    tcgetattr(fd, &Opt);        /*Get current options*/
    tcflush(fd, TCIOFLUSH);/*Flush the buffer*/     
    cfsetispeed(&Opt, B38400);/*Set input speed,38400 is necessary? who can tell me why?*/  
    cfsetospeed(&Opt, 38400); /*Set output speed*/
    tcflush(fd,TCIOFLUSH);        /*Flush the buffer*/
    status = tcsetattr(fd, TCSANOW, &Opt);  /*Set the 38400 Options*/
    if  (status != 0){        
        LogError("[UTL] tcsetattr fd\n");  
        return EOS_ERROR;     
    }   
                    
    if((ioctl(fd,TIOCGSERIAL,&Serial))<0){ /*Get configurations vim IOCTL*/
        LogError("[UTL] Fail to get Serial!\n");
        return EOS_ERROR;
    }
    Serial.flags = ASYNC_SPD_CUST;/*We will use custom buad,May be standard,may be not */
    Serial.custom_divisor=Serial.baud_base/baud;/*In Sep4020,baud_base=sysclk/16*/

    if((ioctl(fd,TIOCSSERIAL,&Serial))<0){ /*Set it*/
        LogError("[UTL] Fail to set Serial\n");
        return EOS_ERROR;
    }
	
    ioctl(fd,TIOCGSERIAL,&Serial);/*Get it again,not necessary.*/
    LogInfo("[UTL] Usart(fd=%d)(baud=%d, custom_divisor=%d, baud_base=%d)OK.\n", 
        fd, baud, Serial.custom_divisor, Serial.baud_base);
    //tcflush(fd,TCIOFLUSH);
    return EOS_OK;
}



#ifdef __cplusplus
}
#endif


