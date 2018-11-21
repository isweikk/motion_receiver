#ifndef __UT_LOG_H__
#define __UT_LOG_H__

#include "eos_type.h"
//#include "utils/zlog/zlog.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define LOG_CFG_PATH "/tmp/log.conf"

#define LOG_FATAL  0
#define LOG_ERROR  1
#define LOG_WARN   2
#define LOG_INFO   3
#define LOG_DBG    4
#define LOG_TRACE  5

#define LOG_LEVEL LOG_DBG

#if 0
#define LogFatal(...) dzlog_fatal(__VA_ARGS__)
#define LogError(...) dzlog_error(__VA_ARGS__) 
#define LogWarn(...)  dzlog_warn(__VA_ARGS__)
#define LogNotice(...) dzlog_notice(__VA_ARGS__)
#define LogInfo(...)  dzlog_info(__VA_ARGS__)
#define LogDbg(...)  dzlog_debug(__VA_ARGS__)
#define LogTrace()   dzlog_debug("\033[1;31;40m$@$@$@$\033[0m\n")

#else
#define LogError(fmt, arg...)  printf("[ERROR ] [%15.15s :%04d] "fmt"", __FILE__, __LINE__, ##arg)
#define LogWarn(fmt, arg...)   printf("[WARN  ] [%15.15s :%04d] "fmt"", __FILE__, __LINE__, ##arg)
#define LogNotice(fmt, arg...) printf("[NOTICE] [%15.15s :%04d] "fmt"", __FILE__, __LINE__, ##arg)
#define LogInfo(fmt, arg...)   printf("[INFO  ] [%15.15s :%04d] "fmt"", __FILE__, __LINE__, ##arg)
#define LogDbg(fmt, arg...)    printf("[DEBUG ] [%15.15s :%04d] "fmt"", __FILE__, __LINE__, ##arg)
#define LogTrace() printf("\033[1;31;40m[TRACE ] [%15.15s : %04d] $@$@$@$\033[0m\n", __FILE__, __LINE__)

#endif
int UtLogInit(char* outpath);

int UtLog(int level, ...);

void UtLogFini(void);


#if defined(__cplusplus)
}
#endif 
#endif
