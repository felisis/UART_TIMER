#ifndef _DEBUG_H_
#define _DEBUG_H_

///////#define DEBUG_MESSAGE_PRINT

void PRINTF_DEBUG(char* fmt, ...);

#if defined(DEBUG_MESSAGE_PRINT)
#define PRINTF_D    PRINTF_DEBUG
#else
#define PRINTF_D(...)
#endif

#endif /* _DEBUG_H_ */
