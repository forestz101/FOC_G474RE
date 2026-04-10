#include "retarget.h"
#include "usart.h"  // For huart4 handle
#include <sys/stat.h>
#include <errno.h>

// Retarget _write to UART4 for printf output
extern UART_HandleTypeDef huart2;

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
