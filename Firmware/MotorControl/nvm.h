/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NVML_H
#define __NVM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

int NVM_init(void);
int NVM_erase(void);
int NVM_read_tail(uint64_t *data, size_t *length);
int NVM_append(uint64_t *data, size_t length);
void NVM_demo(void);

#ifdef __cplusplus
}
#endif

#endif //__NVM_H