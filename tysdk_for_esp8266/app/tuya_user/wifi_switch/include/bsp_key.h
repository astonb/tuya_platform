#ifndef _DOOYA_KEY_H_
#define _DOOYA_KEY_H_

#include "tuya_smart_api.h"
#include "wf_sdk_adpt.h"

#define CALLFUN_MAX 	4
#define SmartConfig		2000


//===== Smart Wifi Config Key=========================//
#define WF_CONFIG_SW1			GPIO_ID_PIN(1)		//配网按键
#define WF_CONFIG_SW1_MUX		GPIO_PIN_REG_1
#define WF_CONFIG_SW1_FUNC		FUNC_GPIO1

//===================================================//
#define KVALUE_NONE     0x00
#define KVALUE_SET      0x01

typedef enum{

	Key_Press = 0,
	Key_Release,
	
}KEY_STATUS;
	

typedef struct {

	uint8_t Exec_WithFree;	
	uint16_t time;
    void (*fun)(void);
  
}KEY_CTRL;

uint8_t KeyValue_Get(void);

int Key_Init(void);


#endif

