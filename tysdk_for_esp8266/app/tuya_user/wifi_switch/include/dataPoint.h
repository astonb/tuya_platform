#ifndef _DATAPOINT_H__
#define _DATAPOINT_H__

#include "tuya_smart_api.h"
#include "wf_sdk_adpt.h"

#define WF_SWITCH_GPIO_PIN		GPIO_ID_PIN(5)
#define WF_SWITCH_GPIO_MUX		GPIO_PIN_REG_5
#define WF_SWITCH_GPIO_FUNC	FUNC_GPIO5


#define MAX_MSG_UPLOAD_SEM_SIZE	(10)

#define DP_control		"1"			
#define DP_delayoff		"101"

typedef struct{

	bool statu;	//状态， 0关， 1开
	int delayoff;//延迟关闭，单位：秒
	
}switch_device;

extern SEM_HANDLE Upload_SemHandle;
extern THRD_HANDLE Upload_TaskHandle;

INT msg_upload_proc(void);
void Motor_ControlByApp(cJSON *root);
void MSG_Upload_Poll_Task(PVOID pArg);


void Product_Init(void);

#endif
