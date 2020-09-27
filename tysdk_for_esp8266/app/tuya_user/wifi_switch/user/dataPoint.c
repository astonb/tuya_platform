#include "dataPoint.h"
#include "bsp_key.h"

SEM_HANDLE Upload_SemHandle;
THRD_HANDLE Upload_TaskHandle;
MUTEX_HANDLE Upload_mutexHandle;

TIMER_ID delayoffTimer;
switch_device  switch_dev;

/***********************************************************
*  Function: msg_upload_proc
*  Description: dp上报函数
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
INT msg_upload_proc(void)
{
	OPERATE_RET err;

	if(tuya_get_cloud_stat() == FALSE)
		return OPRT_DP_REPORT_CLOUD_ERR;

	MutexLock(Upload_mutexHandle);
    GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();
    if(STAT_UNPROVISION == wf_stat || \
       STAT_STA_UNCONN == wf_stat || \
       (tuya_get_gw_status() != STAT_WORK)) {
		err = OPRT_WF_CONN_FAIL;
		goto exit;
    }


	cJSON *root = cJSON_CreateObject();
    if(NULL == root) {
		err = OPRT_BUF_NOT_ENOUGH;
        goto exit;
    }

	//根据实际情况上报设备数据点
	cJSON_AddBoolToObject(root, DP_control, switch_dev.statu);
	cJSON_AddNumberToObject(root, DP_delayoff, switch_dev.delayoff);
	

    CHAR *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if(NULL == out) {
        PR_ERR("cJSON_PrintUnformatted err:");
		err = OPRT_CJSON_PARSE_ERR;
        goto exit;
    }
    
	PR_DEBUG("out[%s]", out);
    OPERATE_RET op_ret = tuya_obj_dp_trans_report(out);
    Free(out);
    
    if( OPRT_OK == op_ret ) {
		err = OPRT_OK;
        goto exit;
    }else {
		PR_DEBUG("msg upload error");
		err = OPRT_DP_REPORT_CLOUD_ERR;
       	goto exit;
    }

exit:

	MutexUnLock(Upload_mutexHandle);

	return err;
}
/***********************************************************
*  Function: Motor_ControlByApp
*  Description: 接受app控制处理函数
*  Input: root命令
*  Output: 
*  Return: 
***********************************************************/
void device_ControlByApp(cJSON *root)
{	
	cJSON *cmd = cJSON_GetObjectItem(root, DP_control);
	if(cmd != NULL){
		switch_dev.statu = (bool)cmd->valueint;	
		tuya_write_gpio_level(WF_SWITCH_GPIO_PIN, switch_dev.statu);
	}

	cmd = cJSON_GetObjectItem(root, DP_delayoff);
	if(cmd != NULL){
		switch_dev.delayoff = cmd->valueint;
	}

	PostSemaphore(Upload_SemHandle);
}

void MSG_Upload_Poll_Task(PVOID pArg)
{	
	OPERATE_RET ret;
	
	Upload_SemHandle = CreateSemaphore();
	if(Upload_SemHandle == NULL)
	{
		PR_ERR("Create Upload_Sem failed");	
		ThrdJoin(Upload_TaskHandle, NULL);
		return;
	}
	InitSemaphore(Upload_SemHandle, 0, 1);
	
	ret = CreateMutexAndInit(&Upload_mutexHandle);
	if(ret != OPRT_OK)
	{
		PR_ERR("Create Upload_Mutex failed");	
		ThrdJoin(Upload_TaskHandle, NULL);
		return;
	}
	
	while(1)
	{
		if(WaitSemaphore(Upload_SemHandle) == OPRT_OK)
		{
			ret = msg_upload_proc();
			if(ret != OPRT_OK)
				PR_ERR("***msg_upload failed!***");
		}
		
	}
}


void Timer_delayoff(UINT timerID, PVOID pTimerArg)
{
	static unsigned int shundown_cnt = 0;

	if(switch_dev.statu)
		shundown_cnt++;
	else
		shundown_cnt = 0;

	if(shundown_cnt >=  switch_dev.delayoff)
		tuya_write_gpio_level(WF_SWITCH_GPIO_PIN, FALSE);
}

void Product_Init(void)
{
	memset(&switch_dev, 0x0, sizeof(switch_dev));

	//设置GPIO
	PIN_FUNC_SELECT(WF_SWITCH_GPIO_MUX, WF_SWITCH_GPIO_FUNC);
	GPIO_DIS_OUTPUT(WF_SWITCH_GPIO_PIN);//设置成输入模式
	
	Key_Init();

	OPERATE_RET op_ret;
	op_ret = sys_add_timer(Timer_delayoff, NULL, &delayoffTimer);
    if(OPRT_OK != op_ret) {
		PR_ERR("add Timer_TaskKEY err");
    	//return op_ret;
    }else{
		sys_start_timer(delayoffTimer, 1000, TIMER_CYCLE);
	}
    	
}


