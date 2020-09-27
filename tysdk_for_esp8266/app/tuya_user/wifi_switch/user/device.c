/***********************************************************
*  File: device.c 
*  Author: HZB
*  Date: 20181206
***********************************************************/
#define __DEVICE_GLOBALS
#include "tuya_smart_api.h"
#include "wf_sdk_adpt.h"

#include "device.h"
#include "datapoint.h"


#define SYN_TIME  2  
#define WM_FAIL 1
#define WM_SUCCESS 0


TIMER_ID Syntimer;

STATIC VOID wfl_timer_cb(UINT timerID, PVOID pTimerArg)
{
	STATIC UINT last_wf_stat = 0xffffffff;
	GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();

	//The system is about to restart without monitoring！

	if (last_wf_stat != wf_stat)
	{
		PR_DEBUG("wf_stat:%d",wf_stat);
		switch (wf_stat)
		{
			case STAT_UNPROVISION:
				PR_DEBUG("STAT_UNPROVISION");
				break;
			case STAT_AP_STA_UNCONN:
			case STAT_AP_STA_CONN:
				PR_DEBUG("STAT_AP_STA_UNCONN");
				break;
			case STAT_STA_CONN:
				PR_DEBUG("STAT_STA_CONN");
				break;
			default:
				break;
		}
		last_wf_stat = wf_stat;
	}
}



STATIC VOID syn_active_data(VOID)
{
	PR_DEBUG("sys active data");
	if(tuya_get_cloud_stat())
		PostSemaphore(Upload_SemHandle);
	else
		PR_DEBUG("cloud unconnect !!!");
}


VOID device_cb(SMART_CMD_E cmd,cJSON *root)
{
	CHAR *buf = cJSON_PrintUnformatted(root);
	if (NULL == buf)
	{
		PR_ERR("malloc error");
		return;
	}

	PR_DEBUG("root cmd:%s",buf);

	//实际控制设备
	device_ControlByApp(root);
	
	Free(buf);
}

STATIC VOID syn_timer_cb(UINT timerID,PVOID pTimerArg)
{
	PR_DEBUG("syn timer cb ...");

	if( FALSE == tuya_get_cloud_stat() ) {
        return;
    }

	PostSemaphore(Upload_SemHandle);
 	sys_stop_timer(Syntimer);
}

STATIC OPERATE_RET device_differ_init(VOID)
{
	OPERATE_RET op_ret;
	TIMER_ID timer;

	CreateAndStart(&Upload_TaskHandle, MSG_Upload_Poll_Task, NULL, 2048, TRD_PRIO_2, "msg_upload_task");
	
	op_ret = sys_add_timer(wfl_timer_cb, NULL, &timer);
	if (OPRT_OK != op_ret)
	{
		return op_ret;
	}
	else
	{
		sys_start_timer(timer, 300, TIMER_CYCLE);
	}
	
	return OPRT_OK;
}

/***********************************************************
*  Function: device_init
*  Description:
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
OPERATE_RET device_init(VOID)
{
	OPERATE_RET op_ret;
	
	PR_NOTICE("fireware info name:%s version:%s", APP_BIN_NAME, USER_SW_VER);

	//Product_Init();
	op_ret = tuya_device_init(PRODECT_KEY, device_cb, USER_SW_VER);
	if (op_ret != OPRT_OK)
	{
		return op_ret;
	}
	
	tuya_active_reg(syn_active_data);

	op_ret = sys_add_timer(syn_timer_cb, NULL, &Syntimer);
    if(OPRT_OK != op_ret) 
	{
		PR_ERR("add syn_timer err");
    	return op_ret;
    }
	else 
	{
    	sys_start_timer(Syntimer, SYN_TIME*1000, TIMER_CYCLE);
    }

	Product_Init();
	
	op_ret = device_differ_init();
	if (op_ret != OPRT_OK)
	{
		return op_ret;
	}

	return op_ret;
}


/***********************************************************
*  Function: app_init
*  Description: 
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
void pre_app_init(void)
{
}

VOID app_init(VOID)
{
	app_cfg_set(WCM_OLD, NULL);
}

//设置固件标识名和版本回调函数，用于工厂生产固件校验
VOID set_firmware_tp(IN OUT CHAR *firm_name, IN OUT CHAR *firm_ver)
{
	strcpy(firm_name, APP_BIN_NAME);
	strcpy(firm_ver, USER_SW_VER);
	return;
}

//GPIO 测试回调函数
BOOL gpio_func_test(VOID)
{
	return TRUE;
}


