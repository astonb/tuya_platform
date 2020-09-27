#include "bsp_key.h"

//=============================================================================================//
typedef struct
{
    uint8_t KeyValue;
    uint16 PressTime;
    void (*KeyFun)(void);
} KEY_FUN;

static THREAD SysReboottHandle;
TIMER_ID KeyScanTimer;
static KEY_FUN KeyFuns[CALLFUN_MAX];

static uint8_t reboot_flag = 0;

void Product_WFSmart_2s(void);
//void Product_LedFlash_2s(void);

//=============================================================================================//
uint8_t Wifi_KeyValue_Get(void)
{
    return tuya_read_gpio_level(WF_CONFIG_SW1);
}

void Product_WFSmart_2s(void)
{
	PR_DEBUG("*****Smart_config*****");
	//tuya_dev_reset_select(NW_SMART_CFG);
	reboot_flag = 1;
}


//= Start ================================================================
bool KeyFun_Register(uint8_t KeyValue, uint16_t PressTime, void (*KeyFun)(void))
{
	int i = 0;
    PressTime /= 10;

    for ( i = 0; i < CALLFUN_MAX; i++)
    {
        if ((KeyFuns[i].KeyValue == KeyValue) && (KeyFuns[i].PressTime == PressTime))
        {
            KeyFuns[i].KeyFun = KeyFun;
            KeyFuns[i].PressTime = PressTime;
            return true;
        }
    }

    for (i = 0; i < CALLFUN_MAX; i++)
    {
        if (KeyFuns[i].KeyFun == NULL)
        {
            KeyFuns[i].KeyValue = KeyValue;
            KeyFuns[i].KeyFun = KeyFun;
            KeyFuns[i].PressTime = PressTime;
            return true;
        }
    }

    return false;
}


void Timer_TaskKEY(UINT timerID, PVOID pTimerArg)
{
    static uint16 PressTime = 0;
    static uint16 LastKey = -1;
	int i = 0;

    int key = Wifi_KeyValue_Get();
    if (LastKey == key)
    {   //ȥ�����
        if (key == Key_Press)
        {   //��Ч����
            PressTime++;

            for (i = 0; i < CALLFUN_MAX; i++)
            {
                if ((KeyFuns[i].KeyValue == Key_Press) && (KeyFuns[i].PressTime == PressTime))
                {
                    if(KeyFuns[i].KeyFun != NULL){
						KeyFuns[i].KeyFun();
						break;
                    }
                }
            }
        }else
        {   //û�м�����
			if(PressTime > 0)
			{
				for (i = 0; i < CALLFUN_MAX; i++)
				{
					if ((KeyFuns[i].KeyValue == Key_Release) && (KeyFuns[i].PressTime <= PressTime))
	                {
						if(KeyFuns[i].KeyFun != NULL){
							KeyFuns[i].KeyFun();
							break;
						}
	                }
				}
			}
			PressTime = 0;
        }
    }
    else
    {
        LastKey = key;
    }
}

void SysReboot_Poll_Task(PVOID pArg)
{
	while(1)
	{
		if(reboot_flag)
		{
			PR_DEBUG("***will_reboot***");
			reboot_flag = 0;
			tuya_dev_reset_select(NW_SMART_CFG);
		}
		SystemSleep(500);
	}
}

int Key_Init(void)
{
	OPERATE_RET op_ret;

	//��������ɨ��
	op_ret = sys_add_timer(Timer_TaskKEY, NULL, &KeyScanTimer);
    if(OPRT_OK != op_ret) {
		PR_ERR("add Timer_TaskKEY err");
    	return op_ret;
    }else {
    	sys_start_timer(KeyScanTimer, 10, TIMER_CYCLE);
    }

	//KeyFun_Register(Key_Press,	 2000,	Product_LedFlash_2s);	//����ledָʾ
	KeyFun_Register(Key_Release, 2000, 	Product_WFSmart_2s);	//����(�ɿ�)
	
	CreateAndStart(&SysReboottHandle, SysReboot_Poll_Task, NULL, 0X400, TRD_PRIO_2, "SysReboot_Poll");

	return OPRT_OK;
}

