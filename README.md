# STM32-
STM32智能恒温热水壶
此项目只能用于个人学习，不可用于商业用途。
版权为个人所有，请遵守版权，违权必究。

/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2021年11月15日 18:44:52
  * @brief   平衡小车+FreeRTOS  参考-<野火>
  ******************************************************************************
  * @attention
  *
  * 实验平台:七彩光子 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
/*
*************************************************************************
*                             包含的头文件
*************************************************************************
*/ 
/* 系统标准头文件 */
#include <string.h>          //字符串操作相关函数
#include <stdio.h>           //标准输入输出
#include <limits.h>          //高等大数据定义
#include <stdlib.h>          //内存操作相关函数
#include <math.h>            //数据计算相关函数


/* FreeRTOS头文件 */
#include "FreeRTOS.h"         //FreeRTOS系统配置头文件
#include "task.h"             //创建任务头文件
#include "queue.h"            //内核队列头文件
#include "semphr.h"           //信号量头文件
#include "event_groups.h"     //事件处理头文件
#include "list.h"             //链表头文件


/* 开发板硬件bsp头文件 */
#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "bsp_key.h"
//#include "bsp_i2c_ee.h"  //软件I2C通讯底层
//#include "bsp_i2c_gpio.h"//软件I2C通讯底层IO口
//#include "bsp_i2c_ee.h"    //硬件I2C底层
#include "bsp_TiMbase.h"
#include "bsp_AdvanceTim.h"
#include "bsp_in_flash.h"   //读写内部Flash
#include "bsp_adc.h"        //高速ADC进行读取,为检测正弦波做准备!
#include "BUZZER.h"         //蜂鸣器
//#include "bsp_ili9341_lcd.h"//FSMC模拟8080时序ILI9341驱动文件
#include "ds18b20.h"        //DS18B20温度检测
#include "iic_old_oled.h"   //IIC_OLED驱动显示文件 
#include "lx_pid.h"         //PID计算头文件


/* FreeRTOS线程任务处理文件 */
#include "task_work.h"      //任务线程处理
#include "host_task.h"      //主机任务
#include "solid_task.h"     //从机任务

//#include "bsp_xpt2046_lcd.h"//触摸屏驱动
//#include "palette.h"        //画板显示应用程序
//#include "lcd_work.h"       //自己写的触摸屏逻辑处理函数
//#include "qq_bmp.h"        // 企鹅图片
//#include "beauty1.h"       // 女朋友图片



/**************************** 任务块 ********************************/
/*   仿照正点原子的按照任务的相关性来进行声明和定义所有的类型等变量
 */
 /* 创建任务函数句柄 */
TaskHandle_t AppTaskCreate_Handle=NULL;
 /* 创建任务函数堆栈大小 */
#define AppTask_STK_SIZE  512
 /* 创建任务函数任务优先级 */
#define AppTask_TASK_PRIO  1
 /* 创建任务函数声明 */
extern void AppTaskCreate(void);/* 用于创建任务 */

/**************************** 任务句柄 ********************************/
/* 
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */

/********************************** 内核对象句柄 *********************************/
/*
 * 信号量，消息队列，事件标志组，软件定时器这些都属于内核的对象，要想使用这些内核
 * 对象，必须先创建，创建成功之后会返回一个相应的句柄。实际上就是一个指针，后续我
 * 们就可以通过这个句柄操作这些内核对象。
 *
 * 内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，
 * 任务间的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数
 * 来完成的
 *  
 */

/******************************* 宏定义 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些宏定义。
 */

/******************************* 全局变量声明 ************************************/
/* 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */
  
/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void BSP_Init(void);/* 硬件初始化 */

/* 裸机驱动测试主函数 */
//int main(void)
//{
//	delay_init();	    	//延时函数初始化	  
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
//	uart_init(115200);	 	//串口初始化为115200

//	while(1)
//	{
//		delay_xms(1000);	
//	}
//}

/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化 
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{
	BaseType_t xReturn = pdTRUE; /* 定义一个创建信息返回值，默认为pdTRUE */
	
  /* 开发板硬件初始化 */
	BSP_Init();
	printf(" 野火FreeRTOS+ILI9341驱动屏幕测试!\r\n");
	
	/* 创建AppTaskCreate任务 */
	xReturn = xTaskCreate( (TaskFunction_t)  AppTaskCreate,         /* 任务入口函数 */
	                       (const char *  )  "AppTaskCreate",       /* 任务名称 */
                         (uint16_t      )  AppTask_STK_SIZE,      /* 任务栈大小 */
                         (void *        )  NULL,                  /* 任务入口函数参数 */
                         (UBaseType_t   )  AppTask_TASK_PRIO,     /* 任务优先级 */
                         (TaskHandle_t *)  &AppTaskCreate_Handle);/* 任务控制块指针(句柄) */

    if(xReturn == pdTRUE)/* 检测任务是否创建成功 */
	{	printf("任务AppTaskCreate创建成功,开启任务调度!\r\n"); }		
	else{return -1;}
	
	vTaskStartScheduler();/* 启动任务，开启调度 */
	
	while(1); /* 正常不会执行到这里 用于判断是否调度出错 */		
}

/***********************************************************************
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 硬件初始化
  * @ 参数    ： 无
  * @ 返回值  ： 无
  **********************************************************************/
void BSP_Init(void)
{ BaseType_t xReturn = pdTRUE; /* 定义一个创建信息返回值，默认为pdTRUE */
	/*
	 * STM32中断优先级分组为4，即4bit都用来表示抢占优先级，范围为：0~15
	 * 优先级分组只需要分组一次即可，以后如果有其他的任务需要用到中断，
	 * 都统一用这个优先级分组，千万不要再分组，切忌。
	 */	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
    /* 延时函数初始化 */
	delay_init();
	  /* 串口1初始化-->调试打印	*/
	uart_init(115200);/* 正点原子驱动 */
	  /* 串口2初始化-->提供给相关的上位机使用 */
	USART_Config();/* 野火驱动 */
    /* 基本定时器TIM7初始化 用于LED的定时控制 */
  BASIC_TIM7_Init();
    /* LED初始化 */
	LED_GPIO_Config();//LED与KEY冲突
    /* KEY按键初始化 */
	Key_GPIO_Config();
    /* 高速ADC值高速度读取初始化 */
  // ADCx_Init();
  //   /* 初始化FSMC驱动ILI9341TFT */
  // ILI9341_Init();
	//	//其中0、3、5、6 模式适合从左至右显示文字，
	//	//不推荐使用其它模式显示文字	其它模式显示文字会有镜像效果			
	//	//其中 6 模式为大部分液晶例程的默认显示方向  
	// ILI9341_GramScan ( 0 );
	// ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* 清屏，显示全黑 */
		/* 0.96寸OLED驱动初始化 */
	IIC_OLD_OLED_Init();//初始化OLED
	IIC_OLD_OLED_Clear();//清屏
	/* DS18B20配置初始化 */
	xReturn=DS18B20_Init();
	printf("DS18B20初始化:%d\r\n",(uint8_t)xReturn);
	/* PID控制针脚输出初始化配置 */
	PID_OutInit();
  PID_Init();
  /* 相关默认参数进行初始化配置 */
  Host_FansInit();
}

/********************************END OF FILE****************************/



