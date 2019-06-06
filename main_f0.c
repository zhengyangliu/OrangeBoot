/***************************************************************************************************
*  TheaBoot                                                                                        *
*  Copyright (C) 2018 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                             *
*  Copyright (c) 2012-2014 PX4 Development Team.                                                   *
*                                                                                                  *
*  This file is part of TheaBoot project.                                                          *
*                                                                                                  *
*  @file     main_f1.c                                                                             *
*  @brief                                                                                          *
*  @author   Arthur Zheng                                                                          *
*  @email    15034186698@163.com                                                                   *
*  @version  0.2.0.0                                                                               *
*  @date     2018/07/15                                                                            *
*                                                                                                  *
*--------------------------------------------------------------------------------------------------*
*  Remark         :                                                                                *
*--------------------------------------------------------------------------------------------------*
*  Change History :                                                                                *
*  <Date>     | <Version> | <Author>       | <Description>                                         *
*--------------------------------------------------------------------------------------------------*
*  2019/06/06 | 0.1.0.0   | unknown        | Create file                                           *
*--------------------------------------------------------------------------------------------------*
* Lisense       : BSD 3-Clause                                                                     *
*                                                                                                  *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR   *
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND *
* FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR       *
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR              *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR         *
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     *
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE            *
* POSSIBILITY OF SUCH DAMAGE.                                                                      *
*--------------------------------------------------------------------------------------------------*
*                                                                                                  *
***************************************************************************************************/

/* Includes --------------------------------------------------------------------------------------*/
#include "hw_config.h"

#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/bkp.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/cm3/systick.h>

#include "bl.h"

/* Defination ------------------------------------------------------------------------------------*/
#define UDID_START      0x1FFFF7AC         /* STM32全球唯一ID地址 */

#ifdef INTERFACE_USART
# define BOARD_INTERFACE_CONFIG		(void *)BOARD_USART
#else
# define BOARD_INTERFACE_CONFIG		NULL
#endif

#define  BOARD_FLASH_SIZE             (BOARD_FLASH_PAGES * BOARD_FLASH_PAGE_SIZE)
#define  BOOTLOADER_RESERVATION_SIZE  (BOOTLOADER_PAGE * BOARD_FLASH_PAGE_SIZE)
#define  APP_SIZE_MAX                 (BOARD_FLASH_SIZE - BOOTLOADER_RESERVATION_SIZE)

#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	(void *)BOARD_USART
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif

/* Global variable  ------------------------------------------------------------------------------*/
struct boardinfo board_info =
{
	.id = DEVICE_ID,
    .rev = DEVICE_REV,
    .sn = DEVICE_SN,
    .fw_size = APP_SIZE_MAX,
    .flash_strc = DEVICE_FLASH_STRC,
    .device_des = DEVICE_DES,
	.systick_mhz = 48,
};

/** 
  * @brief  板载资源初始化
  * @param  none
  * @return none
  */
static void board_init(void)
{
    /* 初始化指示灯 */
#ifdef BOARD_PIN_LED_ACTIVITY
    rcc_periph_clock_enable(&BOARD_CLOCK_LED_ACTIVITY_REGISTER, BOARD_CLOCK_LED_ACTIVITY);
	gpio_mode_setup(BOARD_PORT_LED_ACTIVITY, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, BOARD_PIN_LED_ACTIVITY);
	BOARD_LED_ON(BOARD_PORT_LED_ACTIVITY, BOARD_PIN_LED_ACTIVITY);
#endif

#ifdef BOARD_PIN_LED_BOOTLOADER
    rcc_periph_clock_enable(&BOARD_CLOCK_LED_BOOTLOADER_REGISTER, BOARD_CLOCK_LED_BOOTLOADER);
	gpio_mode_setup(BOARD_PORT_LED_BOOTLOADER, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, BOARD_PIN_LED_BOOTLOADER);
	BOARD_LED_ON(BOARD_PORT_LED_BOOTLOADER, BOARD_PIN_LED_BOOTLOADER);
#endif

    /* 如果强制进入bootloader引脚被定义,初始化该引脚 */
#ifdef BOARD_FORCE_BL_PIN
	rcc_periph_clock_enable(BOARD_FORCE_BL_CLOCK);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN);
#endif

    /* 如果串口开启,初始化串口 */
#ifdef INTERFACE_USART
	rcc_periph_clock_enable(BOARD_USART_PIN_CLOCK);
	gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_AF, GPIO_PUPD_NONE, BOARD_PIN_TX | BOARD_PIN_RX);
	gpio_set_af(BOARD_PORT_USART, BOARD_PORT_USART_AF, BOARD_PIN_TX | BOARD_PIN_RX);
	rcc_periph_clock_enable(BOARD_USART_CLOCK);
#endif
}

/** 
  * @brief  板载资源反初始化
  * @param  none
  * @return none
  */
void board_deinit(void)
{
#ifdef BOARD_PIN_LED_ACTIVITY
    gpio_mode_setup(BOARD_PORT_LED_ACTIVITY, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BOARD_PIN_LED_ACTIVITY);
#endif

#ifdef BOARD_PIN_LED_BOOTLOADER
    gpio_mode_setup(BOARD_PORT_LED_BOOTLOADER, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BOARD_PIN_LED_BOOTLOADER);
#endif

#ifdef BOARD_FORCE_BL_PIN
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN);
#endif

#ifdef INTERFACE_USART
	gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_TX);
	rcc_periph_clock_disable(BOARD_USART_CLOCK);
#endif

	/* 复位APB2总线时钟 */
	RCC_APB2ENR = 0x00000000;
}

/** 
  * @brief  时钟初始化
  * @param  none
  * @return none
  */
static inline void clock_init(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();
}

/** 
  * @brief  强制升级引脚探测
  * @param  none
  * @return true,被触发.false,非触发
  */
static bool board_test_force_pin(void)
{
#if defined(BOARD_FORCE_BL_PIN)
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

    /* 对引脚进行200次采样，用以过滤杂波 */
	for (samples = 0; samples < 200; samples++)
	{
		if (gpio_get(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN) == BOARD_FORCE_BL_STATE)
	    {
			vote++;
		}
	}

	/* 如果采样触发在90%以上,视为有效 */
	if ((vote * 100) > (samples * 90))
	{
		return true;
	}

#endif
	return false;
}

#if INTERFACE_USART
/** 
  * @brief  测试串口是否收到打断boot信号
  * @param  none
  * @return true,被触发.false,非触发
  */
static bool board_test_usart_receiving_break(void)
{
	/* 启动SysTick定时器 */
	systick_interrupt_disable();      /* 关闭SysTick中断 */
	systick_counter_disable();        /* 关闭SysTick */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

	/**
	  * @note 设定定时器周期为波特率的一半
	  *       波特率 = 115200, 周期 = 8.68us
	  *       半周期 = 4.34us
	  *       timer_period = timer_tick / timer_reset_frequency = 168MHz / (1/4.34us) = 729.12 ~= 729
	  */
	systick_set_reload(729);    /* 4.3us 滴答周期 */
	systick_counter_enable();   /* 启动定时器 */

	uint8_t cnt_consecutive_low = 0;
	uint8_t cnt = 0;

	/** 
	  * @note 检测3个字节的传输时长计算高低电平数，采样频率使得可以每个位采集两次
      *       一个传输字节为10bits(8数据位 + 1起始位 + 1停止位)
      *       我们每半个位时间采样一次,每个字节采样20次
	  *       所以3个字节时长共60个采样
	  */
	while (cnt < 60)
	{
        /* 仅在SysTick置位时读取引脚电平 */
		if (systick_get_countflag() == 1)
		{
			if (gpio_get(BOARD_PORT_USART, BOARD_PIN_RX) == 0)
			{
				cnt_consecutive_low++;	   /* 增加计数器值 */

			}
			else
			{
				cnt_consecutive_low = 0;   /* 归零计数器 */
			}

			cnt++;
		}

		/* 如果收到9个位的低电平则退出循环 */
		if (cnt_consecutive_low >= 18)
		{
			break;
		}
	}

	systick_counter_disable(); /* 关闭SysTick */

	if (cnt_consecutive_low >= 18)
	{
		return true;
	}

	return false;
}
#endif

/** 
  * @brief  时钟反初始化
  * @param  none
  * @return none
  */
void clock_deinit(void)
{
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Reset the RCC_CFGR register */
	RCC_CFGR = 0x000000;

	/* Stop the HSE, CSS, PLL, PLLI2S, PLLSAI */
	rcc_osc_off(RCC_HSE);
	rcc_osc_off(RCC_PLL);
	rcc_css_disable();

	/* Reset the HSEBYP bit */
	rcc_osc_bypass_disable(RCC_HSE);

	/* Reset the CIR register */
	RCC_CIR = 0x000000;
}

/** 
  * @brief  读取flash sector大小
  * @param  sector, 块编号
  * @return 0,不存在;!0,sector大小
  * @note   此处将page当sector处理
  */
uint32_t flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_PAGES)
	{
		return BOARD_FLASH_PAGE_SIZE;
	}
	return 0;
}

/** 
  * @brief  擦除flash sector
  * @param  sector, 块编号
  * @return none
  */
void flash_func_erase_sector(unsigned sector)
{
	if(sector > BOARD_FLASH_PAGES || sector < BOOTLOADER_PAGE)
	{
		return;
	}

	/* 计算sector的实际物理地址
	 * flash_func_read_word 已经加上了 APP_LOAD_ADDRESS
	 */
	uint32_t address = 0;

	address = (sector - BOOTLOADER_PAGE) * BOARD_FLASH_PAGE_SIZE ;

	bool blank = true;

	for (unsigned i = 0; i < BOARD_FLASH_PAGE_SIZE; i += sizeof(uint32_t)) 
    {
		if (flash_func_read_word(address + i) != 0xffffffff) 
        {
			blank = false;
			break;
		}
	}

	if (!blank) 
	{
		flash_erase_page(APP_LOAD_ADDRESS + address);
	}
}

/** 
  * @brief  写入flash一个字
  * @param  address 写入地址
  * @param  word 写入数据
  * @return none
  */
void flash_func_write_word(uint32_t address, uint32_t word)
{
	flash_program_word(address + APP_LOAD_ADDRESS, word);
}

/** 
  * @brief  读取flash一个字数据
  * @param  address 地址
  * @return 读出的数据
  */
uint32_t flash_func_read_word(uint32_t address)
{
	return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

/** 
  * @brief  获取MCU全球唯一ID
  * @param  地址
  * @return 序列号值
  */
uint32_t flash_func_read_udid(uint32_t address)
{
	// read a byte out from unique chip ID area
	// it's 12 bytes, or 3 words.
	return *(uint32_t *)(address * 4 + UDID_START);
}

/** 
  * @brief  点亮LED
  * @param  led, led编号
  * @return none
  */
void led_on(unsigned led)
{
	switch (led)
		{
#ifdef BOARD_PIN_LED_ACTIVITY
		case LED_ACTIVITY:
			BOARD_LED_ON(BOARD_PORT_LED_ACTIVITY, BOARD_PIN_LED_ACTIVITY);
			break;
#endif

#ifdef BOARD_PIN_LED_BOOTLOADER
		case LED_BOOTLOADER:
			BOARD_LED_ON(BOARD_PORT_LED_BOOTLOADER, BOARD_PIN_LED_BOOTLOADER);
			break;
#endif
		}
}

/** 
  * @brief  关闭LED
  * @param  led, led编号
  * @return none
  */
void led_off(unsigned led)
{
	switch (led)
		{
#ifdef BOARD_PIN_LED_ACTIVITY
		case LED_ACTIVITY:
			BOARD_LED_OFF(BOARD_PORT_LED_ACTIVITY, BOARD_PIN_LED_ACTIVITY);
			break;
#endif

#ifdef BOARD_PIN_LED_BOOTLOADER
		case LED_BOOTLOADER:
			BOARD_LED_OFF(BOARD_PORT_LED_BOOTLOADER, BOARD_PIN_LED_BOOTLOADER);
			break;
#endif
		}
}

/** 
  * @brief  LED反向
  * @param  led, led编号
  * @return none
  */
void led_toggle(unsigned led)
{
	switch (led)
		{
#ifdef BOARD_PIN_LED_ACTIVITY
		case LED_ACTIVITY:
			gpio_toggle(BOARD_PORT_LED_ACTIVITY, BOARD_PIN_LED_ACTIVITY);
			break;
#endif

#ifdef BOARD_PIN_LED_BOOTLOADER
		case LED_BOOTLOADER:
			gpio_toggle(BOARD_PORT_LED_BOOTLOADER, BOARD_PIN_LED_BOOTLOADER);
			break;
#endif
		}
}

/** 
  * @brief  main函数
  * @param  none
  * @return none
  */
int main(void)
{
    /* 默认为启动 APP */
    uint8_t try_boot = 1;
    
    /* 如果该值非零则会在此时间后退出 bootloader */    
    unsigned timeout = BOOTLOADER_DELAY;  

	/* 板级初始化 */
	board_init();

	/* 配置时钟 */
	clock_init();

	/* 如果强制bootloader引脚被触发，则不进行对APP的引导 */
	if (board_test_force_pin())
	{
		try_boot = false;
	}

#if INTERFACE_USART
	/*
	 * 通过检测RX上的电平，如果电平持续为低，或接收到打断指令，则不进行boot引导
	 */
	if (board_test_usart_receiving_break())
	{
		try_boot = false;
	}
#endif

	/* 如果没有找到升级请求，则尝试直接启动APP */
    if (try_boot)
    {
        /* 直接尝试 boot */
        jump_to_app();

        timeout = 0;
    }

	/* 启动接口 */
#if INTERFACE_USART
	cinit(BOARD_INTERFACE_CONFIG_USART, USART);
#endif
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif

	while (1)
	{
		/* 允许bootloader，在升级完成或超时后返回 */
		bootloader(timeout);

        /* 如果强制bootloader引脚激活，直接重入循环 */
        if (board_test_force_pin())
        {
            continue;
        }

#if INTERFACE_USART
		/* 如果RX线仍在接收到打断，则重入循环 */
		if (board_test_usart_receiving_break())
		{
			continue;
	    }

#endif

		/* 尝试启动app */
		jump_to_app();

		/* app 启动失败，永久停留在bootloader */
		timeout = 0;
	}
}
