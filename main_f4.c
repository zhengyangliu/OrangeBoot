/***************************************************************************************************
*  OrangeBoot                                                                                      *
*  Copyright (C) 2018 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                             *
*  Copyright (c) 2012-2014 PX4 Development Team.                                                   *
*                                                                                                  *
*  This file is part of OrangeBoot project.                                                        *
*                                                                                                  *
*  @file     main_f4.c                                                                             *
*  @brief                                                                                          *
*  @author   Arthur Zheng                                                                          *
*  @email    15034186698@163.com                                                                   *
*  @version  0.1.0.0                                                                               *
*  @date     2018/08/08                                                                            *
*                                                                                                  *
*--------------------------------------------------------------------------------------------------*
*  Remark         :                                                                                *
*--------------------------------------------------------------------------------------------------*
*  Change History :                                                                                *
*  <Date>     | <Version> | <Author>       | <Description>                                         *
*--------------------------------------------------------------------------------------------------*
*  2018/08/01 | 0.1.0.0   | Arthur Zheng   | Creat file                                            *
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
#define UDID_START      0x1FFF7A10         /* STM32全球唯一ID地址 */

#ifdef INTERFACE_USART
# define BOARD_INTERFACE_CONFIG		(void *)BOARD_USART
#else
# define BOARD_INTERFACE_CONFIG		NULL
#endif

#define APP_SIZE_MAX                 (BOARD_FLASH_SIZE - BOOTLOADER_RESERVATION_SIZE)

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
	.systick_mhz = 168,
};

/* Private variable  -----------------------------------------------------------------------------*/
/* Flash结构 */
static struct
 {
	uint32_t	sector_number;
	uint32_t	size;
} flash_sectors[] = 
{
    {0x00, 16 * 1024},
	{0x01, 16 * 1024},
	{0x02, 16 * 1024},
	{0x03, 16 * 1024},
	{0x04, 64 * 1024},
	{0x05, 128 * 1024},
	{0x06, 128 * 1024},
	{0x07, 128 * 1024},
	{0x08, 128 * 1024},
	{0x09, 128 * 1024},
	{0x0a, 128 * 1024},
	{0x0b, 128 * 1024},
	/* 2M型号的第二个blank结构 */
	{0x10, 16 * 1024},
	{0x11, 16 * 1024},
	{0x12, 16 * 1024},
	{0x13, 16 * 1024},
	{0x14, 64 * 1024},
	{0x15, 128 * 1024},
	{0x16, 128 * 1024},
	{0x17, 128 * 1024},
	{0x18, 128 * 1024},
	{0x19, 128 * 1024},
	{0x1a, 128 * 1024},
	{0x1b, 128 * 1024},
};

/* F4标准时钟定义 */
static const struct rcc_clock_scale clock_setup = {
	.pllm = OSC_FREQ,
	.plln = 336,
	.pllp = 2,
	.pllq = 7,
#if defined(STM32F446) || defined(STM32F469)
	.pllr = 2,
#endif
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_4,
	.ppre2 = RCC_CFGR_PPRE_DIV_2,
	.power_save = 0,
	.flash_config = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS,
	.apb1_frequency = 42000000,
	.apb2_frequency = 84000000,
};

/** 
  * @brief  板载资源初始化
  * @param  none
  * @return none
  */
static void board_init(void)
{
    /* 初始化指示灯 */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, BOARD_CLOCK_LED_ACTIVITY);
    gpio_mode_setup(BOARD_PORT_LED_ACTIVITY, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_PIN_LED_ACTIVITY);
	gpio_set_output_options(BOARD_PORT_LED_ACTIVITY, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, BOARD_PIN_LED_ACTIVITY);
    BOARD_LED_ON(BOARD_PORT_LED_ACTIVITY, BOARD_PIN_LED_ACTIVITY);

    rcc_peripheral_enable_clock(&RCC_AHB1ENR, BOARD_CLOCK_LED_BOOTLOADER);
    gpio_mode_setup(BOARD_PORT_LED_BOOTLOADER, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_PIN_LED_BOOTLOADER);
    gpio_set_output_options(BOARD_PORT_LED_BOOTLOADER, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, BOARD_PIN_LED_BOOTLOADER);
	BOARD_LED_ON(BOARD_PORT_LED_BOOTLOADER, BOARD_PIN_LED_BOOTLOADER);

    /* 如果强制进入bootloader引脚被定义,初始化该引脚 */
#ifdef BOARD_FORCE_BL_PIN
	rcc_peripheral_enable_clock(&BOARD_FORCE_BL_CLOCK_REGISTER, BOARD_FORCE_BL_CLOCK_BIT);
	gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, BOARD_FORCE_BL_PULL, BOARD_FORCE_BL_PIN);
#endif

    /* 如果串口开启,初始化串口 */
#ifdef INTERFACE_USART
	rcc_peripheral_enable_clock(&BOARD_USART_PIN_CLOCK_REGISTER, BOARD_USART_PIN_CLOCK_BIT);
	gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BOARD_PIN_TX | BOARD_PIN_RX);
	gpio_set_af(BOARD_PORT_USART, BOARD_PORT_USART_AF, BOARD_PIN_TX);
	gpio_set_af(BOARD_PORT_USART, BOARD_PORT_USART_AF, BOARD_PIN_RX);
	rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

}

/** 
  * @brief  板载资源反初始化
  * @param  none
  * @return none
  */
void board_deinit(void)
{
    gpio_mode_setup(BOARD_PORT_LED_ACTIVITY, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_LED_ACTIVITY);
    gpio_mode_setup(BOARD_PORT_LED_BOOTLOADER, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_LED_BOOTLOADER);

#ifdef BOARD_FORCE_BL_PIN
    gpio_mode_setup(BOARD_FORCE_BL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_FORCE_BL_PIN);
	gpio_clear(BOARD_FORCE_BL_PORT, BOARD_FORCE_BL_PIN);
#endif

#ifdef INTERFACE_USART
    gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_TX | BOARD_PIN_RX);
	rcc_peripheral_disable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
#endif

    /* 关闭AHB外设时钟 */
	RCC_AHB1ENR = 0x00100000; // XXX Magic reset number from STM32F4x reference manual
}

/** 
  * @brief  时钟初始化
  * @param  none
  * @return none
  */
static inline void clock_init(void)
{
	rcc_clock_setup_hse_3v3(&clock_setup);
}

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

	/* Reset the RCC_PLLCFGR register */
	RCC_PLLCFGR = 0x24003010; // XXX Magic reset number from STM32F4xx reference manual

	/* Reset the HSEBYP bit */
	rcc_osc_bypass_disable(RCC_HSE);

	/* Reset the CIR register */
	RCC_CIR = 0x000000;
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
  * @brief  读取flash sector大小
  * @param  sector, 块编号
  * @return 0,不存在;!0,sector大小
  */
uint32_t flash_func_sector_size(unsigned sector)
{
	if (sector < BOARD_FLASH_SECTORS) 
    {
		return flash_sectors[sector].size;
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
	if (sector >= BOARD_FLASH_SECTORS || sector < BOOTLOADER_SECTOR)
    {
		return;
	}

	/* 计算sector的实际物理地址
	 * flash_func_read_word 已经加上了 APP_LOAD_ADDRESS
	 */
	uint32_t address = 0;

	for (unsigned i = BOOTLOADER_SECTOR; i < sector; i++) 
    {
		address += flash_func_sector_size(i);
	}

	/* 查空sector */
	unsigned size = flash_func_sector_size(sector);
	bool blank = true;

	for (unsigned i = 0; i < size; i += sizeof(uint32_t)) 
    {
		if (flash_func_read_word(address + i) != 0xffffffff) 
        {
			blank = false;
			break;
		}
	}

	/* 如果查空失败，则擦除该sector */
	if (!blank) 
	{
		flash_erase_sector(flash_sectors[sector].sector_number, FLASH_CR_PROGRAM_X32);
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
	return *(uint32_t *)(address + UDID_START);
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
		case LED_ACTIVITY:
			BOARD_LED_ON(BOARD_PORT_LED_ACTIVITY, BOARD_PIN_LED_ACTIVITY);
			break;

		case LED_BOOTLOADER:
			BOARD_LED_ON(BOARD_PORT_LED_BOOTLOADER, BOARD_PIN_LED_BOOTLOADER);
			break;
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
		case LED_ACTIVITY:
			BOARD_LED_OFF(BOARD_PORT_LED_ACTIVITY, BOARD_PIN_LED_ACTIVITY);
			break;

		case LED_BOOTLOADER:
			BOARD_LED_OFF(BOARD_PORT_LED_BOOTLOADER, BOARD_PIN_LED_BOOTLOADER);
			break;
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
		case LED_ACTIVITY:
			gpio_toggle(BOARD_PORT_LED_ACTIVITY, BOARD_PIN_LED_ACTIVITY);
			break;

		case LED_BOOTLOADER:
			gpio_toggle(BOARD_PORT_LED_BOOTLOADER, BOARD_PIN_LED_BOOTLOADER);
			break;
		}
}

#ifndef SCB_CPACR
# define SCB_CPACR (*((volatile uint32_t *) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

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

    /* Enable the FPU before we hit any FP instructions */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

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

/********* Copyright (C) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD  *****END OF FILE****/
