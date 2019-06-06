/***************************************************************************************************
*  TheaBoot                                                                                        *
*  Copyright (C) 2018 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                             *
*  Copyright (c) 2012-2014 PX4 Development Team.                                                   *
*                                                                                                  *
*  This file is part of TheaBoot project.                                                          *
*                                                                                                  *
*  @file     bl.c                                                                                  *
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
*  2018/04/19 | 0.1.0.0   | Arthur Zheng   | Create file                                           *
*  2018/07/15 | 0.2.0.0   | Arthur Zheng   | Restyle the structure of project                      *
*  2018/08/06 | 0.2.0.1   | Arthur Zheng   | 取消last_input机制，以修复两端口同时开启时错误输入数据会*
*                                            导致的硬件错误                                         *
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

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/syscfg.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>

#include "bl.h"
#include "cdcacm.h"
#include "usart.h"

/* Defination ------------------------------------------------------------------------------------*/
/** 
  * @breif 基础参数 
  **/
#define BL_PROTOCOL_VERSION 		"0.1.1.0"     	/*!< 当前协议版本 */

#define PROTO_INSYNC				0xA5            /*!< Magic code of INSYNC */
#define PROTO_EOC					0xF7            /*!< Magic code of EOC */
#define PROTO_PROG_MULTI_MAX        256	            /*!< 最大单次烧写数据长度,单位:byte */
#define PROTO_REPLY_MAX             256	            /*!< 最大返回数据长度,单位:byte */

/** 
  * @breif 返回状态 
  **/
#define PROTO_OK					0x10            /*!< 操作成功 */
#define PROTO_FAILED				0x11            /*!< 操作失败 */
#define PROTO_INVALID				0x13	        /*!< 指令无效 */

/** 
  * @breif 操作指令 
  **/
#define PROTO_GET_SYNC				0x21            /*!< 测试同步 */

#define PROTO_GET_UDID				0x31            /*!< 读取芯片指定地址上的 UDID 12字节的值 */
#define PROTO_GET_FW_SIZE           0x32            /*!< 获取固件区大小 */

#define PROTO_GET_BL_REV            0x41            /*!< 获取Bootloader版本 */
#define PROTO_GET_ID                0x42            /*!< 获取电路板型号,包含版本 */
#define PROTO_GET_SN                0x43            /*!< 获取电路板序列号 */
#define PROTO_GET_REV               0x44            /*!< 获取电路板版本 */
#define PROTO_GET_FLASH_STRC        0x45            /*!< 获取FLASH结构描述 */
#define PROTO_GET_DES               0x46            /*!< 获取以 ASCII 格式读取设备描述 */

#define PROTO_CHIP_ERASE			0x51            /*!< 擦除设备 Flash 并复位编程指针 */
#define PROTO_PROG_MULTI			0x52            /*!< 在当前编程指针位置写入指定字节的数据，并使编程指针向后移动到下一段的位置 */
#define PROTO_GET_CRC				0x53	        /*!< 计算并返回CRC校验值 */
#define PROTO_BOOT					0x54            /*!< 引导 APP 程序 */

/* Private variable  -----------------------------------------------------------------------------*/
static uint8_t bl_type;
static const uint8_t bl_proto_rev[] = BL_PROTOCOL_VERSION; /*!<  */ // value returned by PROTO_DEVICE_BL_REV
volatile unsigned timer[NTIMERS];
static enum led_state {LED_BLINK, LED_ON, LED_OFF} _led_state;  /*<! 记录LED状态 */
static unsigned head, tail;
static uint8_t rx_buf[256];

/** 
  * @brief  数据接口初始化
  * @param  none
  * @return none
  */
inline void cinit(void *config, uint8_t interface)
{
#if INTERFACE_USB
    if (interface == USB)
    {
        return usb_cinit();
    }
#endif
#if INTERFACE_USART
    if (interface == USART)
    {
        return uart_cinit(config);
    }
#endif
}

/** 
  * @brief  数据接口反初始化
  * @param  none
  * @return none
  */
inline void cfini(void)
{
#if INTERFACE_USB
    usb_cfini();
#endif
#if INTERFACE_USART
    uart_cfini();
#endif
}

/** 
  * @brief  数据接收
  * @param  none
  * @return -1,失败.其他,接收到的数据
  */
inline int cin(void)
{
#if INTERFACE_USB

    if (bl_type == NONE || bl_type == USB)
    {
        int usb_in = usb_cin();

        if (usb_in >= 0)
        {
            bl_type = USB;
            return usb_in;
        }
    }

#endif

#if INTERFACE_USART

    if (bl_type == NONE || bl_type == USART)
    {
        int uart_in = uart_cin();

        if (uart_in >= 0)
        {
            bl_type = USART;
            return uart_in;
        }
    }

#endif

    return -1;
}

/** 
  * @brief  发送数据
  * @param  buf,数据指针
  *         len,数据长度
  * @return none
  */
inline void cout(uint8_t *buf, unsigned len)
{
#if INTERFACE_USB

    if (bl_type == NONE || bl_type == USB)
    {
        usb_cout(buf, len);
    }

#endif
#if INTERFACE_USART

    if (bl_type == NONE || bl_type == USART)
    {
        uart_cout(buf, len);
    }

#endif
}

void buf_put(uint8_t b)
{
	unsigned next = (head + 1) % sizeof(rx_buf);

	if (next != tail) {
		rx_buf[head] = b;
		head = next;
	}
}

int buf_get(void)
{
	int	ret = -1;

	if (tail != head) {
		ret = rx_buf[tail];
		tail = (tail + 1) % sizeof(rx_buf);
	}

	return ret;
}

/** 
  * @brief  跳转到APP入口汇编代码
  * @param  stacktop,栈顶地址
  *         entrypoint,入口地址
  * @return none
  */
static void do_jump(uint32_t stacktop, uint32_t entrypoint)
{
    asm volatile(
        "msr msp, %0	\n"
        "bx	%1	\n"
        :
        : "r"(stacktop), "r"(entrypoint)
        :);

    // 此处程序不会被执行
    for (;;)
        ;
}

/** 
  * @brief  跳转到APP入口
  * @param  none
  * @return none
  */
void jump_to_app()
{

    const uint32_t *app_base = (const uint32_t *)APP_LOAD_ADDRESS;

    /*
     * APP 的首位默认是没有数据的，直到被上位机刷入固件，如果首位是 0xffffffff ，则我们会不启动
     */
    if (app_base[0] == 0xffffffff)
    {
        return;
    }

    /*
     * 第二个字是 APP 的入口地址，他需要指向 APP flash 地址
     */
    if (app_base[1] < APP_LOAD_ADDRESS) // 小于APP地址范围
    {
        return;
    }

    if (app_base[1] >= (APP_LOAD_ADDRESS + board_info.fw_size)) // 大于APP地址范围
    {
        return;
    }

    /* 锁定FLASH */
    flash_lock();

    /* 关闭 systick 和它的中断 */
    systick_interrupt_disable();
	systick_counter_disable();

    /* 反初始化接口 */
    cfini();

    /* 复位时钟 */
    clock_deinit();

    /* 反初始化板载资源 */
    board_deinit();

#ifdef STM32F0
    /* 如果芯片是STM32F0,则复制固件中断向量表到SRAM头部，并设置中断向量表到SRAM中 */
    memcpy((void*)0x20000000, (void*)APP_LOAD_ADDRESS, VECTOR_SIZE);
    SYSCFG_CFGR1 &= ~(SYSCFG_CFGR1_MEM_MODE);
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_MEM_MODE_SRAM; 
#else
    /* 设置中断向量表位置偏移到APP起始位置 */
    SCB_VTOR = APP_LOAD_ADDRESS;
#endif

    /* 将地址放入寄存器内然后跳转 */
    do_jump(app_base[0], app_base[1]);
}

/** 
  * @brief  SysTick 中断处理函数
  * @param  none
  * @return none
  */
void sys_tick_handler(void)
{
    unsigned i;

	for (i = 0; i < NTIMERS; i++)
		if (timer[i] > 0) {
			timer[i]--;
		}

	if ((_led_state == LED_BLINK) && (timer[TIMER_LED] == 0)) {
		led_toggle(LED_BOOTLOADER);
		timer[TIMER_LED] = 50;
	}
}

/** 
  * @brief  延时ms函数
  * @param  msec,时长
  * @return none
  */
void delay(unsigned msec)
{

    timer[TIMER_DELAY] = msec;

    while (timer[TIMER_DELAY] > 0)
        ;

}

/** 
  * @brief  LED状态设置函数
  * @param  state,LED状态
  * @return none
  */
static void led_set(enum led_state state)
{
	_led_state = state;

	switch (state) {
	case LED_OFF:
		led_off(LED_BOOTLOADER);
		break;

	case LED_ON:
		led_on(LED_BOOTLOADER);
		break;

	case LED_BLINK:
		/* restart the blink state machine ASAP */
		timer[TIMER_LED] = 0;
		break;
	}
}

/** 
  * @brief  操作成功回复
  * @param  none
  * @return none
  */
static void sync_response(void)
{

    uint8_t data[] =
        {
            PROTO_INSYNC, // "in sync"
            PROTO_OK      // "OK"
        };

    cout(data, sizeof(data));

}

/** 
  * @brief  操作无效回复
  * @param  none
  * @return none
  */
static void invalid_response(void)
{

    uint8_t data[] =
        {
            PROTO_INSYNC, // "in sync"
            PROTO_INVALID // "invalid command"
        };

    cout(data, sizeof(data));

}

/** 
  * @brief  操作失败回复
  * @param  none
  * @return none
  */
static void failure_response(void)
{

    uint8_t data[] =
        {
            PROTO_INSYNC, // "in sync"
            PROTO_FAILED  // "command failed"
        };

    cout(data, sizeof(data));
}

static volatile unsigned cin_count;

/** 
  * @brief  等待数据输入
  * @param  timeout,超时时间
  * @return -1,失败.其他,接收到的数据
  */
static int cin_wait(unsigned timeout)
{

    int c = -1;

    /* start the timeout */
    timer[TIMER_CIN] = timeout;

    do
    {
        c = cin();

        if (c >= 0)
        {
            cin_count++;
            break;
        }

    } while (timer[TIMER_CIN] > 0);

    return c;
}

/** 
  * @brief      等待数据EOC输入
  * @param[in]  timeout,超时时间
  * @return     -1,失败.其他,接收到的数据
  */
inline static uint8_t wait_for_eoc(unsigned timeout)
{
    return cin_wait(timeout) == PROTO_EOC;
}

/** 
  * @brief      输出4个字节的数据
  * @param[in]  val,数据
  * @return     none
  */
static void cout_word(uint32_t val)
{
    cout((uint8_t *)&val, 4);
}

/** 
  * @brief      接收4个字节的数据
  * @param[out] wp,数据接收位置的指针
  * @param[in]  timeout,超时时间
  * @return     1,失败.其他,接收到的数据
  */
static int __attribute__((unused)) cin_word(uint32_t *wp, unsigned timeout)
{
    union {
        uint32_t w;
        uint8_t b[4];
    } u;

    for (unsigned i = 0; i < 4; i++)
    {
        int c = cin_wait(timeout);

        if (c < 0)
        {
            return c;
        }

        u.b[i] = c & 0xff;
    }

    *wp = u.w;
    return 0;
}

/** 
  * @brief      计算CRC32校验值
  * @param[in]  src,数据指针
  * @param[in]  len,数据长度
  * @param[in]  state,之前CRC校验的值
  * @return     CRC计算结果
  */
static uint32_t crc32(const uint8_t *src, unsigned len, unsigned state)
{
    static uint32_t crctab[256];

    /* check whether we have generated the CRC table yet */
    /* this is much smaller than a static table */
    if (crctab[1] == 0)
    {
        for (unsigned i = 0; i < 256; i++)
        {
            uint32_t c = i;

            for (unsigned j = 0; j < 8; j++)
            {
                if (c & 1)
                {
                    c = 0xedb88320U ^ (c >> 1);
                }
                else
                {
                    c = c >> 1;
                }
            }

            crctab[i] = c;
        }
    }

    for (unsigned i = 0; i < len; i++)
    {
        state = crctab[(state ^ src[i]) & 0xff] ^ (state >> 8);
    }

    return state;
}

/** 
  * @brief      引导程序主功能函数
  * @param[in]  timeout,退出引导程序的超时时间
  * @return     none
  */
void bootloader(unsigned timeout)
{

    uint32_t address = board_info.fw_size; /* 默认将编程指针设置到固件区末尾 */
    uint32_t first_word = 0xffffffff;

    bl_type = NONE;                        /* 首先收到数据的接口将被确定为数据来源 */

    /* 启动 SysTick 并设定为 1ms 中断间隔*/
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);
	systick_interrupt_enable();
	systick_counter_enable();

    /* 如果 timeout 变量非零，则工作在计时模式下，启动 timeout 计时 */
    if (timeout)
    {
        timer[TIMER_BL_WAIT] = timeout;
    }

	/* 默认状态下闪烁LED */
	led_set(LED_BLINK);

    while (1) // 小循环
    {
        volatile int c;
        int arg;

        static union {
            uint8_t c[256];
            uint32_t w[64];
        } flash_buffer; // 接收缓存

		led_off(LED_ACTIVITY);

        do
        {
            if (timeout && !timer[TIMER_BL_WAIT]) // 如果 timeout 溢出则立即返回
            {
                return;
            }
            c = cin_wait(0); // 尝试从上位机读取一个字节

        } while (c < 0); // 在接收到数据前死循环

		led_on(LED_ACTIVITY);

        // 处理接收到的数据
        switch (c)
        {
        // 测试同步
        case PROTO_GET_SYNC:
            if (!wait_for_eoc(2)) // 等待 EOC 结尾
            {
                goto cmd_bad;
            }

            break;

        // 读取芯片序列号
        case PROTO_GET_UDID:
        {
            uint32_t udid[3] = {0};
            char tmp[12] = {0};
            uint8_t i;

            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            udid[0] = flash_func_read_udid(0);
            udid[1] = flash_func_read_udid(1);
            udid[2] = flash_func_read_udid(2);

            for (i = 0; i < 12; i++)
            {
                tmp[i] = (udid[i / 4] >> (4 * (i % 4)));
            }

            cout((uint8_t *)tmp, 24);
        }
        break;

        // 获取设备固件区大小
        case PROTO_GET_FW_SIZE:
            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            cout((uint8_t *)&board_info.fw_size, sizeof(board_info.fw_size));
            break; 

        // 获取Bootloader版本    
        case PROTO_GET_BL_REV:
            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            cout((uint8_t *)&bl_proto_rev, sizeof(bl_proto_rev));
            break;

        // 获取设备ID
        case PROTO_GET_ID:
            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            cout((uint8_t *)&board_info.id, sizeof(board_info.id));
            break;

        // 获取设备序列号
        case PROTO_GET_SN:
            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            cout((uint8_t *)&board_info.sn, sizeof(board_info.sn)); 
            break;

        // 获取设备版本
        case PROTO_GET_REV:
            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            cout((uint8_t *)&board_info.rev, sizeof(board_info.rev)); 
            break; 

        // 获取设备FLASH结构描述
        case PROTO_GET_FLASH_STRC:
            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            cout((uint8_t *)&board_info.flash_strc, sizeof(board_info.flash_strc));
            break;  

        // 获取设备描述
        case PROTO_GET_DES:
            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            cout((uint8_t *)&board_info.device_des, sizeof(board_info.device_des));
            break; 

        // 擦写固件区
        case PROTO_CHIP_ERASE:

            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

			led_set(LED_ON);

            // 解锁FLASH,擦除整个APP区域
            flash_unlock();

            for (int i = 0; flash_func_sector_size(i) != 0; i++)
            {
                flash_func_erase_sector(i);
            }

			led_set(LED_OFF);

            // verify the erase
            for (address = 0; address < board_info.fw_size; address += 4)
                if (flash_func_read_word(address) != 0xffffffff)
                {
                    goto cmd_fail;
                }

            address = 0; // 复位地址

			led_set(LED_BLINK);

            break;

        // 烧写
        case PROTO_PROG_MULTI: // program bytes

            arg = cin_wait(50);

            if (arg < 0)
            {
                goto cmd_bad;
            }

            // 检测参数
            if (arg % 4) // 长度不是4的整数出错
            {
                goto cmd_bad;
            }

            if ((address + arg) > board_info.fw_size) // 当前地址+长度大于固件区容量出错
            {
                goto cmd_bad;
            }

            if (arg > sizeof(flash_buffer.c)) // 长度大于缓存大小出错
            {
                goto cmd_bad;
            }

            for (int i = 0; i < arg; i++) // 接收数据
            {
                c = cin_wait(1000);

                if (c < 0) // 数据为负出错
                {
                    goto cmd_bad;
                }

                flash_buffer.c[i] = c;
            }

            if (!wait_for_eoc(200))
            {
                goto cmd_bad;
            }

            if (address == 0)
            {
                // 将固件第一个数据保存，直到所有操作完成再写入，以防止操作中出现错误
                first_word = flash_buffer.w[0];
                // 替换固件第一个数据
                flash_buffer.w[0] = 0xffffffff;
            }

            arg /= 4;

            for (int i = 0; i < arg; i++) // 烧写数据
            {
                // 烧写数据
                flash_func_write_word(address, flash_buffer.w[i]);

                // 当即读取数据一确认写入数据无误
                if (flash_func_read_word(address) != flash_buffer.w[i])
                {
                    goto cmd_fail;
                }

                address += 4;
            }

            break;

        // 计算固件区CRC32校验值
        case PROTO_GET_CRC:
        {

            uint32_t sum = 0;

            if (!wait_for_eoc(2))
            {
                goto cmd_bad;
            }

            for (unsigned p = 0; p < board_info.fw_size; p += 4)
            {
                uint32_t bytes;

                // 替换第一位数据为实际数据
                if ((p == 0) && (first_word != 0xffffffff))
                {
                    bytes = first_word;
                }
                else
                {
                    bytes = flash_func_read_word(p);
                }

                sum = crc32((uint8_t *)&bytes, sizeof(bytes), sum);
            }

            cout_word(sum);
        }
        break;

        // 确认完成烧写并启动
        case PROTO_BOOT:

            if (!wait_for_eoc(1000))
            {
                goto cmd_bad;
            }

            // 写入替换的第一字节的数据
            if (first_word != 0xffffffff)
            {
                flash_func_write_word(0, first_word);

                if (flash_func_read_word(0) != first_word)
                {
                    goto cmd_fail;
                }

                // 回复变量数据防止flash写入出错
                first_word = 0xffffffff;
            }

            sync_response();
            delay(100);

            // 停止并跳转到应用程序
            return;

        default:
 			goto cmd_bad;
        }

        // 我们收到一个有效指令，可能已经连接到了升级程序，停止timeout计时
        timeout = 0;

        // 发送操作成功信号
        sync_response();
        continue;

    cmd_bad:
        // 发送操作无效信号
        invalid_response();
        continue;

    cmd_fail:
        // 发送操作失败信号
        failure_response();
        continue;
    }
}

/********* Copyright (C) 2018 YiQiChuang(ShanXi) Electronic Technology CO,LTD  *****END OF FILE****/
