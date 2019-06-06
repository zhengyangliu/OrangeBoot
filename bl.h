/***************************************************************************************************
* OrangeBoot                                                                                       *
* Copyright (C) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                              *
*                                                                                                  *
* This file is part of OrangeBoot project.                                                         *
*                                                                                                  *
* @file     bl.h                                                                                   *
* @brief                                                                                           *
* @author   Arthur Zheng                                                                           *
* @email    arthurzheng150@gmail.com                                                               *
* @version  0.2.0.0                                                                                *
* @date     2018/07/15                                                                             *
*                                                                                                  *
*--------------------------------------------------------------------------------------------------*
* Remark         :                                                                                 *
*--------------------------------------------------------------------------------------------------*
* Change History :                                                                                 *
* <Date>     | <Version> | <Author>       | <Description>                                          *
*--------------------------------------------------------------------------------------------------*
* 2018/04/19 | 0.1.0.0   | Arthur Zheng   | Create file                                            *
* 2018/07/15 | 0.2.0.0   | Arthur Zheng   | Restyle the structure of project                       *
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

#ifndef __BL_H
#define __BL_H

/* Defination ------------------------------------------------------------------------------------*/
#define MAX_DEVICE_ID_LENGTH      32
#define MAX_DEVICE_SN_LENGTH      32
#define MAX_DEVICE_REV_LENGTH     16
#define MAX_FLASH_STRC_LENGTH     128
#define MAX_DEVICE_DES_LENGTH     128

#define LED_ACTIVITY	1
#define LED_BOOTLOADER	2

/* enum for whether bootloading via USB or USART */
enum Interface_Type
{
    NONE,
	USART,
	USB
};

/* board info forwarded from board-specific code to booloader */
struct boardinfo
{
    uint8_t id[MAX_DEVICE_ID_LENGTH];
    uint8_t sn[MAX_DEVICE_SN_LENGTH];
    uint8_t rev[MAX_DEVICE_REV_LENGTH];
    uint8_t flash_strc[MAX_FLASH_STRC_LENGTH];
    uint8_t device_des[MAX_DEVICE_DES_LENGTH];
    uint32_t fw_size;
    uint32_t systick_mhz;
}__attribute__((packed));

extern struct boardinfo board_info;

extern void jump_to_app(void);
extern void bootloader(unsigned timeout);
extern void delay(unsigned msec);
extern unsigned char hex_to_char(unsigned char bHex);

/** 
  * @brief generic timers
  */
#define NTIMERS       4
#   define TIMER_BL_WAIT 0
#   define TIMER_CIN     1
#   define TIMER_LED	 2
#   define TIMER_DELAY   3
extern volatile unsigned timer[NTIMERS]; /* each timer decrements every millisecond if > 0 */

/** 
  * @brief board function in main.c
  */
extern void led_on(unsigned led);
extern void led_off(unsigned led);
extern void led_toggle(unsigned led);
extern void board_deinit(void);
extern void clock_deinit(void);
extern uint32_t flash_func_sector_size(unsigned page);
extern void flash_func_erase_sector(unsigned page);
extern void flash_func_write_word(uint32_t address, uint32_t word);
extern uint32_t flash_func_read_word(uint32_t address);
extern uint32_t flash_func_read_otp(uint32_t address);
extern uint32_t flash_func_read_udid(uint32_t address);
extern uint32_t get_mcu_id(void);

/** 
  * @brief Interface in/output.
  */
extern inline void cinit(void *config, uint8_t interface);
extern inline void cfini(void);
extern inline int cin(void);
extern inline void cout(uint8_t *buf, unsigned len);

/* generic receive buffer for async reads */
extern void buf_put(uint8_t b);
extern int buf_get(void);

#endif

/********* Copyright (C) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD  *****END OF FILE****/

