/***************************************************************************************************
* TheaBoot                                                                                         *
* Copyright (c) 2018 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                              *
* Copyright (c) 2012-2014 PX4 Development Team.                                                    * 
* Copyright (c) 2010 libopencm3 project (Gareth McMullin)                                          *
*                                                                                                  *
* This file is part of TheaBoot project.                                                           *
*                                                                                                  *
* @file     usart.c                                                                                *
* @brief    程序串口驱动                                                                            *
* @author   Arthur Zheng                                                                           *
* @email    15034186698@163.com                                                                    *
* @version  1.1.0.0                                                                                *
* @date     2018/07/15                                                                             *
*                                                                                                  *
*--------------------------------------------------------------------------------------------------*
* Remark         :                                                                                 *
*--------------------------------------------------------------------------------------------------*
* Change History :                                                                                 *
* <Date>     | <Version> | <Author>       | <Description>                                          *
*--------------------------------------------------------------------------------------------------*
* 2018/01/31 | 1.0.0.0   | Arthur Zheng   | Create file                                            *
* 2018/04/19 | 1.1.0.0   | Arthur Zheng   | Restyle the structure of project                       *
*--------------------------------------------------------------------------------------------------*
* Lisense       : GPLv3                                                                            *
*                                                                                                  *
* This program is free software: you can redistribute it and/or modify it under the terms of the   *
* GNU General Public License aspublished by the Free Software Foundation, either version 3 of the  *
* License, or (at your option) any later version.                                                  *
*                                                                                                  *
* This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without *
* even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU   *
* General Public License for more details.                                                         *
*                                                                                                  *
* You should have received a copy of the GNU General Public License along with this program.       *
* If not, see <http://www.gnu.org/licenses/>.                                                      *
*--------------------------------------------------------------------------------------------------*
*                                                                                                  *
***************************************************************************************************/

/* Includes --------------------------------------------------------------------------------------*/
#include "hw_config.h"

# include <libopencm3/stm32/rcc.h>
# include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "bl.h"
#include "usart.h"

/* Private variable  -----------------------------------------------------------------------------*/
uint32_t usart;

/** 
  * @brief  初始化串口接口
  * @param  none
  * @return none
  */
void uart_cinit(void *config)
{
    usart = (uint32_t)config;

	/* 设置串口 */
	usart_set_baudrate(usart, 115200);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	usart_set_mode(usart, USART_MODE_TX_RX);
	usart_set_parity(usart, USART_PARITY_NONE);
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);

	/* 使能串口 */
	usart_enable(usart);
}

/** 
  * @brief  反初始化串口接口
  * @param  none
  * @return none
  */
void uart_cfini(void)
{
    usart_disable(usart);
}

/** 
  * @brief  读取串口接收寄存器
  * @param  none
  * @return c -1,未收到数据;>=0接收到的数据
  */
int uart_cin(void)
{
    int c = -1;

    if (usart_get_flag(usart, USART_FLAG_RXNE))  /* 接受标志位置位 */
	{
		c = usart_recv(usart);            /* 读取数据 */
	}

    return c;
}

/** 
  * @brief  串口输出
  * @param  *buf 输出数据指针
  * @param  len 数据长度
  * @return none
  */
void uart_cout(uint8_t *buf, unsigned len)
{
    while (len--)
	{
		usart_send_blocking(usart, *buf++);
	}
}

/********* Copyright (c) 2018 YiQiChuang(ShanXi) Electronic Technology CO,LTD  *****END OF FILE****/
