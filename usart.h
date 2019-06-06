/***************************************************************************************************
* TheaBoot                                                                                         *
* Copyright (c) 2018 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                              *
* Copyright (c) 2012-2014 PX4 Development Team.                                                    * 
* Copyright (c) 2010 libopencm3 project (Gareth McMullin)                                          *
*                                                                                                  *
* This file is part of TheaBoot project.                                                           *
*                                                                                                  *
* @file     usart.h                                                                                *
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

#ifndef __UART_H
#define __UART_H

extern void uart_cinit(void *config);
extern void uart_cfini(void);
extern int uart_cin(void);
extern void uart_cout(uint8_t *buf, unsigned len);

#endif

/********* Copyright (C) 2018 YiQiChuang(ShanXi) Electronic Technology CO,LTD  *****END OF FILE****/
