/***************************************************************************************************
* OrangeBoot                                                                                       *
* Copyright (c) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                              *
* Copyright (c) 2012-2014 PX4 Development Team.                                                    * 
* Copyright (c) 2010 libopencm3 project (Gareth McMullin)                                          *
*                                                                                                  *
* This file is part of OrangeBoot project.                                                         *
*                                                                                                  *
* @file     cdcacm.h                                                                               *
* @brief    USB CDCACM driver                                                                      *
* @author   Arthur Zheng                                                                           *
* @email    arthurzheng150@gmail.com                                                               *
* @version  1.0.0.2                                                                                *
* @date     2019/06/06                                                                             *
*                                                                                                  *
*--------------------------------------------------------------------------------------------------*
* Remark         :                                                                                 *
*--------------------------------------------------------------------------------------------------*
* Change History :                                                                                 *
* <Date>     | <Version> | <Author>       | <Description>                                          *
*--------------------------------------------------------------------------------------------------*
* unknown    | unknown   | Gareth McMullin| unknown                                                *
* unknown    | unknown   | David Sidrane  | unknown                                                *
* 2018/07/15 | 1.0.0.0   | Arthur Zheng   | Restyle the structure of project                       *
* 2018/07/17 | 1.0.0.1   | Arthur Zheng   | Updata USBlib include for lastest libopencm3           *
* 2019/06/06 | 1.0.0.2   | Arthur Zheng   | Move otg_fs.h include pos incase of compilation        *
*                                           failure when we use stm32f0                            *
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

#pragma once

extern void usb_cinit(void);
extern void usb_cfini(void);
extern int usb_cin(void);
extern void usb_cout(uint8_t *buf, unsigned len);

/********* Copyright (c) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD  *****END OF FILE****/
