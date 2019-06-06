/***************************************************************************************************
* OrangeBoot                                                                                       *
* Copyright (C) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                              *
*                                                                                                  *
* This file is part of OrangeBoot project.                                                         *
*                                                                                                  *
* @file     hw_config.h                                                                            *
* @brief                                                                                           *
* @author   Arthur Zheng                                                                           *
* @email    arthurzheng150@gmail.com                                                               *
* @version  0.3.0.0                                                                                *
* @date     2019/06/06                                                                             *
*                                                                                                  *
*--------------------------------------------------------------------------------------------------*
* Remark         :                                                                                 *
*--------------------------------------------------------------------------------------------------*
* Change History :                                                                                 *
* <Date>     | <Version> | <Author>       | <Description>                                          *
*--------------------------------------------------------------------------------------------------*
* 2015/05/17 | 0.1.0.0   | david_s5       | Create file                                            *
* 2018/07/15 | 0.2.0.0   | Arthur Zheng   | Restyle the structure of project                       *
* 2019/06/06 | 0.3.0.0   | Arthur Zheng   | Update file for new libopencm3                         *
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

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

/***************************************************************************************************
* TARGET_HW_STM32F030_ICM_V2_0                                                                     *
*                                                                                                  *
* Company  : YiQiChuang(ShanXi) Electronic Technology CO,LTD                                       *
* Version  : V2.0                                                                                  *
* Brief    : Science storm robot intelligent controller module                                     *
***************************************************************************************************/
#ifdef TARGET_HW_STM32F030_ICM_V2_0

/* Device text information */
#define DEVICE_ID                      "YQC SSRob ICM"                        /*!< Device model */
#define DEVICE_REV                     "2.0"                                  /*!< Device version */
#define DEVICE_SN                      "3DFC40D115E8174"                      /*!< Device serial number */
#define DEVICE_FLASH_STRC              "@Internal/0x08000000/6*1Ka,58*1Kg"    /*!< Device flash structure description */
#define DEVICE_DES                     "单片机: STM32F030C8T6\r\n意启创科技风暴机器人系列智能控制器模块V2.0"  /*!< Device description */                

/* Interface enable */
#define INTERFACE_USB                  0             /*!< USB interface enable */
#define INTERFACE_USART                1             /*!< USART interface enable */

/* Bootloader config */
#define BOOTLOADER_DELAY               2000          /*!< Boot application delay */  
#define BOOTLOADER_PAGE                6             /*!< Number of page of bootloader area */
#define APP_LOAD_ADDRESS               0x08001800    /*!< Application start address */

/* FLASH stucture */
#define BOARD_FLASH_PAGES              64 
#define BOARD_FLASH_PAGE_SIZE          0x400

/* Interrupt vector table size */
#define VECTOR_SIZE                    (45 * 4)

/* Peripheral config */
#define BOARD_PORT_USART               GPIOA               /*!< USART interface config */
#define BOARD_PORT_USART_AF            GPIO_AF1
#define BOARD_PIN_TX                   GPIO9
#define BOARD_PIN_RX                   GPIO10
#define BOARD_USART_PIN_CLOCK          RCC_GPIOA
#define BOARD_USART                    USART1
#define BOARD_USART_CLOCK              RCC_USART1

#define BOARD_FORCE_BL_PIN             GPIO15              /*!< force stay in bootloader pin config */
#define BOARD_FORCE_BL_PORT            GPIOB
#define BOARD_FORCE_BL_CLOCK           RCC_GPIOB
#define BOARD_FORCE_BL_PULL            GPIO_PUPD_PULLUP    /*!< pull up */
#define BOARD_FORCE_BL_STATE           0                   /*!< active level */

#endif /* TARGET_HW_STM32F030_ICM_V2_0 */

/***************************************************************************************************
* TARGET_HW_STM32F103_Alientek_Warship_V2_0                                                        *
*                                                                                                  *
* Company  : Alientek                                                                              *
* Version  : V2.0                                                                                  *
* Brief    : 正点原子 STM32F103 战舰V2                                                              *
***************************************************************************************************/
#ifdef TARGET_HW_STM32F103_Alientek_Warship_V2_0

#define DEVICE_ID                      "ALIENTEK 战舰 STM32"
#define DEVICE_REV                     "2.0"
#define DEVICE_SN                      "3DFC40D115E8174"
#define DEVICE_FLASH_STRC              "@Internal/0x08000000/5*2Ka,251*2Kg"
#define DEVICE_DES                     "单片机: STM32F103ZET6\r\n原子STM32F103战舰开发板 V2.0"

#define INTERFACE_USB                  1
#define INTERFACE_USART                1

#define BOOTLOADER_DELAY               2000
#define BOOTLOADER_PAGE                5
#define APP_LOAD_ADDRESS               0x08002800
             
#define USBDEVICESTRING                "STM32 WarShip V2.0"
#define USBPRODUCTID                   0x0001
#define USBMFGSTRING                   "ALIENTEK"

#define BOARD_FLASH_PAGES              256
#define BOARD_FLASH_PAGE_SIZE          0x800

#define BOARD_PORT_USART               GPIOA
#define BOARD_PIN_TX                   GPIO9
#define BOARD_PIN_RX                   GPIO10
#define BOARD_USART_PIN_CLOCK_REGISTER RCC_APB2ENR
#define BOARD_USART_PIN_CLOCK_BIT      RCC_APB2ENR_IOPAEN
#define BOARD_USART                    USART1
#define BOARD_USART_CLOCK_REGISTER     RCC_APB2ENR
#define BOARD_USART_CLOCK_BIT          RCC_APB2ENR_USART1EN

#define BOARD_PIN_LED_ACTIVITY              GPIO5
#define BOARD_PORT_LED_ACTIVITY             GPIOB
#define BOARD_CLOCK_LED_ACTIVITY_REGISTER   RCC_APB2ENR
#define BOARD_CLOCK_LED_ACTIVITY            RCC_APB2ENR_IOPBEN
#define BOARD_PIN_LED_BOOTLOADER            GPIO5
#define BOARD_PORT_LED_BOOTLOADER           GPIOE
#define BOARD_CLOCK_LED_BOOTLOADER_REGISTER RCC_APB2ENR
#define BOARD_CLOCK_LED_BOOTLOADER          RCC_APB2ENR_IOPEEN
#define BOARD_LED_ON                        gpio_clear
#define BOARD_LED_OFF                       gpio_set

#define BOARD_FORCE_BL_PIN             GPIO0
#define BOARD_FORCE_BL_PORT            GPIOA
#define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_APB2ENR
#define BOARD_FORCE_BL_CLOCK_BIT       RCC_APB2ENR_IOPAEN
#define BOARD_FORCE_BL_MODE            GPIO_CNF_OUTPUT_PUSHPULL
#define BOARD_FORCE_BL_PULL            GPIO_CNF_INPUT_PULL_UPDOWN
#define BOARD_FORCE_BL_STATE           1

#endif /* TARGET_HW_STM32F103_Alientek_Warship_V2_0 */

/***************************************************************************************************
* TARGET_HW_STM32F429_DISCO                                                                        *
*                                                                                                  *
* Company  : ST                                                                                    *
* Version  :                                                                                       *
* Brief    :                                                                                       *
***************************************************************************************************/
#ifdef TARGET_HW_STM32F429_DISCO

#define DEVICE_ID                      "STM32F429I-DISCO"                  
#define DEVICE_REV                     "1.0"                                 
#define DEVICE_SN                      "3DFC40D115E8174"                     
#define DEVICE_FLASH_STRC              "@Internal/0x08000000/1*16Ka,3*16Kg,1*64Kg,7*128Kg,4*16Kg,1*64Kg,7*128Kg"  
#define DEVICE_DES                     "单片机: STM32F429ZIT6\r\nST官方评估板"                 

#define INTERFACE_USB                  1             
#define INTERFACE_USART                1            

#define BOOTLOADER_DELAY               2000  
#define BOOTLOADER_SECTOR              1
#define BOOTLOADER_RESERVATION_SIZE    (16 * 1024)             
#define APP_LOAD_ADDRESS               0x08004000
         
#define USBDEVICESTRING                "STM32F429I-DISCO"
#define USBPRODUCTID                   0x0001
#define USBMFGSTRING                   "ST"
#define USBPORT_HS                     1
#define USBPORT_FS                     0      

#define BOARD_FLASH_SECTORS            24
#define BOARD_FLASH_SIZE               (1024 * 1024 * 2)

#define OSC_FREQ                       8

#define BOARD_PORT_USART               GPIOA
#define BOARD_PIN_TX                   GPIO9
#define BOARD_PIN_RX                   GPIO10
#define BOARD_USART_PIN_CLOCK_REGISTER RCC_AHB1ENR
#define BOARD_USART_PIN_CLOCK_BIT      RCC_AHB1ENR_IOPAEN
#define BOARD_PORT_USART_AF            GPIO_AF7
#define BOARD_USART                    USART1
#define BOARD_USART_CLOCK_REGISTER     RCC_APB2ENR
#define BOARD_USART_CLOCK_BIT          RCC_APB2ENR_USART1EN

#define BOARD_PIN_LED_ACTIVITY         GPIO14
#define BOARD_PORT_LED_ACTIVITY        GPIOG
#define BOARD_CLOCK_LED_ACTIVITY       RCC_AHB1ENR_IOPGEN
#define BOARD_PIN_LED_BOOTLOADER       GPIO13
#define BOARD_PORT_LED_BOOTLOADER      GPIOG
#define BOARD_CLOCK_LED_BOOTLOADER     RCC_AHB1ENR_IOPGEN
#define BOARD_LED_ON                   gpio_set
#define BOARD_LED_OFF                  gpio_clear

#define BOARD_FORCE_BL_PIN             GPIO0
#define BOARD_FORCE_BL_PORT            GPIOA
#define BOARD_FORCE_BL_CLOCK_REGISTER  RCC_AHB1ENR
#define BOARD_FORCE_BL_CLOCK_BIT       RCC_AHB1ENR_IOPAEN
#define BOARD_FORCE_BL_MODE            GPIO_PUPD_PULLDOWN
#define BOARD_FORCE_BL_PULL            0
#define BOARD_FORCE_BL_STATE           1

#endif /* TARGET_HW_STM32F429_DISCO */

#endif /* HW_CONFIG_H_ */

/********* Copyright (C) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD  *****END OF FILE****/
