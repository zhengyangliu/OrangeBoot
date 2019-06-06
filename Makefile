#--------------------------------------------------------------------------------------------------#
# OrangeBoot                                                                                       #
# Copyright (c) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD.                              #
#                                                                                                  #
# This file is part of OrangeBoot project.                                                         #
#                                                                                                  #
# File    : makeifle                                                                               #
# Brief   :                                                                                        #
# Author  : Arthur Zheng                                                                           #
# Email   : arthurzheng150@gmail.com                                                               #
# Version : 1.1.0.0                                                                                #
# Date    : 2018/07/15                                                                             #
#                                                                                                  #
#--------------------------------------------------------------------------------------------------#
# Remark         :                                                                                 #
#--------------------------------------------------------------------------------------------------#
# Change History :                                                                                 #
# <Date>     | <Version> | <Author>       | <Description>                                          #
#--------------------------------------------------------------------------------------------------#
# 2017/01/27 | 1.0.0.0   | Arthur Zheng   | Create file                                            #
# 2018/07/15 | 1.1.0.0   | Arthur Zheng   | Restyle the structure of project                       #
#--------------------------------------------------------------------------------------------------#
# Lisense       : BSD 3-Clause                                                                     #
#                                                                                                  #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR   #
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND #
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR       #
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR              #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR         #
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     #
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     #
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE            #
# POSSIBILITY OF SUCH DAMAGE.                                                                      #
#--------------------------------------------------------------------------------------------------#

#---------------------------------------------------------------------------------------------------
# Path config
#---------------------------------------------------------------------------------------------------
export BUILD_DIR_ROOT ?= build
export BL_BASE		  ?= $(wildcard .)
export LIBOPENCM3	  ?= $(wildcard libopencm3)
MKFLAGS=--no-print-directory

#---------------------------------------------------------------------------------------------------
# Toolchain config
#---------------------------------------------------------------------------------------------------
export CC	 	 	= arm-none-eabi-gcc
export OBJCOPY		= arm-none-eabi-objcopy

#---------------------------------------------------------------------------------------------------
# Common config
#---------------------------------------------------------------------------------------------------
export FLAGS		= -std=gnu99 -Os -g -Wundef -Wall -fno-builtin -fgnu89-inline\
                      -I$(LIBOPENCM3)/include -ffunction-sections -nostartfiles\
                      -lnosys -Wl,-gc-sections -Wl,-g \
					  -Werror

export COMMON_SRCS	 = bl.c
export ARCH_SRCS	 = cdcacm.c  usart.c

#---------------------------------------------------------------------------------------------------
# Tartget to build
#---------------------------------------------------------------------------------------------------
TARGETS	= \
		TARGET_HW_STM32F030_ICM_V2_0 \
		TARGET_HW_STM32F103_Atom_Warship_V2_0 \
		STM32F429_DISCO


all:	$(TARGETS) sizes

clean:
	cd libopencm3 && make --no-print-directory clean && cd ..
	rm -f *.elf *.bin *.hex 
	rm -rf build

#---------------------------------------------------------------------------------------------------
# Build command
#---------------------------------------------------------------------------------------------------
TARGET_HW_STM32F030_ICM_V2_0: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make ${MKFLAGS} -f Makefile.f0 TARGET_HW=STM32F030_ICM_V2_0 LINKER_FILE=stm32f0.ld TARGET_FILE_NAME=$@

TARGET_HW_STM32F103_Atom_Warship_V2_0: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make ${MKFLAGS} -f Makefile.f1 TARGET_HW=STM32F103_Atom_Warship_V2_0 LINKER_FILE=stm32f1.ld TARGET_FILE_NAME=$@

STM32F429_DISCO: $(MAKEFILE_LIST) $(LIBOPENCM3)
	make ${MKFLAGS} -f Makefile.f4 TARGET_HW=STM32F429_DISCO LINKER_FILE=stm32f4.ld TARGET_FILE_NAME=$@

#---------------------------------------------------------------------------------------------------
# Build libopencm3
#---------------------------------------------------------------------------------------------------
.PHONY: $(LIBOPENCM3)
$(LIBOPENCM3): 
	make -C $(LIBOPENCM3) lib

#---------------------------------------------------------------------------------------------------
# Display file size
#---------------------------------------------------------------------------------------------------
.PHONY: sizes
sizes:
	@-find build/*/ -name '*.elf' -type f | xargs size 2> /dev/null || :

#********* Copyright (C) 2019 YiQiChuang(ShanXi) Electronic Technology CO,LTD  *****END OF FILE****#
