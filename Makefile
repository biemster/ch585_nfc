all : flash

TARGET:=ch585_nfc
TARGET_MCU:=CH585
TARGET_MCU_PACKAGE:=CH585M

include ../../CH570/ch32fun/ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean
