# Optimization level, can be [0, 1, 2, 3, s].
#     0 = turn off optimization. s = optimize for size.
#
OPT = -O1
# OPT = -O1         # for debugging

# Object files directory
# Warning: this will be removed by make clean!
#
OBJDIR = obj_app

# Target file name (without extension)
TARGET = $(OBJDIR)/stmio

# Define all C source files (dependencies are generated automatically)
INCDIRS += Inc

SOURCES += Src/main.c
SOURCES += Src/sserial.c
SOURCES += Src/crc8.c
SOURCES += Src/stm32f1xx_hal_msp.c
SOURCES += Src/stm32f1xx_it.c
SOURCES += Src/system_stm32f1xx.c

INCDIRS += Drivers/STM32F1xx_HAL_Driver/Inc

SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_cortex.c
# SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_dma.c
# SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_flash.c
# SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_flash_ex.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_gpio.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_gpio_ex.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_pwr.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_rcc.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_rcc_ex.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_tim.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_tim_ex.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_uart.c
SOURCES += Drivers/STM32F1xx_HAL_Driver/Src//stm32f1xx_hal_spi.c

INCDIRS += Drivers/CMSIS/Device/ST/STM32F1xx/Include
INCDIRS += Drivers/CMSIS/Include
SOURCES += Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f105xc.s

CPPFLAGS += -DSTM32F105xC
CPPFLAGS += -DUSE_HAL_DRIVER
LDSCRIPT = STM32F105RBTx_FLASH.ld

#============================================================================
OBJECTS += $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(SOURCES))))
CPPFLAGS += $(addprefix -I,$(INCDIRS))

#---------------- Preprocessor Options ----------------
#  -fsingle...    make better use of the single-precision FPU
#  -g             generate debugging information
#  -save-temps    preserve .s and .i-files
#
CPPFLAGS += -fsingle-precision-constant
CPPFLAGS += -g
# CPPFLAGS += -save-temps=obj

#---------------- C Compiler Options ----------------
#  -O*            optimization level
#  -f...          tuning, see GCC documentation
#  -Wall...       warning level
#
CFLAGS += $(OPT)
CFLAGS += -std=gnu11
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wall
CFLAGS += -fno-builtin ## from old
CFLAGS += -nostartfiles
CFLAGS += -Wfatal-errors
#CFLAGS += -Wstrict-prototypes
#CFLAGS += -Wextra
#CFLAGS += -Wpointer-arith
#CFLAGS += -Winline
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wundef

# Use a friendly C dialect
CPPFLAGS += -fno-strict-aliasing
CPPFLAGS += -fwrapv

#---------------- C++ Compiler Options ----------------
#
CXXFLAGS += $(OPT)
CXXFLAGS += -ffunction-sections
CXXFLAGS += -fdata-sections
CXXFLAGS += -Wall

#---------------- Assembler Options ----------------
#  -Wa,...    tell GCC to pass this to the assembler
#

#---------------- Linker Options ----------------
#  -Wl,...      tell GCC to pass this to linker
#  -Map         create map file
#  --cref       add cross reference to  map file
#
LDFLAGS += $(OPT)
LDFLAGS += -lm
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections

# LDFLAGS += -specs=nano.specs -u _printf_float -u _scanf_float
LDFLAGS += -T$(LDSCRIPT)

#============================================================================

POSTLD  = cp

# Compiler flags to generate dependency files
#
GENDEPFLAGS = -MMD -MP

# Combine all necessary flags and optional flags
# Add target processor to flags.
#
CPU = -mthumb -mcpu=cortex-m3 -mfloat-abi=soft

CFLAGS   += $(CPU)
CXXFLAGS += $(CPU)
ASFLAGS  += $(CPU)
LDFLAGS  += $(CPU)

# Default target
#
all:  gccversion build showsize

build: elf hex bin lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex
bin: $(TARGET).bin
lss: $(TARGET).lss
sym: $(TARGET).sym

# Display compiler version information
#
gccversion:
	@$(CC) --version

# Show the final program size
#
showsize: build
	@echo
	@$(SIZE) $(TARGET).elf 2>/dev/null

flash: $(TARGET).bin
	st-flash --reset write $(TARGET).bin 0x08000000

# Target: clean project
#
clean:
	@echo Cleaning project:
	rm -rf $(OBJDIR)

# Include the base rules
#
include base.mak
include toolchain.mak

# Include the dependency files
#
-include $(OBJECTS:.o=.d)

# Listing of phony targets
#
.PHONY: all build flash clean elf lss sym showsize gccversion
