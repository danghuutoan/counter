PROJECT = $(notdir $(CURDIR))
COMPILER = arm-none-eabi-

CC		= $(COMPILER)gcc
CP		= $(COMPILER)objcopy
AS		= $(COMPILER)as
LD 		= $(COMPILER)ld
OD		= $(COMPILER)objdump
SIZE    = $(COMPILER)size

DEF+=HSE_VALUE=8000000
DEF+=USE_FULL_ASSERT
DEF+=STM32F10X_MD
DEF+=USE_STDPERIPH_DRIVER


CPU = -mthumb -mcpu=cortex-m3 
OPTIMIZE=0

VPATH= app:$(STM32F103)/src:\
      $(CMSIS):\
      $(STM32F103)/startup/TrueSTUDIO:\
      $(HAL):\
      driver

PLATFORM  =platform
STM32F103 =$(PLATFORM)/stm32f103
HAL       =$(PLATFORM)/hal
CMSIS 	  =$(STM32F103)/include/cmsis



INC+= $(STM32F103)/include
INC+= $(HAL)/include
INC+= $(CMSIS)
INC+= $(PLATFORM)/common
INC+= driver/include

SRC+= main.c
SRC+= system_stm32f10x.c
SRC+= hal_com.c
#SRC+= hal_dio.c
SRC+= stm32f10x_gpio.c
SRC+= stm32f10x_usart.c
SRC+= stm32f10x_rcc.c
SRC+= misc.c
SRC+= log.c

STARTUP = startup_stm32f10x_md.s
LDSCRIPT= $(STM32F103)/linker/stm32f103.ld



OUTPUT 	= GCC-ARM/Output
DEPDIR 	= GCC-ARM/Dependencies

$(shell mkdir -p $(OUTPUT))
$(shell mkdir -p $(DEPDIR))




OBJS+=$(patsubst %.c,%.o,$(SRC))
OBJS+=$(patsubst %.s,%.o,$(STARTUP))

OBJECTS = $(addprefix $(OUTPUT)/, $(OBJS))

CFLAGS+=$(patsubst %,-I%,$(INC))
CFLAGS+=$(patsubst %,-D%,$(DEF))
CFLAGS+=-g
CFLAGS+=$(CPU)
CFLAGS+=-O$(OPTIMIZE)
CFLAGS+=-std=gnu99
CFLAGS+=-Wp,-MM,-MP,-MT,$(OUTPUT)/$(*F).o,-MF,$(DEPDIR)/$(*F).d

LDFLAGS+=$(patsubst %,-T%,$(LDSCRIPT))
LDFLAGS+=$(CPU) -Wl,-Map=$(OUTPUT)/$(PROJECT).map,--cref,--gc-sections
LDFLAGS+=$(CFLAGS)
LDFLAGS+=-lgcc -lc 
LDFLAGS+=-lm
LDFLAGS+=-lrdimon
LDFLAGS+=-lnosys


ASFLAGS+=-g
ASFLAGS+=$(CPU)

$(OUTPUT)/%.o: %.s Makefile $(LDSCRIPT) system_stm32f10x.c
	@mkdir -p $(@D)
	@$(AS) $(ASFLAGS) -mthumb $< -o $@
	@echo "AS $(notdir ${@})"

$(OUTPUT)/%.o: %.c Makefile 
	@mkdir -p $(@D)
	@$(CC) $(CFLAGS)  -c $< -o $@
	@echo "CC $(notdir ${@})"



all: elf bin size

elf: $(OUTPUT)/$(PROJECT).elf
bin: $(OUTPUT)/$(PROJECT).bin

$(OUTPUT)/$(PROJECT).elf : $(OBJECTS)
	@mkdir -p $(@D)
	@$(CC) $^ $(LDFLAGS) $(LIBS) -o $@
	@echo "LD $(notdir ${@})"
$(OUTPUT)/$(PROJECT).bin : $(OUTPUT)/$(PROJECT).elf
	@mkdir -p $(@D)
	@$(CP) -O binary $^ $@
	@echo "CP $(notdir ${@})"

size: $(OUTPUT)/$(PROJECT).elf
	@$(SIZE) $^
	
clean: 
	@rm -f $(OBJECTS) $(OUTPUT)/*.bin $(wildcard $(DEPDIR)/*.d) $(OUTPUT)/*.elf $(OUTPUT)/*.map
flash:
	@st-flash --reset write $(OUTPUT)/$(PROJECT).bin 0x8000000

-include $(wildcard $(DEPDIR)/*.d)

