include ../Makefile.common

ifeq ($(CHIP),HD)
	OBJS+=startup/startup_stm32f10x_hd.o
endif
ifeq ($(CHIP),MD)
	OBJS+=startup/startup_stm32f10x_md.o
endif
ifeq ($(CHIP),LD)
	OBJS+=startup/startup_stm32f10x_ld.o
endif

ifdef TX
	CFLAGS+=-DDEBUG_TX
endif
ifdef SM
	CFLAGS+=-DDEBUG_SM
endif

ifndef TX
ifndef SM
	CFLAGS+=-DRELEASE
endif
endif

#OBJS+=./newlib_stubs.o
OBJS+=./system_stm32f10x.o
OBJS+=./stm32f10x_it.o

OBJS+=./hw_config.o\
      ./usb_desc.o \
	  ./usb_endp.o \
	  ./usb_istr.o \
	  ./usb_prop.o \
	  ./usb_pwr.o

OBJS+=./NRF24L01.o\
	  ./oled.o\
	  ./spi_flash.o

OBJS+=./main.o

all: src

src: app.a

app.a: $(OBJS)
	$(AR) cr app.a $(OBJS)

#startup.o:startup.s
#	$(CC) $(CFLAGS) -c -o startup.o startup.s

.PHONY: src clean

clean:
	- rm -f app.a *.o
	- rm -f startup/*.o

