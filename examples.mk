SOURCES = $(PROG).c ./mongoose.c
CFLAGS = -g -W -Wall -I./ -I./mc -Wno-unused-function $(CFLAGS_EXTRA) $(MODULE_CFLAGS)

all: $(PROG)

CC = gcc

SOURCES += mc/motion_motor_ctrl.c \
           mc/ut_usart.c

ifeq ($(OS), Windows_NT)
# TODO(alashkin): enable SSL in Windows
CFLAGS += -lws2_32
else
CFLAGS += -pthread
endif

ifeq ($(SSL_LIB),openssl)
CFLAGS += -DMG_ENABLE_SSL -lssl -lcrypto
endif
ifeq ($(SSL_LIB), krypton)
CFLAGS += -DMG_ENABLE_SSL ../../../krypton/krypton.c -I../../../krypton
endif
ifeq ($(SSL_LIB),mbedtls)
CFLAGS += -DMG_ENABLE_SSL -DMG_SSL_IF=MG_SSL_IF_MBEDTLS -DMG_SSL_MBED_DUMMY_RANDOM -lmbedcrypto -lmbedtls -lmbedx509
endif

ifdef ASAN
CC = clang
CFLAGS += -fsanitize=address
endif

$(PROG): $(SOURCES)
	$(CC) $(SOURCES) -o $@ $(CFLAGS)

$(PROG).exe: $(SOURCES)
	cl $(SOURCES) /I../.. /MD /Fe$@

clean:
	rm -rf *.gc* *.dSYM *.exe *.obj *.o a.out $(PROG)
