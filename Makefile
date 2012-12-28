# Comment/uncomment the following line to disable/enable debugging
DEBUG = y

TARGET = gsmMuxd
SRC = gsm0710.c buffer.c
OBJS = gsm0710.o buffer.o

CC = gcc
LD = gcc
CFLAGS = -Wall
LDLIBS = -lm

ifeq ($(DEBUG),y)
  CFLAGS += -DDEBUG
endif


all: $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(LD) $(LDLIBS) -o $@ $(OBJS)

.PHONY: all clean
