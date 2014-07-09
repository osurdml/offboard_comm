OUTPUT = arkex

CFLAGS = -Wall -lm

SRCS = main.c

SRCDIR = src
INCDIR = include
OBJDIR = build

OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SOURCES))

all:
	gcc -o $(OUTPUT) src/main.c
#	gcc -o $(OUTPUT) $(SRCS)

clean:
	rm $(OUTPUT)
#	rm $(OBJS)
#	rm $(

