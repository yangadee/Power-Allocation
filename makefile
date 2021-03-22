CC= gcc
CFLAGS+=-I.
LDFLAGS+=-L.
TARGET:= sum
objs:= $(patsubst %.c,%.o,$(wildcard *.c))
.PHONY:clean all

all: ${TARGET} clean
${TARGET}: $(objs)
        @echo "[LINK] $@"
        $(CC)  $(objs) -o $@ $(LDFLAGS)
%.o: %.c
        @echo "[CC] $@"
        $(CC) -o $@ -c $< $(CFLAGS)
clean:
        rm -f *.o
        @echo "Clean done"
