PROG?=main

all: $(PROG)

run: all
	LD_LIBRARY_PATH=../lib ./$(PROG)

new: clean all

clean:
	-rm $(PROG)

$(PROG): $(PROG).c ../lib/libxbee.so
	gcc $(filter %.c,$^) -g -o $@ -I ../include/ -L ../lib -lxbee -lpthread -lrt
