TARGET=test
CC=gcc
CFLAGS=-g
LINK=-ldl -lpthread
SRC=robot_sdk.c
LIB_TARGET=librobot.so
STATIC_LIB_TARGET=librobot.a

all: lib test

lib:
	$(CC) -o $(LIB_TARGET) $(SRC) $(LINK) -fPIC -shared $(CFLAGS)
	
librobot.o:
	$(CC) -o $@ -c $(SRC) $(LINK) $(CFLAGS)
	
static_lib: librobot.o
	ar -r $(STATIC_LIB_TARGET) $^

test:
	$(CC) -o $@ test.c -L. -lrobot $(CFLAGS)

clean:
	rm -f $(TARGET) *.o *.a *.so