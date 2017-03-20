#################################################################
EXECUTABLE=camera

#################################################################
#compiler
CC=g++

#################################################################
#include files
CFLAGS=`pkg-config opencv --cflags` `pkg-config opencv --libs`
CFLAGS+=-static-libstdc++ \
	-static-libasan \
	-Ofast -g \
	-fsanitize=address \
	-fno-omit-frame-pointer
CFLAGS+=-lusb
LDFLAGS:=

#################################################################
#compile all c++ files in dir
SRC=main.cpp

#################################################################
OUTPUT=./output

all:$(EXECUTABLE)

$(EXECUTABLE):$(SRC)
	$(CC) $(SRC) $(LDFLAGS) $(CFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)

asan_run:
	export ASAN_OPTIONS=symbolize=1
	export ASAN_SYMBOLIZER_PATH=$(shell which llvm-symbolizer)
	./$(EXECUTABLE)
	
#################################################################
