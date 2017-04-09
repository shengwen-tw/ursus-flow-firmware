#===============================================================#

EXECUTABLE=monitor

#===============================================================#

#compiler
CC=g++

#===============================================================#

CFLAGS=`pkg-config opencv --cflags` `pkg-config opencv --libs`
CFLAGS+=-static-libstdc++ \
	-static-libasan \
	-Ofast -g \
	-fsanitize=address \
	-fno-omit-frame-pointer
CFLAGS+=-lusb-1.0
LDFLAGS:=

#===============================================================#

SRC=main.cpp

#===============================================================#

all:$(EXECUTABLE)

$(EXECUTABLE):$(SRC)
	$(CC) $(SRC) $(LDFLAGS) $(CFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)

asan_run:
	export ASAN_OPTIONS=symbolize=1
	export ASAN_SYMBOLIZER_PATH=$(shell which llvm-symbolizer)
	./$(EXECUTABLE)

.ONESHELL:
astyle:
	cd ..
	astyle -r --style=linux --indent=tab=8 *.cpp *.hpp
	cd camera/

#===============================================================#	
