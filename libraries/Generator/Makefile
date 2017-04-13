
CC :=g++

# causes the compiler to auto-detect the architecture of the build computer. -Q -v to your GCC flags to see what optimizations are enabled
CFLAGS := -pedantic -Wall #-O2
CPPFLAGS:=
#HEADER_1 := -I $(CROSS)/include #-DHARDWARE=0
HEADER_1 := -I../LinkedList #-DHARDWARE=0
#HEADER_2 := -I /usr/include
LDFLAGS :=# -L/opt/cross/lib -lboost_iostreams -lboost_system -lpthread
LDFLAGS_proto :=# -L/usr/lib/x86_64-linux-gnu -lboost_iostreams -lboost_system 


CSRC := $(wildcard *.cpp)
OBJS := $(CSRC:%.cpp=%.o)
OBJL  := $(wildcard *.o) 
HDRS := $(wildcard *.h)


.PHONY: all
all: example
  
  
example: RF24generator.o example.o 
	$(CC) -o example RF24generator.o example.o

example.o: example.cpp
	$(CC) $(CFLAGS) ${HEADER_1} -c  example.cpp

RF24generator.o: RF24generator.cpp
	$(CC) $(CFLAGS) ${HEADER_1} -c RF24generator.cpp
	

.PHONY: clean
clean:
	rm -rf *.o example
