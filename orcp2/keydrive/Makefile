#
# Makefile
#
# http://www.gnu.org/software/make/manual/make.html
#

CC		:= g++
TARGET	:= keydrive

OBJECTS := console.o ../orcp2.o ../serial.o ../times.o ../robot_4wd.o ../imu.o

CFLAGS		:= -I. -I.. -DLINUX=1 

LIBRARIES	:= -lpthread -lrt

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS) main.cpp $(LIBRARIES)
	
serial_read: $(OBJECTS)
	$(CC) $(CFLAGS) -o serial_read $(OBJECTS) serial_read.cpp $(LIBRARIES)	

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $< $(LIBRARIES)
	
clean:
	rm -f *.o $(TARGET) serial_read
	
rebuild:
	make clean
	make
