CXX = g++

CXXFLAGS = -std=c++17 -Wall -O2 -pthread 

LIBS = -lwiringPi 

INCLUDES = -I/usr/include/eigen3

SRCS = collection_v2.cpp

OBJS = $(SRCS:.cpp=.o)

TARGET = sensor_logger

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJS) -o $(TARGET) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
