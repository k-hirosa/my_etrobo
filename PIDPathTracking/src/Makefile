CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2

TARGET = pid_test
OBJS = main.o PIDController.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

main.o: main.cpp PIDController.h
	$(CXX) $(CXXFLAGS) -c main.cpp

PIDController.o: PIDController.cpp PIDController.h
	$(CXX) $(CXXFLAGS) -c PIDController.cpp

clean:
	rm -f $(OBJS) $(TARGET)

