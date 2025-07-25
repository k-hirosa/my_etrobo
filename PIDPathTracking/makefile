CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2 -Iinclude
LDFLAGS = -lyaml-cpp

TARGET = robot_control

SRCS = main.cpp \
       src/logger.cpp \
       src/obstacle_map.cpp \
       src/obstacle_aware_pid_controller.cpp \
       src/local_goal_manager.cpp \
       src/pid_controller.cpp

OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: all clean
