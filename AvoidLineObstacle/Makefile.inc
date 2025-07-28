USE_RASPIKE_ART=1

mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-spike/Makefile

endif

APPL_CXXOBJS +=\
UltrasonicSensorWrapper.o\
ColorSensorWrapper.o\
ForceSensorWrapper.o\
IMUWrapper.o\
EncoderWrapper.o\
TwinMotorWrapper.o\
SelfLocalizer.o\
PIDController.o\
LocalGoalCreator.o\

Logger.o


APPL_LIBS += -lm

APPL_DIRS +=\
	$(mkfile_path)HAL\
	$(mkfile_path)HAL/SelfLocalizer\
	$(mkfile_path)src\
	$(mkfile_path)PIDController\
	$(mkfile_path)Logging\
	$(mkfile_path)app\
	$(mkfile_path)unit\

INCLUDES += \
	-I$(mkfile_path)HAL\
	-I$(mkfile_path)HAL/SelfLocalizer\
	-I$(mkfile_path)include\
	-I$(mkfile_path)PIDController\
	-I$(mkfile_path)config\
	-I$(mkfile_path)Logging\

