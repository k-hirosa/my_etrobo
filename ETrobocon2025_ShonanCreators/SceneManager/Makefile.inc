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
SwitchSceneManager.o\
PIDController.o\

APPL_LIBS += -lm

APPL_DIRS +=\
	$(mkfile_path)HAL\
	$(mkfile_path)HAL/SelfLocalizer\
	$(mkfile_path)SwitchSceneManager\
	$(mkfile_path)SwitchSceneManager/SceneManagers\
	$(mkfile_path)SwitchSceneManager/SceneManagers/AvoidLineObstacle/src\
	$(mkfile_path)InformationProcessing\
	$(mkfile_path)InformationProcessing/PIDController\
	$(mkfile_path)app\
	$(mkfile_path)unit\

INCLUDES += \
	-I$(mkfile_path)HAL\
	-I$(mkfile_path)HAL/SelfLocalizer\
	-I$(mkfile_path)SwitchSceneManager\
	-I$(mkfile_path)SwitchSceneManager/SceneManagers/AvoidLineObstacle/include\
	-I$(mkfile_path)InformationProcessing\
	-I$(mkfile_path)InformationProcessing/PIDController\
