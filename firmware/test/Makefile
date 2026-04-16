CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wno-absolute-value -I. -I../include
TARGETS = test_ik test_transform test_balance test_offsets test_gait_ik test_servos test_engage

# Mock firmware build
MOCK_FLAGS = -std=c++17 -O2 -Wall \
	-I../mock \
	-I. \
	-I../include \
	-I../.pio/libdeps/mechdog/ArduinoJson/src \
	-DHOST_BUILD=1 -DMOCK_FIRMWARE=1 -DWIFI_ENABLED=1 -DPINS_VERIFIED=1 \
	-include Arduino.h

MOCK_SRC = \
	../mock/mock_main.cpp \
	../mock/mock_sensor_task.cpp \
	../mock/imu_mock.cpp \
	../mock/sonar_mock.cpp \
	../mock/physics.cpp \
	../mock/net_tcp.cpp \
	../mock/mock_globals.cpp \
	../src/main.cpp \
	../src/balance.cpp \
	../src/command_handlers.cpp \
	../src/gait.cpp \
	../src/offsets.cpp \
	../src/servos.cpp

all: $(TARGETS)

bark-mock: $(MOCK_SRC)
	$(CXX) $(MOCK_FLAGS) -pthread -o $@ $(MOCK_SRC)

clean:
	rm -f $(TARGETS) bark-mock *.json

test_ik: test_ik.cpp mock_arduino.h ../include/ik.h
	$(CXX) $(CXXFLAGS) -o $@ test_ik.cpp

test_transform: test_transform.cpp mock_arduino.h ../include/ik.h ../include/body_transform.h
	$(CXX) $(CXXFLAGS) -o $@ test_transform.cpp

test_balance: test_balance.cpp mock_arduino.h ../include/ik.h ../include/body_transform.h ../include/balance.h ../src/balance.cpp
	$(CXX) $(CXXFLAGS) -o $@ test_balance.cpp ../src/balance.cpp

test_offsets: test_offsets.cpp mock_arduino.h mock_preferences.h ../include/offsets.h
	$(CXX) $(CXXFLAGS) -o $@ test_offsets.cpp

test_gait_ik: test_gait_ik.cpp mock_arduino.h ../include/ik.h ../include/body_transform.h ../include/gait_math.h
	$(CXX) $(CXXFLAGS) -o $@ test_gait_ik.cpp

test_servos: test_servos.cpp mock_arduino.h mock_preferences.h ../include/servos.h ../include/config.h
	$(CXX) $(CXXFLAGS) -o $@ test_servos.cpp

test_engage: test_engage.cpp mock_arduino.h mock_preferences.h ../include/servos.h ../include/config.h ../src/servos.cpp
	$(CXX) $(CXXFLAGS) -o $@ test_engage.cpp

.PHONY: all clean
