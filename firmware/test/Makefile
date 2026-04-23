CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wno-absolute-value -I. -I../include
TARGETS = test_ik test_transform test_balance test_offsets test_gait_ik test_gait_taper test_servos test_engage test_buzzer test_button test_gpio_aux test_aux_servo test_comms_out test_pin_registry test_pin_registry_aux test_command_handlers test_command_handlers_aux test_battery_led test_ota_auth

# Mock firmware build
MOCK_FLAGS = -std=c++17 -O2 -Wall \
	-I../mock \
	-I. \
	-I../include \
	-I../lib/monocypher \
	-I../.pio/libdeps/mechdog/ArduinoJson/src \
	-DHOST_BUILD=1 -DMOCK_FIRMWARE=1 -DWIFI_ENABLED=1 -DPINS_VERIFIED=1 \
	-include Arduino.h

MOCK_SRC = \
	../mock/mock_main.cpp \
	../mock/mock_sensor_task.cpp \
	../mock/imu_mock.cpp \
	../mock/sonar_mock.cpp \
	../mock/button_mock.cpp \
	../mock/physics.cpp \
	../mock/net_tcp.cpp \
	../mock/mock_globals.cpp \
	../mock/buzzer_mock.cpp \
	../mock/gpio_aux_mock.cpp \
	../src/main.cpp \
	../src/balance.cpp \
	../src/battery_led.cpp \
	../src/command_handlers.cpp \
	../src/comms_out.cpp \
	../src/gait.cpp \
	../src/offsets.cpp \
	../src/pin_registry.cpp \
	../src/servos.cpp \
	../src/ota_auth.cpp \
	../lib/monocypher/monocypher.c \
	../lib/monocypher/monocypher-ed25519.c \
	test_owner_pubkey.cpp

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

test_gait_taper: test_gait_taper.cpp Arduino.h mock_arduino.h ../src/gait.cpp \
	../include/gait.h ../include/gait_math.h ../include/ik.h ../include/body_transform.h \
	../include/balance.h ../include/offsets.h ../include/servos.h ../include/config.h
	$(CXX) $(CXXFLAGS) -o $@ test_gait_taper.cpp

test_servos: test_servos.cpp mock_arduino.h mock_preferences.h ../include/servos.h ../include/config.h ../src/servos.cpp ../src/offsets.cpp ../src/pin_registry.cpp
	$(CXX) $(CXXFLAGS) -o $@ test_servos.cpp

test_engage: test_engage.cpp mock_arduino.h mock_preferences.h ../include/servos.h ../include/config.h ../src/servos.cpp ../src/offsets.cpp ../src/pin_registry.cpp
	$(CXX) $(CXXFLAGS) -o $@ test_engage.cpp

test_buzzer: test_buzzer.cpp ../mock/buzzer_mock.cpp mock_arduino.h ../include/buzzer.h ../include/config.h
	$(CXX) $(CXXFLAGS) -o $@ test_buzzer.cpp ../mock/buzzer_mock.cpp

test_button: test_button.cpp mock_arduino.h ../include/button.h ../include/config.h ../src/button.cpp
	$(CXX) $(CXXFLAGS) -o $@ test_button.cpp

test_gpio_aux: test_gpio_aux.cpp mock_arduino.h ../include/gpio_aux.h ../src/gpio_aux.cpp
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 -o $@ test_gpio_aux.cpp

test_aux_servo: test_aux_servo.cpp mock_arduino.h mock_preferences.h ../include/servos.h ../include/config.h ../src/servos.cpp ../src/offsets.cpp ../src/pin_registry.cpp
	$(CXX) $(CXXFLAGS) -DAUX_SERVOS_ENABLED=1 -o $@ test_aux_servo.cpp

test_comms_out: test_comms_out.cpp mock_arduino.h ../include/comms_out.h ../src/comms_out.cpp
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 -I../mock -I../.pio/libdeps/mechdog/ArduinoJson/src -o $@ test_comms_out.cpp

test_pin_registry: test_pin_registry.cpp mock_arduino.h ../include/pin_registry.h ../src/pin_registry.cpp
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 -o $@ test_pin_registry.cpp

test_pin_registry_aux: test_pin_registry.cpp mock_arduino.h ../include/pin_registry.h ../src/pin_registry.cpp
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 -DAUX_SERVOS_ENABLED=1 -DTEST_AUX_ENABLED -o $@ test_pin_registry.cpp

test_command_handlers: test_command_handlers.cpp mock_arduino.h ../include/config.h ../src/command_handlers.cpp \
    ../include/ota_auth.h ../src/ota_auth.cpp \
    ../lib/monocypher/monocypher.h ../lib/monocypher/monocypher.c \
    ../lib/monocypher/monocypher-ed25519.h ../lib/monocypher/monocypher-ed25519.c \
    test_owner_pubkey.cpp
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 -I../mock -I../lib/monocypher \
		-I../.pio/libdeps/mechdog/ArduinoJson/src \
		-o $@ test_command_handlers.cpp \
		../src/ota_auth.cpp \
		../lib/monocypher/monocypher.c \
		../lib/monocypher/monocypher-ed25519.c \
		test_owner_pubkey.cpp

test_command_handlers_aux: test_command_handlers.cpp mock_arduino.h ../include/config.h ../src/command_handlers.cpp \
    ../include/ota_auth.h ../src/ota_auth.cpp \
    ../lib/monocypher/monocypher.h ../lib/monocypher/monocypher.c \
    ../lib/monocypher/monocypher-ed25519.h ../lib/monocypher/monocypher-ed25519.c \
    test_owner_pubkey.cpp
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 -DAUX_SERVOS_ENABLED=1 -I../mock -I../lib/monocypher \
		-I../.pio/libdeps/mechdog/ArduinoJson/src \
		-o $@ test_command_handlers.cpp \
		../src/ota_auth.cpp \
		../lib/monocypher/monocypher.c \
		../lib/monocypher/monocypher-ed25519.c \
		test_owner_pubkey.cpp

test_battery_led: test_battery_led.cpp mock_arduino.h ../include/battery_led.h ../include/config.h ../src/battery_led.cpp
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 -o $@ test_battery_led.cpp ../src/battery_led.cpp

test_ota_auth: test_ota_auth.cpp mock_arduino.h test_owner_pubkey.cpp \
    ../include/ota_auth.h ../src/ota_auth.cpp \
    ../lib/monocypher/monocypher.h ../lib/monocypher/monocypher.c \
    ../lib/monocypher/monocypher-ed25519.h ../lib/monocypher/monocypher-ed25519.c
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 \
	    -I../lib/monocypher \
	    -I../.pio/libdeps/mechdog/ArduinoJson/src \
	    -o $@ test_ota_auth.cpp \
	    ../lib/monocypher/monocypher.c \
	    ../lib/monocypher/monocypher-ed25519.c

test: all
	@for t in test_*; do [ -x "$$t" ] || continue; echo "=== $$t ===" && ./$$t; done

.PHONY: all clean test
