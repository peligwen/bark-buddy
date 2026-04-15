CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wno-absolute-value -I. -I../include
TARGETS = test_ik test_transform test_balance test_offsets test_gait_ik test_servos test_lifecycle

all: $(TARGETS)

clean:
	rm -f $(TARGETS) *.json

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

test_lifecycle: test_lifecycle.cpp mock_arduino.h mock_preferences.h ../include/gait.h ../include/config.h ../src/servos.cpp ../src/offsets.cpp ../src/balance.cpp ../src/gait.cpp
	$(CXX) $(CXXFLAGS) -o $@ test_lifecycle.cpp

.PHONY: all clean
