CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wno-absolute-value -I. -I../include
TARGETS = gait_tests profile_analyzer pose_generator test_ik test_transform test_balance test_offsets test_gait_ik test_servos

all: $(TARGETS)

gait_tests: main.cpp mock_arduino.h kinematics.h mock_imu.h balance_pid.h
	$(CXX) $(CXXFLAGS) -o $@ main.cpp

profile_analyzer: profile_analyzer.cpp mock_arduino.h kinematics.h physical_model.h
	$(CXX) $(CXXFLAGS) -o $@ profile_analyzer.cpp

pose_generator: test_poses.cpp mock_arduino.h kinematics.h
	$(CXX) $(CXXFLAGS) -o $@ test_poses.cpp

clean:
	rm -f $(TARGETS) *.json

# Run all gait tests
run: gait_tests
	./gait_tests

# Run parameter sweep
sweep: gait_tests
	./gait_tests sweep

# Generate synthetic profile and analyze it
test-profile: profile_analyzer
	./profile_analyzer --generate > synthetic_profile.json
	./profile_analyzer synthetic_profile.json
	@echo "---"
	./profile_analyzer synthetic_profile.json --optimize

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

.PHONY: all clean run sweep test-profile
