CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wno-absolute-value -I. -I../include
TARGETS = gait_tests profile_analyzer pose_generator

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

.PHONY: all clean run sweep test-profile
