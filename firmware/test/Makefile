CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -I. -I../include
TARGET = gait_tests

SRCS = main.cpp
HEADERS = mock_arduino.h kinematics.h mock_imu.h balance_pid.h

$(TARGET): $(SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)

sweep: $(TARGET)
	./$(TARGET) sweep

.PHONY: clean run sweep
