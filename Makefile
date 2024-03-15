# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -I/usr/include/opencv4
LDFLAGS = -L/usr/lib/qt6

# Libraries
LIBS = -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_highgui -lQt6Core -lQt6Gui -lQt6Widgets -lopencv_videoio

# Source file
SRC = camera.cpp

# Output executable
OUT = dronecam

# Compile rule
$(OUT): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(OUT) $(SRC) $(LIBS)

# Clean rule
clean:
	rm -f $(OUT)
