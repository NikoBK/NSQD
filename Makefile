# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -I/usr/include/opencv4

# Libraries
LIBS = -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lQt6Core -lQt6Gui -lQt6Widgets

# Source files directory
SRCDIR = src

# Build files directory
BUILDDIR = build

# Source files
SRCS := $(wildcard $(SRCDIR)/*.cpp)

# Object files
OBJS := $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.o,$(SRCS))

# Output executable
OUT = main

# Compile rule
$(OUT): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(OUT) $(OBJS) $(LIBS)

# Rule to compile source files into object files
$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJS) $(OUT)
