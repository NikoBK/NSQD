# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic

# Source files
SRCS = main.cpp functions.cpp  # Add your source files here

# Object files
OBJS = $(SRCS:.cpp=.o)

# Executable name
TARGET = my_program

# Default rule
all: $(TARGET)

# Rule to link object files and create the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Rule to compile source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJS) $(TARGET)

