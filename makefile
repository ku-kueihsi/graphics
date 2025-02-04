# Compiler
CC = clang
CXX = clang++

# Compiler flags
CFLAGS_DEBUG   = -g -Wall -O0
CFLAGS_RELEASE = -O2 -Wall

CXXFLAGS_DEBUG   = -g -Wall -O0 -std=c++20
CXXFLAGS_RELEASE = -O2 -Wall -std=c++20

# Directories
BUILD_DIR = out
SRCS = $(wildcard *.c) $(wildcard *.cpp)
OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(SRCS:.c=.o)))
OBJS := $(OBJS:.cpp=.o)
TARGET = $(BUILD_DIR)/my_program

# Default target
all: release

# Debug build
debug: CFLAGS = $(CFLAGS_DEBUG)
debug: CXXFLAGS = $(CXXFLAGS_DEBUG)
debug: $(TARGET)

# Release build
release: CFLAGS = $(CFLAGS_RELEASE)
release: CXXFLAGS = $(CXXFLAGS_RELEASE)
release: $(TARGET)

# Create the build directory if it doesn't exist
$(BUILD_DIR):
		mkdir -p $(BUILD_DIR)

# Link the binary
$(TARGET): $(OBJS) | $(BUILD_DIR)
		$(CXX) $(CXXFLAGS) -o $@ $^

# Compile each C source file into an object file
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
		$(CC) $(CFLAGS) -c $< -o $@

# Compile each C++ source file into an object file
$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
		$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
		rm -rf $(BUILD_DIR)

# Phony targets (not actual files)
.PHONY: all debug release clean