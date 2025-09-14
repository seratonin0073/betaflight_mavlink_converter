CXX = g++
CXXFLAGS = -std=c++17 -g -Wall -Iinclude
LDFLAGS = -lpthread -lboost_system

SRC_DIR = src
SRC = $(wildcard $(SRC_DIR)/*.cpp) \
      $(wildcard $(SRC_DIR)/serial/*.cpp) \
      $(wildcard $(SRC_DIR)/msp/*.cpp) \
      $(wildcard $(SRC_DIR)/mavlink/*.cpp) \
      $(wildcard $(SRC_DIR)/udp/*.cpp)

OBJ = $(SRC:.cpp=.o)
TARGET = bfmavconverter

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
