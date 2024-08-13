DEBUG := 0
CXX := clang++
CFLAGS := -Isrc -Wall -std=c++20

ifeq ($(DEBUG),1)
	CFLAGS += -g
else
	CFLAGS += -O3 -flto
endif

HEADERS := $(wildcard src/*.hpp)
SRC_OBJECTS := $(patsubst %.cpp, %.o, $(wildcard src/*.cpp))
MAIN_OBJECTS := $(patsubst %.cpp, %.o, $(wildcard main/*.cpp))
EXES := $(patsubst main/%.cpp, bin/%, $(wildcard main/*.cpp))

.PHONY: all clean
all: $(EXES)

clean:
	rm -f $(SRC_OBJECTS) $(MAIN_OBJECTS) $(EXES)

$(SRC_OBJECTS): %.o: %.cpp $(HEADERS)
	$(CXX) $(CFLAGS) -c $< -o $@

$(MAIN_OBJECTS): %.o: %.cpp $(HEADERS)
	$(CXX) $(CFLAGS) -c $< -o $@

$(EXES): bin/%: main/%.o $(SRC_OBJECTS)
	@mkdir -p bin
	$(CXX) $< $(SRC_OBJECTS) $(CFLAGS) -o $@
