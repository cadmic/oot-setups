CXX := clang++
# TODO: add easy debug mode
CFLAGS := -Isrc -Wall -std=c++17 -g -O3 -flto

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
