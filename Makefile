#####################################################################
# Build Configurations

BINARY := proj_model
CXX := g++
CC := gcc
INCLUDE_FLAGS := -I/usr/include/opencv4
CXX_FLAGS := -ggdb -std=c++14 -Wall -Wextra -pipe
LD_FLAGS := -lGL -ldl -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc -lopencv_aruco -lopencv_calib3d -lassimp
OBJS := main shader model detect_routine
GLOBAL_HEADERS := globals

#####################################################################

OBJS_FULL := $(patsubst %,obj/%.o,$(OBJS))
GLOBAL_HEADERS_FULL := $(patsubst %,src/%.hpp,$(GLOBAL_HEADERS))

default: $(OBJS_FULL) obj/glad.o
	$(CXX) -o $(BINARY) obj/glad.o $(OBJS_FULL) $(LD_FLAGS)

$(OBJS_FULL): obj/%.o: src/%.cpp src/%.hpp $(GLOBAL_HEADERS_FULL)
	$(CXX) -c $< -o $@ $(INCLUDE_FLAGS) $(CXX_FLAGS)

obj/glad.o: src/glad.c src/glad.h
	$(CC) -c src/glad.c -o obj/glad.o

clean:
	rm -f obj/* $(BINARY)

.PHONY: clean
