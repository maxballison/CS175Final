BASE = asst3

all: $(BASE)

OS := $(shell uname -s)

ifeq ($(OS), Linux)
  LIBS += -lGL -lGLU -lGLEW -lglfw
endif

ifeq ($(OS), Darwin)
  CPPFLAGS += -D__MAC__ -std=c++11 -stdlib=libc++
  LDFLAGS += -framework OpenGL -framework IOKit -framework Cocoa -L/opt/homebrew/Cellar/glfw/3.3.8/lib -L/opt/homebrew/Cellar/glew/2.2.0_1/lib
  LIBS += -lglfw.3 -lGLEW
endif

ifdef OPT
  #turn on optimization
  CXXFLAGS += -O2
else
  #turn on debugging
  CXXFLAGS += -g
endif

CXX = g++

OBJ = $(BASE).o ppm.o glsupport.o

$(BASE): $(OBJ)
	$(LINK.cpp) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJ) $(BASE)
