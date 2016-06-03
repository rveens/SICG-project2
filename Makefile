CXX = g++ -std=c++11 -I./Eigen -Wall
CXXFLAGS = -g -O2 -Wall -Wno-sign-compare -Iinclude -DHAVE_CONFIG_H 
OBJS = demo.o solver.o

all: project2

project2: $(OBJS)
	$(CXX) -o $@ $^ -lGLU -lGL -lglut -lpng
clean:
	rm -f $(OBJS) project2
