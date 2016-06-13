CXX = clang++ -std=c++11 -I./Eigen -Wall -Wextra
CXXFLAGS = -g -O2 -Wall -Wno-sign-compare -Iinclude -DHAVE_CONFIG_H 
OBJS = demo.o Solver.o RigidBody.o RigidBodySquare.o

all: project2

project2: $(OBJS)
	$(CXX) -o $@ $^ -lGLU -lGL -lglut -lpng
clean:
	rm -f $(OBJS) project2
