all: main

CXX_FLAGS=-std=c++11 -O3 -Wall -Wextra -pedantic

LD_LIBRARIES=-lsfml-system -lsfml-window -lsfml-graphics

mars-lander.o: mars-lander.hpp mars-lander.cpp
	g++ -c mars-lander.cpp $(CXX_FLAGS)

main.o: main.cpp mars-lander.hpp
	g++ -c main.cpp $(CXX_FLAGS)

main: main.o mars-lander.o
	g++ -o main mars-lander.o main.o $(LD_LIBRARIES)

clean:
	rm -rf main *.o

