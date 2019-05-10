all: main

OBJECTS=mars-lander.o

CXX_FLAGS=-std=c++11 -O3 -Wall -Wextra -pedantic

LD_LIBRARIES=-lsfml-system -lsfml-window -lsfml-graphics

$(OBJECTS): %.o: %.cpp %.hpp
	g++ -c $< $(CXX_FLAGS)

main.o: main.cpp $(OBJECTS:.o=.hpp)
	g++ -c main.cpp $(CXX_FLAGS)

main: main.o $(OBJECTS)
	g++ -o main main.o $(OBJECTS) $(LD_LIBRARIES)

clean:
	rm -rf main *.o

