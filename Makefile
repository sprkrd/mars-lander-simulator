all: main-interactive

OBJECTS=common.o environment.o lander.o

CXX_FLAGS=-std=c++11 -O3 -Wall -Wextra -pedantic

LD_SFML=-lsfml-system -lsfml-window -lsfml-graphics

$(OBJECTS): %.o: %.cpp %.hpp
	g++ -c $< $(CXX_FLAGS)

libmars.a: $(OBJECTS)
	ar rcs libmars.a $(OBJECTS)

main-interactive: main-interactive.cpp libmars.a
	g++ $(CXX_FLAGS) -o main-interactive main-interactive.cpp -L. -lmars $(LD_SFML)

clean:
	rm -rf main-interactive *.o libmars.a

