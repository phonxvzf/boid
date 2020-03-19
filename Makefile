boid: boid.cpp
	c++ -std=c++14 -ggdb -rdynamic -o boid boid.cpp -lm -lSDL2 -lSDL2_image

clean:
	rm boid

.PHONY: clean
