

CXXFLAGS = -lX11 -lGL -lpthread -lpng -lstdc++fs -std=c++17
.PHONY: clean
main: main.o
	g++ -o main main.o -lX11 -lGL -lpthread -lpng -lstdc++fs 

main.o:
	g++ -o main.o -c main.cpp $(CXXFLAGS)

clean:
	rm -rf *.o main
