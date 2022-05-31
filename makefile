main:
	echo "Main target"
compile: main.cpp
	g++ -c VRP.cpp
	g++ VRP.o -o result
clean:
	rm *.o
workresult:
	./result
