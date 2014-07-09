all: offboard_comm

# clang++ -std=c++11 -Iinclude -Iinclude/common -I/usr/include/boost -o test test.cpp -lboost_system

offboard_comm: offboard_comm.o
	clang++ offboard_comm.o -o offboard_comm -lboost_system

offboard_comm.o: offboard_comm.cpp
	clang++ -Iinclude/common -I/usr/include/boost -c offboard_comm.cpp

clean:
	rm -rf *o offboard_comm
