OBJS = joint.o motion.o platform.o
CC = g++
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG) -O2 -larmadillo
LFLAGS = -Wall $(DEBUG) -O2 -larmadillo

platform : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o platform

joint.o : include/joint.hpp
	$(CC) $(CFLAGS) src/joint.cpp

motion.o : include/motion.hpp
	$(CC) $(CFLAGS) src/motion.cpp


platform.o : src/platform.cpp 
	$(CC) $(CFLAGS) src/platform.cpp


clean:
	\rm *.o platform

tar:
	tar cfv platform.tar src include \
	Makefile
