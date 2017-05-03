CC = gcc
CFLAGS = -I ./

1wire: 1wire.o
	$(CC) $(LDFLAGS) 1wire.o owerr.o ownet.o crcutil.o linuxlnk.o linuxses.o owtran.o -o 1wire
1wire.o: 1wire.c
	$(CC) $(CFLAGS) -c 1wire.c owerr.c ownet.c crcutil.c linuxlnk.c linuxses.c owtran.c

# remove object files and executable when user executes "make clean"
clean:
	rm *.o 1wire
