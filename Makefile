CC = gcc
CFLAGS = -O2 -Wall
LDLIBS = -lx264 -ljpeg

rtvideo: main.c
	$(CC) $(CFLAGS) -o rtvideo main.c $(LDLIBS)

run: rtvideo
	./rtvideo | ffplay -fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0 -framedrop -sync ext -i -

clean:
	rm -f rtvideo *.o