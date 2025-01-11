CC= gcc
CFLAGS=
LDFLAGS=
TARGET= bbs_main
OBJECTS= feather_serial.o parameters.o util.o main.o newton_raphson.o
LIBS= -lm -L. -linvkin

all: clean compile link

#NEW RULES MORE VERSATILE (CAN CHANGE FILES NAMES)
compile: $(OBJECTS)

link: $(TARGET).out

clean:
	rm -f *.o
	rm -f *.out

.PHONY: clean all compile link

# IMPLICIT RULE FOR COMPILING OBJECTS
%.o: %.c
	$(CC) -g -c $^ -o $@ $(CFLAGS)
	
%.out:
	$(CC) -g -o $(TARGET).out  $(LDFLAGS) $(OBJECTS) $(LIBS)

