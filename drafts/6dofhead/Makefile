#NOTICE: You must have opencv from sourceforge's cvs
APP      = 6dofhead
CC       = g++
CFLAGS   = -g -O3 -I /usr/local/include/opencv/ 
LDFLAGS  = -L /usr/local/lib/ -lGL -lglut -lcv -lhighgui -lcvaux  -lml -lcxcore 
SRCS     = 6dofhead.cpp ehci.cpp
OBJS     = $(SRCS:.cpp=.o)

all: $(APP)
	

.cpp.o:	
	$(CC) $(CFLAGS) -c $< -o $@

$(APP):	$(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o$(APP)

clean:
	rm -f *.o *~ $(APP)
