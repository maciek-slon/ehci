#NOTICE: You must have opencv from sourceforge's cvs
APP      = _ehci.so
CC       = c++
CFLAGS   = -fpic -c -g -O3 -I /usr/local/include/opencv/ -I /usr/include/python2.5/ 
LDFLAGS  = -L /usr/local/lib/ -lGL -lglut -lcv -lhighgui -lcvaux  -lml -lcxcore -shared
SRCS     = 6dofhead.cpp
OBJS     = $(SRCS:.cpp=.o)

all: $(APP)
	

.cpp.o:	
	$(CC) $(CFLAGS) ehci_wrap.c $< 

$(APP):	$(OBJS)
	$(CC) $(OBJS) ehci_wrap.o $(LDFLAGS) -o$(APP)

clean:
	rm -f *.o *~ $(APP)
