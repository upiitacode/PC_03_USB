INCLUDE_DIRS:= ./
SOURCE_DIRS:=  ./

LIBS:=libusb-1.0

Output_NAME:=myBulk 

CFILES:=  Bulk_Application.c

CFLAGS+= -std=c99 -g -Wall
CPPFLAGS:= `pkg-config --cflags $(LIBS)` 
LFLAGS:= `pkg-config --libs $(LIBS)`
CC:=gcc

vpath %.h $(INCLUDE_DIRS)
vpath %.c $(SOURCE_DIRS)
vpath %.o ./

COBJECTS:= $(patsubst %.c,%.o,$(CFILES))

all: $(Output_NAME)

$(Output_NAME): $(COBJECTS)
	$(CC) $(CFLAGS) $(CPPFLAGS) *.o  $(LFLAGS) -o $@
$(COBJECTS): %.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@
clean:
	rm -f *.o ;
	rm -f $(Output_NAME) ;