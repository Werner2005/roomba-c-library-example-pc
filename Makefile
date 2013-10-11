PROG := connector
FILES := connector.c 
OBJS := $(FILES:%.c=%.o)
ROOMBALIBDIR := ../lib
ROOMBACONFIG := --enable-nonblocking
CFLAGS += -Wall -Werror -ansi -pedantic -g -DDEBUG -D_BSD_SOURCE -D_POSIX_C_SOURCE=199506L -g -I$(ROOMBALIBDIR)/includes
LOADLIBES:= -pthread -Llib -lroomba
all:lib/Makefile $(PROG) 

$(PROG): $(OBJS)
	make -C lib all
	$(CC) $(CFLAGS) -o $@  $^ $(LOADLIBES)

lib/Makefile:
	-@mkdir lib
	@cd lib;\
	if test -f $(ROOMBALIBDIR)/configure; then \
		$(ROOMBALIBDIR)/configure $(ROOMBACONFIG);\
	elif test -f ../$(ROOMBALIBDIR)/configure; then \
		../$(ROOMBALIBDIR)/configure $(ROOMBACONFIG); \
	else \
		echo "Error: Roomba Library Configure"; \
		exit 1; \
	fi

clean: 
	-rm $(PROG) $(OBJS)
	-rm -R lib
test: all
	./connector /dev/ttyUSB0
