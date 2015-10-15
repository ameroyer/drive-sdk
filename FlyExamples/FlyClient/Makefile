# FlyServer/Client makefile
# To compile the debug verison need to overwrite CXXFLAGS variable to include -ggdb

CC = gcc 
CFLAGS = -Ofast
OUTPUTNAME = FlyClient
INCLUDE = -I../../include -I/usr/include/flycapture
LIBS = -L../../lib/C -lflycapture-c -L../../lib -lflycapture

OUTDIR = .

OBJS = FlyClient.o

${OUTPUTNAME}: ${OBJS}
	${CC} -o ${OUTPUTNAME} ${OBJS} ${LIBS} ${COMMON_LIBS} 

%.o: %.c
	${CC} ${CFLAGS} ${INCLUDE} -c $*.c
	
clean_obj:
	rm -f ${OBJS}

clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS}
