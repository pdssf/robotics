COPPELIA_DIR=/home/paulosalgado/Documentos/ufpe/CoppeliaSim_Edu_V4_1_0_Ubuntu18_04/programming

INCLUDE_DIR=$(COPPELIA_DIR)/include
API_DIR=$(COPPELIA_DIR)/remoteApi

CC=g++
CFLAGS=-I$(API_DIR) -I$(INCLUDE_DIR) -g -Wall

DEFINES=-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255

LIBS= -lstdc++ -pthread -lrt

SOURCES+= \
    Projeto1.cpp \
    $(COPPELIA_DIR)/remoteApi/extApi.c \
    $(COPPELIA_DIR)/remoteApi/extApiPlatform.c \
    $(COPPELIA_DIR)/common/shared_memory.c

robot: $(SOURCES)
	$(CC) -o $@ $(CFLAGS) $(DEFINES) $(SOURCES) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o robot
	
	
