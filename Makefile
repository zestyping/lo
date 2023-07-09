platform=$(shell uname)

ifeq ($(platform),Darwin)
  INCDIRS=-I /usr/local/Cellar/libfreenect/master/include/libfreenect -I../opc/src
  FRAMEWORKS=-framework OpenGL -framework GLUT
  OPTS=$(INCDIRS) $(FRAMEWORKS) -Wno-deprecated -Wno-parentheses
  LIBS=-L /usr/local/Cellar/libfreenect/master/lib -lfreenect
else ifeq ($(platform),Linux)
  LIBDIR=/usr/lib/i386-linux-gnu
  LIBDIR=/usr/lib/x86_64-linux-gnu
  INCDIRS=-I../opc/src
  OPTS=$(INCDIRS) -O3 -lfreenect -lGL -lGLU -lglut
  LIBS=$(LIBDIR)/libGL.so $(LIBDIR)/libGLU.so $(LIBDIR)/libglut.so $(LIBDIR)/libfreenect.so -lpthread -lm
endif

ALL: build/play

clean:
	rm -rf build/*

build/play: play.c ../opc/src/opc_client.c
	gcc $(OPTS) -o $@ $^ $(LIBS)
