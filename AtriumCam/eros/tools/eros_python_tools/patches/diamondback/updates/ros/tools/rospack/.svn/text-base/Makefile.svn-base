# Can't include cmake.mk because we need rospack to find it, and we're
# trying to build rospack.  So we inline a stripped-down version of
# cmake.mk.
#include $(shell rospack find mk)/cmake.mk

ifeq ($(MAKE),) 
  MAKE=make
endif	

# The all target does the heavy lifting. It does two passes - the first
# includes a check for a toolchain, builds accordingly and if a cross
# compile was detected it will then do a second pass to build the native
# rospack/rosstack tools.
all:
	@rm -f cross_compiled
	-mkdir -p bin
	@mkdir -p build/first_pass
	@mkdir -p build/second_pass
	@if ! (cd build/first_pass && cmake -DCMAKE_TOOLCHAIN_FILE=$(ROS_ROOT)/core/rosbuild/rostoolchain.cmake ../..); then \
  		echo "[rosbuild] CMake failed; trying to clean and start over"; \
  		rm -f cross_compiled; rm -f build/first_pass; \
  		mkdir -p build/first_pass; \
  		cd build/first_pass && cmake -DCMAKE_TOOLCHAIN_FILE=$(ROS_ROOT)/core/rosbuild/rostoolchain.cmake ../..; \
	fi
	cd build/first_pass && $(MAKE) $(ROS_PARALLEL_JOBS)
	@if [ -f ./cross_compiled ]; then \
		if ! (cd build/second_pass && cmake -DHOST_TOOLS:BOOL=ON ../..); then \
			echo "[rosbuild] CMake failed; trying to clean and start over"; \
			rm -rf build/second_pass; \
			mkdir -p build/second_pass; \
			cd build/second_pass && cmake -DHOST_TOOLS:BOOL=ON ../..; \
		fi; \
		cd build/second_pass && $(MAKE) $(ROS_PARALLEL_JOBS); \
	fi

install: all
	cd build/first_pass && $(MAKE) install
	@if [ -f ./cross_compiled ]; then \
		cd build/second_pass && $(MAKE) install; \
	fi

# The clean target blows everything away
clean:
	-cd build/first_pass && $(MAKE) clean
	-cd build/second_pass && $(MAKE) clean
	rm -rf build cross_compiled

# These don't actually have anything to do yet.
test: all
	if cd build/first_pass && $(MAKE) -k $@; then $(MAKE) test-results; else $(MAKE) test-results && exit 1; fi

tests: all
	cd build/first_pass && $(MAKE) $@
test-future: all
	cd build/first_pass && $(MAKE) -k $@
gcoverage: all
	cd build/first_pass && $(MAKE) $@

#SRC = main.cpp
#LIBSRC = rospack.cpp \
#      tinyxml-2.5.3/tinystr.cpp \
#			tinyxml-2.5.3/tinyxml.cpp \
#			tinyxml-2.5.3/tinyxmlparser.cpp \
#			tinyxml-2.5.3/tinyxmlerror.cpp
#
#
#OBJ = $(SRC:.cpp=.o)
#LIBOBJ = $(LIBSRC:.cpp=.o)
#OUT = ../../bin/rospack
#LIBOUT = librospack.a
#
#CC=g++
#AR=ar
#LIBS=-lm
#CFLAGS= -O3 -Wall -fPIC
## For code coverage
##LIBS=-lm -lgcov
##CFLAGS= -O0 -Wall -g -fprofile-arcs -ftest-coverage
#INCLUDES=-I.
#
#default: depend $(OUT) $(LIBOUT)
#
#$(OUT): $(OBJ) $(LIBOUT)
#	${CC} $(OBJ) -o $(OUT) $(LIBS) -L. -lrospack
#
#$(LIBOUT): $(LIBOBJ)
#	${AR} cr $@ $(LIBOBJ)
#
#.cpp.o:
#	${CC} ${CFLAGS} ${INCLUDES} -c $< -o $@
#
#depend: $(SRC) $(LIBSRC)
#	gcc $(INCLUDES) -MM $(SRC) $(LIBSRC) >depend
#			  
#clean:
#	rm -f depend $(OBJ) $(OUT) $(LIBOUT) $(LIBOBJ)
#
#-include depend
