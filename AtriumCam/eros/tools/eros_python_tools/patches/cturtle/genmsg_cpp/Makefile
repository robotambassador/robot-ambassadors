CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

ifeq ($(MAKE),)
  MAKE=make
endif


# The all target does the heavy lifting, creating the build directory and
# invoking CMake
all:
	@rm -f cross_compiled
	@mkdir -p build/first_pass
	@mkdir -p build/second_pass
	-mkdir -p bin
	@if ! (cd build/first_pass && cmake -DCMAKE_TOOLCHAIN_FILE=$(ROS_ROOT)/core/rosbuild/rostoolchain.cmake ../..); then \
		echo "[rosbuild] CMake failed; trying to clean and start over"; \
		rm -rf build/first_pass; \
		mkdir -p build/first_pass; \
 		cd build/first_pass && cmake -DCMAKE_TOOLCHAIN_FILE=$(ROS_ROOT)/core/rosbuild/rostoolchain.cmake ../..; \
    fi
	cd build/first_pass && $(MAKE) $(ROS_PARALLEL_JOBS)
	@if [ -f ./cross_compiled ]; then \
		if ! (cd build/second_pass && cmake -DNATIVE_BUILD_TOOLS:BOOL=ON ../..); then \
			echo "[rosbuild] CMake failed; trying to clean and start over"; \
			rm -rf build/second_pass; \
			mkdir -p build/second_pass; \
			cd build/second_pass && cmake -DNATIVE_BUILD_TOOLS:BOOL=ON ../..; \
		fi; \
		cd build/second_pass && $(MAKE) $(ROS_PARALLEL_JOBS); \
	fi

# The clean target blows everything away
# It also removes auto-generated message/service code directories, 
# to handle the case where the original .msg/.srv file has been removed,
# and thus CMake no longer knows about it.
clean:
	-cd build/first_pass && $(MAKE) clean
	-cd build/second_pass && $(MAKE) clean
	rm -rf build cross_compiled

# All other targets are just passed through
test: 

tests:

test-future:

gcoverage: 

#include $(shell rospack find mk)/buildtest.mk

