include $(shell rospack find mk)/cmake.mk

install:
	@if [ -d build ]; then \
		cd build; \
		$(MAKE) install; \
	else \
		mkdir -p build; \
		cd build; \
		cmake ..; \
		$(MAKE) install; \
	fi 
	    