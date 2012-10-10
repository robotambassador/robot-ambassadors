include $(shell rospack find mk)/cmake.mk

install:
		@if [ -d build ]; then \
			cd build; \
			sudo $(MAKE) install; \
		else \
			$(MAKE) all; \
			cd build; \
			sudo $(MAKE) install; \
		fi 

install/strip:
		@if [ -d build ]; then \
			cd build; \
			sudo $(MAKE) install/strip; \
		else \
			$(MAKE) all; \
			cd build; \
			sudo $(MAKE) install/strip; \
		fi 

uninstall:
		@if [ -d build ]; then \
			cd build; \
			sudo $(MAKE) uninstall; \
		else \
			echo No build directory; \
		fi
