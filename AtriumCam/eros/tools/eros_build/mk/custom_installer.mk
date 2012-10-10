include $(shell rospack find mk)/cmake.mk

install:
		@if [ -d build ]; then \
			cd build; \
			sudo PATH=${PATH} $(MAKE) custom_install; \
		else \
			$(MAKE) all; \
			cd build; \
			sudo PATH=${PATH} $(MAKE) custom_install; \
		fi 

uninstall:
		@if [ -d build ]; then \
			cd build; \
			sudo PATH=${PATH} $(MAKE) custom_uninstall; \
		else \
			echo No build directory; \
		fi
