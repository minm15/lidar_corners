.PHONY: help debian

help:
	@echo 'Usage:'
	@echo '    make help      Show this help message'
	@echo '    make debian    Create a Debian package'
	@echo '    make clean     Clean up generated Debian package and artifacts'

debian:
	fakeroot debian/rules binary
	mv ../ros-humble-lidar-detect-board_*.deb .
	mv ../ros-humble-lidar-detect-board-dbgsym_*.ddeb .
