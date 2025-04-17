.PHONY: help debian build clean

help:
	@echo 'Usage:'
	@echo '    make help      Show this help message'
	@echo '    make debian    Create a Debian package'
	@echo '    make build     Build the project'
	@echo '    make clean     Clean up generated Debian package and artifacts'

debian:
	rm -rf .obj-*
	fakeroot debian/rules binary
	mv ../ros-humble-lidar-detect-board_*.deb .
	mv ../ros-humble-lidar-detect-board-dbgsym_*.ddeb .

build:
	. /opt/ros/humble/setup.sh && \
	colcon build \
		--symlink-install \
		--packages-select lidar_detect_board \
		--cmake-args -DCMAKE_BUILD_TYPE=Release

clean:
	rm -rf .obj-* install build log *.deb *.ddeb
