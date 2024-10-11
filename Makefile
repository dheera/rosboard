.PHONY: install run build-docker

install:
	pip3 install tornado simplejpeg rospkg

run:
	source /opt/ros/humble/setup.bash && ./run

build-docker:
	docker build -t ghcr.io/arlyx-technologies/vortex-tech-99/rosboard .

run-docker:
	docker run --rm --network ros_network -p 8888:8888 --privileged ghcr.io/arlyx-technologies/vortex-tech-99/rosboard