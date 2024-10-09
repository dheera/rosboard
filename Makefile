.PHONY: install run build-docker

install:
	python3 -m venv venv
	source venv/bin/activate
	pip3 install tornado simplejpeg rospkg

run:
	source /opt/ros/humble/setup.bash && ./run

build-docker:
	docker build -t ghcr.io/arlyx-technologies/vortex-tech-99/rosboard .