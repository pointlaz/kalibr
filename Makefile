scanner_compose_file := ~/third_party/Scanner/.devcontainer/compose.yaml


all: build

build:
	docker compose build

start:
	docker compose -f $(scanner_compose_file) -f ./compose.scanner.yaml run --rm backend /bin/bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch lazaruss idling.launch"

start_cameras:
	docker compose -f $(scanner_compose_file) -f ./compose.scanner.yaml run --rm backend /bin/bash -c "/scripts/start_multi_camera_node.sh 5"

start_recording:
	docker compose -f $(scanner_compose_file) run --rm -f ./compose.scanner.yaml backend /bin/bash -c "bash /scripts/start_recording.sh -p /recordings/calibrations/cameras -d 60"

calibrate_cameras:
	docker compose up kalibr

validate_calibrations:
	docker compose run --rm kalibr bash camera_validator.sh


