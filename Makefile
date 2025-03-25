scanner_compose_file := "/home/${USER}/third_party/Scanner/.devcontainer/compose.yaml"
kalibr_compose_file := "/home/${USER}/third_party/kalibr/compose.scanner.yaml"


all: build

build:
	docker compose -f $(scanner_compose_file) -f $(kalibr_compose_file) build

start:
	docker compose -f $(scanner_compose_file) down
	docker compose -f $(scanner_compose_file) -f $(kalibr_compose_file) run --rm backend /bin/bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch lazaruss idling.launch"

stop:
	 docker compose -f $(scanner_compose_file) -f $(kalibr_compose_file) down

start_cameras:
	docker compose -f $(scanner_compose_file) -f $(kalibr_compose_file) exec backend /bin/bash -c "bash start_multi_camera_node.sh 5"

start_recording:
	docker compose -f $(scanner_compose_file) -f $(kalibr_compose_file) exec backend /bin/bash -c "bash start_recording.sh -p /recordings/calibrations/cameras -d 120"

calibrate_cameras:
	docker compose up --build kalibr

validate_calibrations:
	docker compose run --rm kalibr bash camera_validator.sh


