scanner_compose_file := ~/third_party/Scanner/.devcontainer/compose.yaml


all: build

build:
	docker compose build

start_cameras:
	docker compose -f $(scanner_compose_file) run --rm -v ./scripts:/scripts backend /bin/bash /scripts/start_multi_camera_node.sh 5

start_recording:
	docker compose -f $(scanner_compose_file) run --rm -v ./scripts:/scripts backend /bin/bash /scripts/start_recording.sh

calibrate_cameras:
	docker compose up kalibr

validate_calibrations:
	docker compose run --rm kalibr bash camera_validator.sh


