start_calibrations:
	docker compose up --build --detach kalibr
	docker compose exec --workdir /Calibration kalibr bash /scripts/start_calibrations.bash || docker compose down
	docker compose down
