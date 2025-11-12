SHELL := /bin/bash
.ONESHELL:
MAKEFLAGS += --warn-undefined-variables

ROS_SETUP ?= /opt/ros/humble/setup.bash
WORKSPACE_DIR ?= /workspace/aeris
OVERLAY_SETUP ?= install/setup.bash
COLCON_PACKAGES := aeris_msgs aeris_map aeris_perception aeris_mesh_agent aeris_orchestrator

ROS_ENV := if [ ! -f "$(ROS_SETUP)" ]; then \
              echo "[make] Missing ROS setup: $(ROS_SETUP). Run inside the Humble container or override ROS_SETUP."; \
              exit 1; \
            fi; \
            if [ ! -d "$(WORKSPACE_DIR)" ]; then \
              echo "[make] Workspace $(WORKSPACE_DIR) not found. Set WORKSPACE_DIR=/path/to/aeris."; \
              exit 1; \
            fi; \
            source "$(ROS_SETUP)"; \
            cd "$(WORKSPACE_DIR)";

OVERLAY_ENV := if [ -f "$(OVERLAY_SETUP)" ]; then \
                 source "$(OVERLAY_SETUP)"; \
               else \
                 echo "[make] Overlay $(OVERLAY_SETUP) missing; run 'make build' first."; \
               fi;

.PHONY: help build run-sim run-map first-tile kpis viewer-serve judge-demo

help:
	@echo "Aeris judge workflow targets (override WORKSPACE_DIR if not /workspace/aeris):"
	@echo "  make build        - colcon build $(COLCON_PACKAGES)"
	@echo "  make run-sim      - launch perception_demo (ROS + mesh stack)"
	@echo "  make run-map      - run aeris_map map_tile_publisher"
	@echo "  make first-tile   - measure /map/tiles first message"
	@echo "  make kpis         - run KPI collector -> /tmp/kpi.json"
	@echo "  make viewer-serve - serve docs/viewer via python http"
	@echo "  make judge-demo   - build + launch sim/map + KPIs + viewer"

build:
	set -euo pipefail
	$(ROS_ENV)
	colcon build --packages-select $(COLCON_PACKAGES) --symlink-install
	if [ -f "$(OVERLAY_SETUP)" ]; then source "$(OVERLAY_SETUP)"; fi
	printf "[build] Completed colcon build for %s\n" "$(COLCON_PACKAGES)"

run-sim:
	set -euo pipefail
	$(ROS_ENV)
	$(OVERLAY_ENV)
	ros2 launch software/edge/launch/perception_demo.launch.py

run-map:
	set -euo pipefail
	$(ROS_ENV)
	$(OVERLAY_ENV)
	ros2 run aeris_map map_tile_publisher

first-tile:
	set -euo pipefail
	$(ROS_ENV)
	$(OVERLAY_ENV)
	python3 software/edge/tools/first_tile_timer.py --timeout-sec 120

kpis:
	set -euo pipefail
	$(ROS_ENV)
	$(OVERLAY_ENV)
	software/edge/tools/collect_kpis.sh

viewer-serve:
	set -euo pipefail
	if [ ! -d "docs/viewer" ]; then echo "[viewer-serve] docs/viewer missing"; exit 1; fi
	cd docs/viewer
	python3 -m http.server 8000

judge-demo:
	set -euo pipefail
	$(ROS_ENV)
	mkdir -p log
	SIM_PID=
	MAP_PID=
	VIEWER_PID=
	trap 'status=$$?; [ -n "$$SIM_PID" ] && kill $$SIM_PID >/dev/null 2>&1 || true; \
	      [ -n "$$MAP_PID" ] && kill $$MAP_PID >/dev/null 2>&1 || true; \
	      [ -n "$$VIEWER_PID" ] && kill $$VIEWER_PID >/dev/null 2>&1 || true; exit $$status' EXIT
	colcon build --packages-select $(COLCON_PACKAGES) --symlink-install
	if [ -f "$(OVERLAY_SETUP)" ]; then source "$(OVERLAY_SETUP)"; fi
	echo "[judge-demo] Launching perception stack in background..."
	ros2 launch software/edge/launch/perception_demo.launch.py > log/judge_sim.log 2>&1 &
	SIM_PID=$$!
	echo "[judge-demo] Launching map tile publisher in background..."
	ros2 run aeris_map map_tile_publisher > log/judge_map.log 2>&1 &
	MAP_PID=$$!
	sleep 6
	echo "[judge-demo] Collecting KPIs..."
	software/edge/tools/collect_kpis.sh
	echo "[judge-demo] Serving Leaflet viewer on http://localhost:8000 ..."
	cd docs/viewer && python3 -m http.server 8000 > ../../log/judge_viewer.log 2>&1 &
	VIEWER_PID=$$!
	echo "[judge-demo] Press Ctrl-C to stop viewer; KPIs in /tmp/kpi.json" 
	wait $$VIEWER_PID
