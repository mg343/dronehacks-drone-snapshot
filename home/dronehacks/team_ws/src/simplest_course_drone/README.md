# simplest_course_drone

Flies the course from a map file. Everything it needs is in this package: no other team package
has to be built or installed beside it. Outside it: ROS 2 Humble, the organisers' kit
(`skybeta_flight_interface`, started by the launch file), numpy and PyYAML.

    COMPETITOR_PKG=simplest_course_drone
    COMPETITOR_LAUNCH=simplest_course_drone.launch.py

## How it flies

    maps/*.yaml --planner/--> route --route_steps.py--> fly_to / hover / land lines
        --fusion_node.py (fusion.py's trust gate, tunnel coast)--> engine.py
        --> /skybeta/setpoint/{pose,twist} --> the kit --> PX4

## Files, and where each came from

The other `simplest_*` packages were the rungs this was built up on. Their code is copied here,
with only the import lines changed, so this package can be built on its own.

| File | What it does | Copied from |
|---|---|---|
| `node.py` | the node: map, route, steps, checkpoint progress | (this package) |
| `route_steps.py` | the route as steps; the nose keeps the launch heading (`turns:=true`: turns where there is room) | (this package) |
| `fusion_node.py` | subscriptions, the trust gate, publishing setpoints | `simplest_fusion_drone/node.py` |
| `fusion.py`, `image.py` | "where am I, can I trust it?"; image brightness/texture | `simplest_fusion_drone` |
| `engine.py`, `steps.py` | the flight-tested step engine; the step vocabulary (and the square) | `simplest_square_drone` |
| `hearing.py`, `subscriptions.py` | topic name matching; the relevant topic subset | `simplest_listen_drone` |
| `course.py` | course frame to odom, nearest wall, checkpoints | `simplest_map_drone` |
| `planner/` | A* route planner: `course_map`, `clearance`, `grid_search`, `route` | `dronehacks_planner` (unmodified) |
| `maps/` | course maps; the default is `course_arena_2026-11-08_flown.yaml` | `simplest_map_drone/maps` |

## Tests

From this folder, with nothing else on the path:

    PYTHONPATH=. python3 -m pytest test

`test/flights/` holds 20 Hz fusion inputs from three real skybeta-11 flights, replayed by
`test_real_flights.py`. Not yet built with colcon on the drone or flown.
