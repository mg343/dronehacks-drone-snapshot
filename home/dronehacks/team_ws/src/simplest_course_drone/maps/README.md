# Course maps

`course_v3_step.yaml` and `course_arena_2026-11-08.yaml` are copied unmodified from DroneHacks
`nav_traj/maps/` on branch `navigation/new-course-layout` (@ af393e8), where they are generated from the organisers' course STEP model by
`dronehacks_planner.convert`. See that folder's README for how they were made.

| File | What it is |
|---|---|
| `course_v3_step.yaml` | The 2026-11-07 layout as published: Gate A, tunnel, Gate B or window, pad. |
| `course_arena_2026-11-08_flown.yaml` | **The default.** The taped course, corrected by the team's flights: adds the obstacle two flights struck west of the tunnel's south mouth, drops the rock 4 in off the east net. Its header gives the evidence. |
| `course_arena_2026-11-08.yaml` | The course as the team taped it on 2026-11-08. |

None is surveyed: the real course may sit a few centimetres off. A new course is a new file
here; no code changes.
