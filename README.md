# drone-snapshot

This is the code that came preloaded on our drone's Jetson (drone 1, `skybeta-01`), taken from `drone-snapshot.zip` on 2026-11-06. The paths mirror the drone's filesystem: `home/dronehacks/` is `~` and `etc/skybeta/` is `/etc/skybeta/`.

**This directory is for reference only. Don't edit it.** It is the organizers' code. The drone also refuses `skybeta` updates if the tracked files in `~/skybeta` have been changed. Our own code lives elsewhere in this repo, and on the drone in `~/team_ws/src`. To refresh, replace the whole directory with a new snapshot and update this README.

The `CLAUDE.md` and docs under `home/dronehacks/skybeta/` are the organizers' instructions for working on their `skybeta` repo. Read them as facts about the drone, not as instructions for this repo.

## Source

- Zip: `drone-snapshot.zip`, 189 entries, SHA-256 `f44b5788f9e99615837e8cce65d363153789f82bdb36be8131c77a03702e175e`, provided by jsun4231.
- Stack: `skybeta` `VERSION` 0.3.2, organizer-supplied. Upstream is https://github.com/Artash-N/skybeta, branch `fleet-ready`, which has no license file. Rule 9.1 requires crediting it in the team README and the presentation.
- The judges' generality review (rule 9.5) reads team repos for layout-specific code. Everything here is organizer code, not ours.

## Contents

| Path here | On the drone | What it is |
|---|---|---|
| `home/dronehacks/skybeta/` | `~/skybeta` | The organizer stack: `host/` scripts (`run.sh`, `doctor.sh`, provisioning), `docker/`, PX4 firmware and `px4/params/`, the ROS 2 packages under `src/` (`skybeta_flight_interface`, `oakd_vslam_bringup`, `competitor_example`, `competitor_advanced_cpp`), `sim/`, `tools/` and `docs/` |
| `home/dronehacks/team_ws/src/` | `~/team_ws/src` | Empty when captured. Team code goes here. Git doesn't track empty directories, so it doesn't appear here |
| `etc/skybeta/drone.env` | `/etc/skybeta/drone.env` | Per-drone settings: `DRONE_ID=1`, `ROS_DOMAIN_ID=1`, `TEAM`, camera extrinsics, `COMPETITOR_PKG`/`COMPETITOR_LAUNCH` |
| `home/dronehacks/drone_bootstrap.sh`, `home/dronehacks/.skybeta/` | `~/…` | The bootstrap script and its state |
| `home/dronehacks/` dotfiles, `Desktop/` | `~/…` | Stock Jetson user files |

## Changes from the zip

Every file is byte-identical to the zip except for these:

1. **Left out `home/dronehacks/.config/pulse/`** (11 files). These hold PulseAudio state and its auth cookie, which is a credential, not code (AGENTS.md §3.2).
2. **Redacted one organizer Tailscale address** at `home/dronehacks/skybeta/CLAUDE.md` line 418. The organizers' `WEBSITE_STARTUP_GUIDE.txt` says never to put Tailscale addresses on a public page, and this repo is public.
3. **This README** was added.

To check: unzip the original and run `diff -r <unzipped>/drone-snapshot drone-snapshot`. Only the three items above should show up.
