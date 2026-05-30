# Clip library

Prepared b-roll for the launch film, rendered from the **actual** control-station
UI and simulator (not mockups). The product code will keep changing — these are a
frozen snapshot to cut with now or later.

Two sources, both reproducible from the scripts in `tools/`:

## `scenes/` — raw simulator footage (no UI)

For every sim scene, two angles rendered straight from the engine:

- `scene_<name>_omni.mp4` — 1280×720 cinematic god's-eye orbit (the "omniscient"
  ground-truth view: all drones, goal lines, scene geometry, smoke/fire/cloth).
- `scene_<name>_pov.mp4` — 640×480 drone forward-camera POV (the OV2640 stream the
  perception stack actually sees, with sensor noise).

Scenes: `open_field, warehouse, office, city, park, construction, atrium,
clothing_store, retail_store, house_on_fire`.

Regenerate:

```bash
PYTHONPATH=. .venv/bin/python tools/film_scene_clips.py \
  --out film/assets/clips/scenes --seconds 10 --drones 4
```

## `ui/` — real control-station UI captures

Captured via Electron **offscreen rendering** (`electron/record.js`), so frames are
pixel-clean and independent of any desktop. Per recorded scene:

- `ui_<scene>_wall.mp4` — ~24 s of the full tile wall (omniscient + per-drone
  camera/seg/depth tiles + trajectories + estimated trajectory + point cloud +
  splat + world segmentation), the "here's the whole control station" shot.
- `ui_<scene>_tile_<tile>.mp4` — ~14 s of a single tile maximized, the "zoom in
  and talk about this one" shots. Tiles: `omniscient`, `camera_sim_0`,
  `estimated_trajectory`, `trajectory`, `pointcloud`, `seg_sim_0`, `depth_sim_0`,
  `world_seg`.

Regenerate (renders to `film/assets/clips/ui/`):

```bash
bash tools/record_ui_offscreen.sh clothing_store 4
bash tools/record_ui_offscreen.sh house_on_fire 4
```

The recorder drives a real sim session over the service API and maximizes each tile
via the `#max=<tileId>` URL hash — no simulated clicks. Recording hooks are all
env-guarded (`DRONE_REC`, `DRONE_SERVICE_URL`) and do not affect the shipped app.

Frame geometry: offscreen frames come out 1919×1079; the encoder crops to even
dimensions for h.264.
