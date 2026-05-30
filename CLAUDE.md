# Agent guide — drone-control

## Workflow policy (hard rule)

**Work on `main`. Do not create feature branches.** Commit directly to `main`;
if you ever end up on another branch, merge it back into `main` and continue
there. Keep everything on a single line of history.

## Repo conventions

- Video/binary assets are not committed. `film/public/media/`,
  `film/assets/source/`, and `film/assets/clips/**/*.mp4` are gitignored;
  generated clips are reproducible from `tools/`.
- Python runs from the repo venv (`.venv`); the Electron app spawns
  `drone_control.service` and loads the built UI in `ui/dist/` (gitignored —
  build with `npm --prefix ui run build`).

## Film / b-roll tooling

Launch-film assets live in `film/`. To (re)generate b-roll from the real UI and
simulator, see `film/assets/clips/README.md`:

- `tools/film_scene_clips.py` — per-scene sim footage (omniscient orbit + drone POV).
- `tools/record_ui_offscreen.sh` + `electron/record.js` — captures the real
  control-station UI (full tile wall, then each tile maximized) via Electron
  offscreen rendering. Use offscreen capture, not screen-grab: the X display is
  shared and screen-grabs get corrupted.

Recording hooks in `electron/main.js` and `ui/src/store/SessionContext.tsx`
(maximize a tile via the `#max=<tileId>` URL hash) are env-guarded
(`DRONE_REC`, `DRONE_SERVICE_URL`) and do not affect the shipped app.
