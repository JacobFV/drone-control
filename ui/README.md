# Drone Control Station — UI

React + Vite single-page app for the Electron control station. Replaces the
legacy vanilla renderer in `../app/`.

## Build

```bash
cd ui
npm install
npm run build      # type-checks then bundles to ui/dist (base './', file://-safe)
```

Electron loads `ui/dist/index.html` when present and falls back to the legacy
`app/index.html` otherwise, so the desktop app still launches before a build.

## Develop

```bash
npm run dev        # Vite dev server (browser); IPC bridge is Electron-only
npm run typecheck  # tsc --noEmit
```

## Architecture

- `src/api/` — `bridge.ts` (window.droneStation IPC contract), `client.ts`
  (typed wrappers for every backend endpoint), `types.ts` (service JSON shapes).
- `src/store/StationContext.tsx` — global state + background polling
  (state 5s, session 5s, reconstruction 2s, runtime 1s).
- `src/components/` — `TitleBar`, `DroneTree` (left), `Viewport`
  (Forward / Down / 3D-Sim / World Model), `Inspector` (workflow tabs),
  `panels.tsx` (Connect / Fly / Record / Reconstruct / Swarm), `SettingsDrawer`.
- `src/lib/pose3d.ts` — orbit-camera + pinhole projection for the 3D track view.

The inspector is organised by **workflow** (Connect → Fly → Record →
Reconstruct) with a persistent **Swarm · Batched VLA** panel; configuration
lives in a separate **Settings** drawer rather than mixed into operations.
All backend calls route through the Electron IPC bridge (never direct `fetch`).
The world-model `.ply` snapshot uses the `fetchBinary` bridge helper.
