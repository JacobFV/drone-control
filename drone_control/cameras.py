"""
OV-series camera models for the ESP32-bridged drones (E99 class).

The toy E99 / ESP32-CAM family streams MJPEG from an OmniVision sensor over a
flaky 2.4 GHz WiFi link. The *sensor* can do megapixels, but what actually
reaches the station is a small frame at a modest, jittery frame rate — the
ESP32's JPEG encoder + WiFi throughput are the bottleneck, not the optics. The
simulator must reproduce what the link delivers, so each model below carries the
**streamed** resolution and a realistic effective FPS over the bridge, not the
sensor's datasheet maximum.

These drive the sim camera renderer (resolution + FOV) and the frame cadence
(fps). ``hfov_deg`` is the horizontal field of view of the lens actually fitted
to these modules (wide, cheap plastic lenses).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True, frozen=True)
class CameraModel:
    id: str
    name: str
    width: int          # streamed frame width (px)
    height: int         # streamed frame height (px)
    fps: float          # realistic effective frame rate over the ESP32 WiFi bridge
    hfov_deg: float     # horizontal field of view of the fitted lens
    sensor: str         # human-readable sensor note
    jitter: float = 0.35  # fraction of frame-interval RNG jitter (link is bursty)

    @property
    def aspect(self) -> float:
        return self.width / self.height


# Effective streamed configs (not datasheet maxima). FPS numbers reflect what an
# ESP32-CAM-class bridge realistically sustains at that resolution on 2.4 GHz.
OV_CAMERAS: dict[str, CameraModel] = {
    "ov7670": CameraModel(
        "ov7670", "OV7670", 320, 240, 12.0, 50.0,
        "0.3MP VGA, no JPEG — ESP32 streams QVGA, slow"),
    "ov2640": CameraModel(
        "ov2640", "OV2640", 640, 480, 18.0, 70.0,
        "2MP UXGA sensor w/ HW JPEG — VGA stream (E99 default)"),
    "ov3660": CameraModel(
        "ov3660", "OV3660", 640, 480, 15.0, 65.0,
        "3MP QXGA sensor w/ HW JPEG — VGA stream"),
    "ov5640": CameraModel(
        "ov5640", "OV5640", 800, 600, 12.0, 64.0,
        "5MP sensor w/ HW JPEG — SVGA stream, autofocus"),
}

DEFAULT_CAMERA = "ov2640"   # the E99's typical module


def get_camera(model: str | None) -> CameraModel:
    return OV_CAMERAS.get((model or DEFAULT_CAMERA).lower(), OV_CAMERAS[DEFAULT_CAMERA])


def list_cameras() -> list[dict[str, object]]:
    return [
        {"id": c.id, "name": c.name, "width": c.width, "height": c.height,
         "fps": c.fps, "hfovDeg": c.hfov_deg, "sensor": c.sensor}
        for c in OV_CAMERAS.values()
    ]
