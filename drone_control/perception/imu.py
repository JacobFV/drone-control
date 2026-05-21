from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Iterable

from .state import ImuSample


def imu_samples_from_jsonl(path: str | Path) -> list[ImuSample]:
    samples: list[ImuSample] = []
    for line in Path(path).read_text().splitlines():
        if not line.strip():
            continue
        item = json.loads(line)
        if not isinstance(item, dict):
            continue
        sample = imu_sample_from_mapping(item)
        if sample is not None:
            samples.append(sample)
    return samples


def imu_samples_from_csv(path: str | Path) -> list[ImuSample]:
    with Path(path).open(newline="") as handle:
        return [sample for sample in (imu_sample_from_mapping(row) for row in csv.DictReader(handle)) if sample is not None]


def latest_imu_sample(samples: Iterable[ImuSample]) -> ImuSample | None:
    latest: ImuSample | None = None
    for sample in samples:
        if latest is None or (sample.timestamp or 0.0) >= (latest.timestamp or 0.0):
            latest = sample
    return latest


def imu_sample_from_mapping(item: dict[str, Any]) -> ImuSample | None:
    timestamp = _float_or_none(item.get("timestamp", item.get("time", item.get("t"))))
    acc = _triple(item, ("ax", "ay", "az"), "acceleration")
    gyro = _triple(item, ("gx", "gy", "gz"), "gyro")
    if acc is None and gyro is None:
        return None
    return ImuSample(timestamp=timestamp, acceleration=acc, gyro=gyro)


class FileImuSource:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._mtime_ns = -1
        self._latest: ImuSample | None = None

    def latest(self) -> ImuSample | None:
        if not self.path.is_file():
            return None
        stat = self.path.stat()
        if stat.st_mtime_ns == self._mtime_ns:
            return self._latest
        self._mtime_ns = stat.st_mtime_ns
        if self.path.suffix.lower() == ".csv":
            samples = imu_samples_from_csv(self.path)
        else:
            samples = imu_samples_from_jsonl(self.path)
        self._latest = latest_imu_sample(samples)
        return self._latest


def _triple(item: dict[str, Any], keys: tuple[str, str, str], nested_key: str) -> tuple[float, float, float] | None:
    nested = item.get(nested_key)
    if isinstance(nested, (list, tuple)) and len(nested) == 3:
        values = [_float_or_none(value) for value in nested]
        if all(value is not None for value in values):
            return values[0], values[1], values[2]  # type: ignore[return-value]
    values = [_float_or_none(item.get(key)) for key in keys]
    if all(value is not None for value in values):
        return values[0], values[1], values[2]  # type: ignore[return-value]
    return None


def _float_or_none(value: object) -> float | None:
    if value in {None, ""}:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
