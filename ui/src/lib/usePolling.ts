import { useEffect, useRef } from "react";

/**
 * Invoke an async callback on an interval. The callback is kept in a ref so the
 * timer is not torn down on every render. Pass `enabled: false` to pause.
 */
export function usePolling(
  callback: () => void | Promise<void>,
  intervalMs: number,
  enabled = true,
): void {
  const saved = useRef(callback);
  saved.current = callback;

  useEffect(() => {
    if (!enabled) return;
    let cancelled = false;
    const tick = () => {
      if (!cancelled) void saved.current();
    };
    tick();
    const id = window.setInterval(tick, intervalMs);
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, [intervalMs, enabled]);
}
