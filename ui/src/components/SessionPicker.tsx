import { useEffect, useRef, useState } from "react";
import { useSession } from "../store/SessionContext";
import { ChevronDownIcon, RecordIcon } from "./icons";

/**
 * Top-left session picker: the live session (when active) plus every saved past
 * session, grouped by environment. Selecting a past session puts the wall into
 * review mode; selecting "Live" returns to the live feed.
 */
export function SessionPicker() {
  const { state, snapshot, reviewSessionId, setReviewSessionId } = useSession();
  const [open, setOpen] = useState(false);
  const rootRef = useRef<HTMLDivElement>(null);

  const liveId = snapshot?.session.active ? snapshot.session.sessionId : null;

  useEffect(() => {
    const onClick = (e: MouseEvent) => {
      if (rootRef.current && !rootRef.current.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener("mousedown", onClick);
    return () => document.removeEventListener("mousedown", onClick);
  }, []);

  const allSessions = (state?.environments ?? []).flatMap((env) =>
    env.sessions.map((s) => ({ env, session: s })),
  );
  const selected = reviewSessionId
    ? allSessions.find((x) => x.session.id === reviewSessionId)?.session
    : null;

  const label = reviewSessionId
    ? selected?.name ?? "Session"
    : liveId
      ? "● Live session"
      : "No active session";

  const pick = (id: string | null) => {
    setReviewSessionId(id);
    setOpen(false);
  };

  return (
    <div className="session-picker" ref={rootRef}>
      <button type="button" className="session-picker-btn" onClick={() => setOpen(!open)}>
        {!reviewSessionId && liveId && <RecordIcon size={10} />}
        <span className="session-picker-label">{label}</span>
        <ChevronDownIcon size={14} />
      </button>
      {open && (
        <div className="session-menu">
          <button
            type="button"
            className={`session-menu-item${!reviewSessionId ? " is-active" : ""}`}
            onClick={() => pick(null)}
          >
            <RecordIcon size={9} />
            <span>{liveId ? "Live session" : "Live (idle)"}</span>
          </button>
          {state?.environments.map((env) =>
            env.sessions.length === 0 ? null : (
              <div key={env.id} className="session-menu-group">
                <div className="session-menu-group-label">
                  {env.name} · {env.kind}
                </div>
                {env.sessions.map((s) => (
                  <button
                    key={s.id}
                    type="button"
                    className={`session-menu-item${reviewSessionId === s.id ? " is-active" : ""}`}
                    onClick={() => pick(s.id)}
                  >
                    <span className="session-menu-name">{s.name}</span>
                    <span className="session-menu-meta">
                      {s.state === "recording" ? "live" : s.duration} · {s.records.length} rec
                    </span>
                  </button>
                ))}
              </div>
            ),
          )}
        </div>
      )}
    </div>
  );
}
