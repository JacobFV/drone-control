import type { ButtonHTMLAttributes, ReactNode } from "react";
import { formatValue } from "../lib/format";

type ButtonVariant = "default" | "primary" | "danger";

interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: ButtonVariant;
}

export function Button({ variant = "default", className = "", ...rest }: ButtonProps) {
  const cls = ["btn", variant === "primary" && "is-primary", variant === "danger" && "is-danger", className]
    .filter(Boolean)
    .join(" ");
  return <button className={cls} {...rest} />;
}

interface PanelProps {
  title?: string;
  right?: ReactNode;
  children: ReactNode;
  className?: string;
}

export function Panel({ title, right, children, className = "" }: PanelProps) {
  return (
    <section className={`panel ${className}`}>
      {(title || right) && (
        <div className="panel-title-row">
          {title && <h3 className="section-label">{title}</h3>}
          {right}
        </div>
      )}
      {children}
    </section>
  );
}

type PillTone = "default" | "ok" | "danger" | "recording";

export function Pill({ children, tone = "default" }: { children: ReactNode; tone?: PillTone }) {
  const cls = [
    "pill",
    tone === "ok" && "is-armed",
    tone === "danger" && "is-danger",
    tone === "recording" && "is-recording",
  ]
    .filter(Boolean)
    .join(" ");
  return <span className={cls}>{children}</span>;
}

export interface KeyValueEntry {
  key: string;
  value: unknown;
  mono?: boolean;
}

export function KeyValue({ entries }: { entries: KeyValueEntry[] }) {
  return (
    <dl className="kv">
      {entries.map((entry, idx) => (
        <DivPair key={`${entry.key}-${idx}`} entry={entry} />
      ))}
    </dl>
  );
}

function DivPair({ entry }: { entry: KeyValueEntry }) {
  return (
    <>
      <dt>{entry.key}</dt>
      <dd className={entry.mono ? "mono" : undefined}>{formatValue(entry.value)}</dd>
    </>
  );
}

export interface SegmentOption<T extends string> {
  value: T;
  label: string;
}

interface SegmentedControlProps<T extends string> {
  options: SegmentOption<T>[];
  value: T;
  onChange: (value: T) => void;
  ariaLabel?: string;
}

export function SegmentedControl<T extends string>({
  options,
  value,
  onChange,
  ariaLabel,
}: SegmentedControlProps<T>) {
  return (
    <div className="segmented" role="group" aria-label={ariaLabel}>
      {options.map((opt) => (
        <button
          key={opt.value}
          type="button"
          className={`segment${opt.value === value ? " is-active" : ""}`}
          onClick={() => onChange(opt.value)}
        >
          {opt.label}
        </button>
      ))}
    </div>
  );
}

interface FieldProps {
  label: string;
  children: ReactNode;
}

export function Field({ label, children }: FieldProps) {
  return (
    <label className="field">
      <span className="field-label">{label}</span>
      {children}
    </label>
  );
}
