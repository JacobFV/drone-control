export function formatValue(value: unknown): string {
  if (value === undefined || value === null || value === "") return "—";
  if (typeof value === "number") {
    return Number.isInteger(value) ? value.toLocaleString() : value.toFixed(3);
  }
  return String(value);
}

export function formatPolicy(policy: unknown): string {
  if (!policy) return "—";
  if (typeof policy === "string") return policy;
  if (typeof policy === "object") {
    const name = (policy as { name?: string }).name;
    return name || JSON.stringify(policy);
  }
  return String(policy);
}

export function upper(value: unknown): string {
  return String(value ?? "").toUpperCase();
}
