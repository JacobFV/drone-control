// Small inline-SVG icon set (stroke = currentColor) used across the chrome so
// buttons/tabs read as icons rather than text labels.
import type { SVGProps } from "react";

type IconProps = SVGProps<SVGSVGElement> & { size?: number };

function base({ size = 16, ...rest }: IconProps) {
  return {
    width: size,
    height: size,
    viewBox: "0 0 24 24",
    fill: "none",
    stroke: "currentColor",
    strokeWidth: 1.8,
    strokeLinecap: "round" as const,
    strokeLinejoin: "round" as const,
    ...rest,
  };
}

export const MaximizeIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5" />
  </svg>
);

export const RestoreIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <path d="M9 4 4 9M15 4l5 5M9 20l-5-5M15 20l5-5" />
    <rect x="8" y="8" width="8" height="8" rx="1" />
  </svg>
);

export const PanelIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <rect x="3" y="4" width="18" height="16" rx="2" />
    <path d="M15 4v16" />
  </svg>
);

export const FlightIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <path d="M21 4 3 11l6 2 2 6 4-7 6-8Z" />
  </svg>
);

export const ConnectionsIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <path d="M5 13a10 10 0 0 1 14 0M8.5 16.5a5 5 0 0 1 7 0" />
    <circle cx="12" cy="20" r="1" />
  </svg>
);

export const ConfigIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <path d="M4 7h10M18 7h2M4 17h2M10 17h10" />
    <circle cx="16" cy="7" r="2" />
    <circle cx="8" cy="17" r="2" />
  </svg>
);

export const ChevronDownIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <path d="m6 9 6 6 6-6" />
  </svg>
);

export const PlusIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <path d="M12 5v14M5 12h14" />
  </svg>
);

export const RecordIcon = (p: IconProps) => (
  <svg {...base(p)} fill="currentColor" stroke="none">
    <circle cx="12" cy="12" r="6" />
  </svg>
);

export const CloseIcon = (p: IconProps) => (
  <svg {...base(p)}>
    <path d="M6 6l12 12M18 6 6 18" />
  </svg>
);
