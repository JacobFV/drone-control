import { SegmentedControl } from "./primitives";
import { useStation, type WorkflowStep } from "../store/StationContext";
import { ConnectPanel, FlyPanel, RecordPanel, ReconstructPanel, SimPanel, SwarmPanel } from "./panels";

const STEP_OPTIONS: { value: WorkflowStep; label: string }[] = [
  { value: "connect", label: "Connect" },
  { value: "fly", label: "Fly" },
  { value: "record", label: "Record" },
  { value: "reconstruct", label: "Reconstruct" },
];

export function Inspector() {
  const { step, setStep } = useStation();
  return (
    <aside className="sidebar rhs">
      <div className="inspector-steps">
        <SegmentedControl options={STEP_OPTIONS} value={step} onChange={setStep} ariaLabel="Workflow step" />
      </div>
      <div className="inspector-body">
        {step === "connect" && <ConnectPanel />}
        {step === "fly" && <FlyPanel />}
        {step === "record" && <RecordPanel />}
        {step === "reconstruct" && <ReconstructPanel />}
        <SwarmPanel />
        <SimPanel />
      </div>
    </aside>
  );
}
