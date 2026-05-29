import { Composition } from "remotion";
import { Film } from "./Film";

const FPS = 30;
// 180+60+258+90+180+114+228+105+99+189+195+102+174 = 1974 frames = 65.8s
const DURATION = 1974;

export const RemotionRoot: React.FC = () => {
  return (
    <Composition
      id="Film"
      component={Film}
      durationInFrames={DURATION}
      fps={FPS}
      width={1920}
      height={1080}
    />
  );
};
