import React from "react";
import { AbsoluteFill, Series } from "remotion";
import { C } from "./theme";
import {
  SceneHook,
  SceneTitle,
  SceneCollision,
  SceneInsight,
  SceneBridge,
  SceneRig,
  SceneTerminal,
  SceneWire,
  SceneDrones,
  SceneBrains,
  SceneStack,
  SceneTrajectory,
  SceneFinale,
} from "./scenes";
import { SubtitleTrack } from "./subtitles";

// 30fps. Durations sized to the narration beats in subtitles.tsx.
// 180+60+258+90+180+114+228+105+99+189+195+102+174 = 1974 frames = 65.8s
export const Film: React.FC = () => {
  return (
    <AbsoluteFill style={{ backgroundColor: C.bg }}>
      <Series>
        <Series.Sequence durationInFrames={180}><SceneHook /></Series.Sequence>
        <Series.Sequence durationInFrames={60}><SceneTitle /></Series.Sequence>
        <Series.Sequence durationInFrames={258}><SceneCollision /></Series.Sequence>
        <Series.Sequence durationInFrames={90}><SceneInsight /></Series.Sequence>
        <Series.Sequence durationInFrames={180}><SceneBridge /></Series.Sequence>
        <Series.Sequence durationInFrames={114}><SceneRig /></Series.Sequence>
        <Series.Sequence durationInFrames={228}><SceneTerminal /></Series.Sequence>
        <Series.Sequence durationInFrames={105}><SceneWire /></Series.Sequence>
        <Series.Sequence durationInFrames={99}><SceneDrones /></Series.Sequence>
        <Series.Sequence durationInFrames={189}><SceneBrains /></Series.Sequence>
        <Series.Sequence durationInFrames={195}><SceneStack /></Series.Sequence>
        <Series.Sequence durationInFrames={102}><SceneTrajectory /></Series.Sequence>
        <Series.Sequence durationInFrames={174}><SceneFinale /></Series.Sequence>
      </Series>
      <SubtitleTrack />
    </AbsoluteFill>
  );
};
