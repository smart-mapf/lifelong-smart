import { Suspense, useEffect, useMemo, useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls, Ring, useGLTF } from "@react-three/drei";
import { useAtomValue, useSetAtom } from "jotai";
import {
  mapDataAtom,
  framesAtom,
  currentFrameAtom,
  playingAtom,
  speedAtom,
  agentStatesAtom,
} from "./state";
import type { AgentState, TickAgent } from "./state";
import { Matrix4, type Group, type InstancedMesh } from "three";

type SceneMapData = { size: { width: number; height: number } };

function gridCellToScenePosition(
  col: number,
  row: number,
  mapData: SceneMapData,
  width = 1,
  height = 1
) {
  return {
    x: col - mapData.size.width / 2 + width / 2,
    z: row - mapData.size.height / 2 + height / 2,
  };
}

function argosToScenePosition(
  agent: Pick<TickAgent, "x" | "y" | "z">,
  mapData: SceneMapData
) {
  /*
   * LSMART places robots into ARGoS with:
   *   argos_x = -row
   *   argos_y = -col
   *
   * To match the native ARGoS visualisation layout on screen:
   *   screen x grows with column (left -> right)
   *   screen z grows with row    (top -> bottom)
   */
  return {
    x: -agent.y - mapData.size.width / 2 + 0.5,
    y: agent.z,
    z: -agent.x - mapData.size.height / 2 + 0.5,
  };
}

useGLTF.preload("/robot-final.gltf");

// ─── Obstacles ──────────────────────────────────────────────────────────────

function Obstacles() {
  const mapData = useAtomValue(mapDataAtom);
  if (!mapData) return null;

  return (
    <group>
      {mapData.items.map((item, i) => {
        const pos = gridCellToScenePosition(
          item.x,
          item.y,
          mapData,
          item.width,
          item.height
        );
        return (
          <mesh
            key={i}
            position={[pos.x, 0.25, pos.z]}
          >
            <boxGeometry args={[item.width, 0.5, item.height]} />
            <meshStandardMaterial color="#ffffff" />
          </mesh>
        );
      })}
    </group>
  );
}

// ─── Domain Base ────────────────────────────────────────────────────────────

function DomainBase() {
  const mapData = useAtomValue(mapDataAtom);
  if (!mapData) return null;

  return (
    <mesh
      rotation={[-Math.PI / 2, 0, -Math.PI / 2]}
      position={[0, -0.01, 0]}
      receiveShadow
    >
      <planeGeometry
        args={[mapData.size.width, mapData.size.height]}
      />
      <meshStandardMaterial color="#1a1a2e" />
    </mesh>
  );
}

function GridOverlay({
  width,
  height,
}: {
  width: number;
  height: number;
}) {
  const horizontalRef = useRef<InstancedMesh | null>(null);
  const verticalRef = useRef<InstancedMesh | null>(null);

  useEffect(() => {
    if (!horizontalRef.current || !verticalRef.current) return;

    const matrix = new Matrix4();

    for (let row = 0; row <= height; row += 1) {
      matrix.setPosition(0, 0.001, row - height / 2);
      horizontalRef.current.setMatrixAt(row, matrix);
    }
    horizontalRef.current.instanceMatrix.needsUpdate = true;

    for (let col = 0; col <= width; col += 1) {
      matrix.setPosition(col - width / 2, 0.001, 0);
      verticalRef.current.setMatrixAt(col, matrix);
    }
    verticalRef.current.instanceMatrix.needsUpdate = true;
  }, [width, height]);

  return (
    <group>
      <instancedMesh
        ref={horizontalRef}
        args={[undefined, undefined, height + 1]}
      >
        <boxGeometry args={[width, 0.002, 0.01]} />
        <meshBasicMaterial color="#ffffff" />
      </instancedMesh>
      <instancedMesh
        ref={verticalRef}
        args={[undefined, undefined, width + 1]}
      >
        <boxGeometry args={[0.01, 0.002, height]} />
        <meshBasicMaterial color="#ffffff" />
      </instancedMesh>
    </group>
  );
}

function GridIntersectionCrosses({
  width,
  height,
}: {
  width: number;
  height: number;
}) {
  const horizontalRef = useRef<InstancedMesh | null>(null);
  const verticalRef = useRef<InstancedMesh | null>(null);
  const count = (width + 1) * (height + 1);

  useEffect(() => {
    if (!horizontalRef.current || !verticalRef.current) return;

    const matrix = new Matrix4();
    let index = 0;

    for (let col = 0; col <= width; col += 1) {
      for (let row = 0; row <= height; row += 1) {
        matrix.setPosition(col - width / 2, 0.003, row - height / 2);
        horizontalRef.current.setMatrixAt(index, matrix);
        verticalRef.current.setMatrixAt(index, matrix);
        index += 1;
      }
    }

    horizontalRef.current.instanceMatrix.needsUpdate = true;
    verticalRef.current.instanceMatrix.needsUpdate = true;
  }, [width, height]);

  return (
    <group>
      <instancedMesh
        ref={horizontalRef}
        args={[undefined, undefined, count]}
      >
        <boxGeometry args={[0.08, 0.003, 0.01]} />
        <meshBasicMaterial color="#ffffff" />
      </instancedMesh>
      <instancedMesh
        ref={verticalRef}
        args={[undefined, undefined, count]}
      >
        <boxGeometry args={[0.01, 0.003, 0.08]} />
        <meshBasicMaterial color="#ffffff" />
      </instancedMesh>
    </group>
  );
}

// ─── Agent Meshes ───────────────────────────────────────────────────────────

const AGENT_COLORS: Record<string, string> = {
  unknown: "#666",
  initialized: "#ffcc00",
  active: "#00ff88",
  idle: "#4488ff",
  finished: "#888",
};
const ROBOT_YAW_OFFSET = -Math.PI / 2;

function RobotAgent({
  agent,
  status,
  mapData,
  robotScene,
}: {
  agent: TickAgent;
  status: AgentState["status"];
  mapData: SceneMapData;
  robotScene: Group;
}) {
  const color = AGENT_COLORS[status] || AGENT_COLORS.unknown;
  const pos = argosToScenePosition(agent, mapData);
  const robotClone = useMemo(() => robotScene.clone(), [robotScene]);

  return (
    <group
      position={[pos.x, pos.y, pos.z]}
      rotation={[0, agent.rz, 0]}
    >
      <Ring
        args={[0.35, 0.45, 32]}
        position={[0, 0.01, 0]}
        rotation={[-Math.PI / 2, 0, 0]}
      >
        <meshBasicMaterial color={color} />
      </Ring>
      <primitive
        object={robotClone}
        position={[0, 0.07, 0]}
        rotation={[0, ROBOT_YAW_OFFSET, 0]}
        scale={2}
      />
    </group>
  );
}

function RobotAgentsContent({
  agents,
  agentStates,
  mapData,
}: {
  agents: TickAgent[];
  agentStates: Record<number, AgentState>;
  mapData: SceneMapData;
}) {
  const { scene: robotScene } = useGLTF("/robot-final.gltf");

  return (
    <group>
      {agents.map((agent, i) => {
        const agentId = agent.id ?? i;
        const state = agentStates[agentId];
        const status = state?.status || "unknown";
        return (
          <RobotAgent
            key={agentId}
            agent={agent}
            status={status}
            mapData={mapData}
            robotScene={robotScene}
          />
        );
      })}
    </group>
  );
}

// ─── Agents Layer ───────────────────────────────────────────────────────────

function Agents() {
  const frames = useAtomValue(framesAtom);
  const currentFrame = useAtomValue(currentFrameAtom);
  const agentStates = useAtomValue(agentStatesAtom);
  const mapData = useAtomValue(mapDataAtom);

  if (!mapData) return null;
  const agents = frames[currentFrame]?.agents || [];

  return (
    <Suspense fallback={null}>
      <RobotAgentsContent
        agents={agents}
        agentStates={agentStates}
        mapData={mapData}
      />
    </Suspense>
  );
}

// ─── Playback Controller ────────────────────────────────────────────────────

function PlaybackController() {
  const frames = useAtomValue(framesAtom);
  const playing = useAtomValue(playingAtom);
  const speed = useAtomValue(speedAtom);
  const setCurrentFrame = useSetAtom(currentFrameAtom);
  const accRef = useRef(0);
  const frameRef = useRef(0);

  useFrame((_, delta) => {
    if (!playing || frames.length === 0) return;

    accRef.current += delta * speed * 10; // 10 ticks per second base
    const steps = Math.floor(accRef.current);
    accRef.current -= steps;

    if (steps > 0) {
      frameRef.current = Math.min(
        frameRef.current + steps,
        frames.length - 1
      );
      setCurrentFrame(frameRef.current);
    }
  });

  return null;
}

// ─── Camera ─────────────────────────────────────────────────────────────────

function SceneCamera() {
  const mapData = useAtomValue(mapDataAtom);
  if (!mapData) return null;

  const centerX = 0;
  const centerZ = 0;
  const dist = Math.max(mapData.size.width, mapData.size.height) * 0.8;

  return (
    <OrbitControls
      target={[centerX, 0, centerZ]}
      maxPolarAngle={Math.PI / 2.2}
      minDistance={2}
      maxDistance={dist * 2}
    />
  );
}

// ─── Main Scene ─────────────────────────────────────────────────────────────

export default function Scene() {
  const mapData = useAtomValue(mapDataAtom);
  const width = mapData?.size.width ?? 0;
  const height = mapData?.size.height ?? 0;

  return (
    <Canvas
      camera={{
        position: [0, 20, 20],
        fov: 45,
        near: 0.1,
        far: 1000,
      }}
      style={{ width: "100vw", height: "100vh", background: "#0a0a14" }}
    >
      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 20, 10]} intensity={0.8} />
      <SceneCamera />
      <DomainBase />
      {mapData && (
        <GridOverlay
          key={`grid-${width}-${height}`}
          width={width}
          height={height}
        />
      )}
      {mapData && (
        <GridIntersectionCrosses
          key={`cross-${width}-${height}`}
          width={width}
          height={height}
        />
      )}
      <Obstacles />
      <Agents />
      <PlaybackController />
    </Canvas>
  );
}
