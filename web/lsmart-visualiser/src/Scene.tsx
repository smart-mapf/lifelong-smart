import { Suspense, useEffect, useMemo, useRef } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls, useGLTF } from "@react-three/drei";
import { useAtomValue, useSetAtom } from "jotai";
import {
  mapDataAtom,
  framesAtom,
  currentFrameAtom,
  playingAtom,
  speedAtom,
  statsAtom,
  goalArrivalsAtom,
  metaAtom,
} from "./state";
import type { TickAgent } from "./state";
import {
  DataTexture,
  NearestFilter,
  NeutralToneMapping,
  RepeatWrapping,
  type DirectionalLight,
  type Group,
  type MeshStandardMaterial,
} from "three";

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

function addObstacleGrain(
  shader: Parameters<MeshStandardMaterial["onBeforeCompile"]>[0]
) {
  shader.vertexShader =
    "varying vec3 vObstaclePosition;\n" +
    shader.vertexShader.replace(
      "#include <begin_vertex>",
      `#include <begin_vertex>
       vObstaclePosition = (modelMatrix * vec4(transformed, 1.0)).xyz;`
    );

  shader.fragmentShader =
    `varying vec3 vObstaclePosition;
     float obstacleNoise(vec3 p) {
       return fract(sin(dot(p, vec3(127.1, 311.7, 74.7))) * 43758.5453);
     }
    ` +
    shader.fragmentShader
      .replace(
        "#include <map_fragment>",
        `#include <map_fragment>
         float obstacleGrain =
           obstacleNoise(floor(vObstaclePosition * 64.0));
         diffuseColor.rgb *= 0.8 + obstacleGrain * 0.1;`
      )
      .replace(
        "#include <roughnessmap_fragment>",
        `#include <roughnessmap_fragment>
         roughnessFactor *= 0.75 + obstacleNoise(
           floor(vObstaclePosition * 64.0) + 17.0
         ) * 0.25;`
      );
}

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
            castShadow
          >
            <boxGeometry args={[item.width, 0.5, item.height]} />
            <meshStandardMaterial
              color="#cccccc"
              roughness={0.8}
              onBeforeCompile={addObstacleGrain}
            />
          </mesh>
        );
      })}
    </group>
  );
}

// ─── Domain Base ────────────────────────────────────────────────────────────

function createFloorTexture(width: number, height: number) {
  const size = 64;
  const crossArmLength = 5;
  const crossThickness = 1;
  const intersectionGap = 2;
  const pixels = new Uint8Array(size * size * 4);

  for (let y = 0; y < size; y += 1) {
    for (let x = 0; x < size; x += 1) {
      let hash = Math.imul(x + 1, 374761393) ^ Math.imul(y + 1, 668265263);
      hash = Math.imul(hash ^ (hash >>> 13), 1274126177);
      const random = (hash ^ (hash >>> 16)) >>> 0;
      const edgeX = Math.min(x, size - x);
      const edgeY = Math.min(y, size - y);
      const horizontalLine = y === 0 || y === size - 1;
      const verticalLine = x === 0 || x === size - 1;
      const cross =
        (edgeY <= crossThickness && edgeX <= crossArmLength) ||
        (edgeX <= crossThickness && edgeY <= crossArmLength);
      const line =
        (horizontalLine &&
          edgeX > crossArmLength + intersectionGap) ||
        (verticalLine &&
          edgeY > crossArmLength + intersectionGap);
      const value = (cross || line ? 95 : 50) + (random % 11);
      const offset = (y * size + x) * 4;

      pixels[offset] = value;
      pixels[offset + 1] = value + 5;
      pixels[offset + 2] = value + 20;
      pixels[offset + 3] = 255;
    }
  }

  const texture = new DataTexture(pixels, size, size);
  texture.wrapS = RepeatWrapping;
  texture.wrapT = RepeatWrapping;
  texture.minFilter = NearestFilter;
  texture.magFilter = NearestFilter;
  texture.repeat.set(width * 2, height * 2);
  texture.needsUpdate = true;
  return texture;
}

function DomainBase() {
  const mapData = useAtomValue(mapDataAtom);
  const texture = useMemo(
    () =>
      mapData
        ? createFloorTexture(mapData.size.width, mapData.size.height)
        : null,
    [mapData]
  );

  useEffect(() => () => texture?.dispose(), [texture]);

  if (!mapData || !texture) return null;

  return (
    <mesh
      rotation={[-Math.PI / 2, 0, 0]}
      position={[0, -0.01, 0]}
      receiveShadow
    >
      <planeGeometry
        args={[mapData.size.width, mapData.size.height]}
      />
      <meshStandardMaterial color="#cccccc" map={texture} />
    </mesh>
  );
}

function GoalHighlights() {
  const mapData = useAtomValue(mapDataAtom);
  const frames = useAtomValue(framesAtom);
  const currentFrame = useAtomValue(currentFrameAtom);
  const arrivals = useAtomValue(goalArrivalsAtom);
  const ticksPerSecond = useAtomValue(metaAtom)?.ticks_per_second ?? 10;

  if (!mapData) return null;
  const clock = frames[currentFrame]?.clock;
  if (clock === undefined) return null;

  const visibleCells = new Map<string, { col: number; row: number }>();
  for (const arrival of arrivals) {
    if (arrival.clock <= clock && clock < arrival.clock + ticksPerSecond) {
      visibleCells.set(`${arrival.col},${arrival.row}`, arrival);
    }
  }

  return (
    <group>
      {[...visibleCells.values()].map(({ col, row }) => {
        const pos = gridCellToScenePosition(col, row, mapData);
        return (
          <mesh
            key={`${col},${row}`}
            rotation={[-Math.PI / 2, 0, 0]}
            position={[pos.x, 0.005, pos.z]}
          >
            <planeGeometry args={[1, 1]} />
            <meshBasicMaterial
              color="#ff0000"
              transparent
              opacity={0.25}
              depthWrite={false}
            />
          </mesh>
        );
      })}
    </group>
  );
}

// ─── Agent Meshes ───────────────────────────────────────────────────────────

const ROBOT_YAW_OFFSET = -Math.PI / 2;

function RobotAgent({
  agent,
  mapData,
  robotScene,
}: {
  agent: TickAgent;
  mapData: SceneMapData;
  robotScene: Group;
}) {
  const pos = argosToScenePosition(agent, mapData);
  const robotClone = useMemo(() => robotScene.clone(), [robotScene]);

  return (
    <group
      position={[pos.x, pos.y, pos.z]}
      rotation={[0, agent.rz, 0]}
    >
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
  mapData,
}: {
  agents: TickAgent[];
  mapData: SceneMapData;
}) {
  const { scene: robotScene } = useGLTF("/robot-final.gltf");

  return (
    <group>
      {agents.map((agent, i) => {
        const agentId = agent.id ?? i;
        return (
          <RobotAgent
            key={agentId}
            agent={agent}
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
  const mapData = useAtomValue(mapDataAtom);

  if (!mapData) return null;
  const agents = frames[currentFrame]?.agents || [];

  return (
    <Suspense fallback={null}>
      <RobotAgentsContent
        agents={agents}
        mapData={mapData}
      />
    </Suspense>
  );
}

// ─── Playback Controller ────────────────────────────────────────────────────

function PlaybackController() {
  const frames = useAtomValue(framesAtom);
  const currentFrame = useAtomValue(currentFrameAtom);
  const playing = useAtomValue(playingAtom);
  const speed = useAtomValue(speedAtom);
  const stats = useAtomValue(statsAtom);
  const setCurrentFrame = useSetAtom(currentFrameAtom);
  const setPlaying = useSetAtom(playingAtom);
  const accRef = useRef(0);

  useFrame((_, delta) => {
    if (!playing || frames.length === 0) return;
    if (stats && currentFrame >= frames.length - 1) {
      setPlaying(false);
      return;
    }

    accRef.current += delta * speed * 10; // 10 ticks per second base
    const steps = Math.floor(accRef.current);
    accRef.current -= steps;

    if (steps > 0) {
      const nextFrame = Math.min(currentFrame + steps, frames.length - 1);
      setCurrentFrame(nextFrame);
      if (stats && nextFrame >= frames.length - 1) setPlaying(false);
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

function SceneLights() {
  const light = useRef<DirectionalLight | null>(null);
  const scene = useThree((state) => state.scene);

  useEffect(() => {
    const target = light.current?.target;
    if (!target) return;
    scene.add(target);
    return () => {
      scene.remove(target);
    };
  }, [scene]);

  useFrame(({ camera }) => {
    if (!light.current) return;

    const x = Math.floor(camera.position.x);
    const z = Math.floor(camera.position.z);
    light.current.position.set(x + 100, 100, z + 50);
    light.current.target.position.set(x, 0, z);

    const extent = Math.floor(
      Math.min(10_000, Math.max(3, camera.position.y)) * 3
    );
    const shadowCamera = light.current.shadow.camera;
    shadowCamera.left = -extent;
    shadowCamera.right = extent;
    shadowCamera.top = -extent;
    shadowCamera.bottom = extent;
    shadowCamera.updateProjectionMatrix();
  });

  return (
    <>
      <ambientLight intensity={Math.PI / 2} color="#ade5ff" />
      <directionalLight
        ref={light}
        color="#ffefd9"
        castShadow
        shadow-bias={-0.0001}
        shadow-mapSize={[3072, 3072]}
        intensity={5}
      />
    </>
  );
}

// ─── Main Scene ─────────────────────────────────────────────────────────────

export default function Scene() {
  return (
    <Canvas
      shadows
      dpr={[1, 1.5]}
      gl={{
        antialias: true,
        powerPreference: "high-performance",
        toneMapping: NeutralToneMapping,
      }}
      camera={{
        position: [0, 20, 20],
        fov: 45,
        near: 0.1,
        far: 1000,
      }}
      style={{ width: "100vw", height: "100vh" }}
    >
      <fog attach="fog" args={["#181c20", 20, 100]} />
      <color attach="background" args={["#181c20"]} />
      <SceneLights />
      <SceneCamera />
      <DomainBase />
      <GoalHighlights />
      <Obstacles />
      <Agents />
      <PlaybackController />
    </Canvas>
  );
}
