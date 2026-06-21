import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

const meshParts = [
  {
    name: "base",
    file: "/robot_meshes/base_link.STL",
    color: 0xc9d1ee,
    position: [0, 0, 0.0445],
    rotation: [0, 0, -Math.PI / 2]
  },
  {
    name: "leftWheel",
    file: "/robot_meshes/left_wheel_link.STL",
    color: 0x333333,
    position: [-0.0445, 0.08975, 0.0245],
    rotation: [-Math.PI / 2, 0, 0]
  },
  {
    name: "rightWheel",
    file: "/robot_meshes/right_wheel_link.STL",
    color: 0x333333,
    position: [-0.0445, -0.08975, 0.0245],
    rotation: [Math.PI / 2, 0, 0]
  },
  {
    name: "caster",
    file: "/robot_meshes/caster_wheel_link.STL",
    color: 0xb8c0dd,
    position: [0.05, 0, 0.00422],
    rotation: [Math.PI / 2, 0, Math.PI / 2]
  },
  {
    name: "lidar",
    file: "/robot_meshes/lidar_link.STL",
    color: 0xf8fafc,
    position: [0.00003, 0.000025, 0.24558],
    rotation: [Math.PI / 2, 0, Math.PI]
  }
];

function RobotModelViewer({ pose, motorsReady }) {
  const hostRef = useRef(null);
  const robotRef = useRef(null);
  const wheelsRef = useRef([]);
  const lastPoseRef = useRef({ x: pose?.x ?? 0, y: pose?.y ?? 0, distance: 0 });

  useEffect(() => {
    const host = hostRef.current;
    if (!host) return undefined;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf8fafc);

    const camera = new THREE.PerspectiveCamera(45, host.clientWidth / host.clientHeight, 0.01, 20);
    camera.position.set(0.42, -0.55, 0.36);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(host.clientWidth, host.clientHeight);
    host.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 0, 0.08);
    controls.minDistance = 0.25;
    controls.maxDistance = 2.5;

    scene.add(new THREE.HemisphereLight(0xffffff, 0x7c8798, 2.6));
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
    keyLight.position.set(0.4, -0.6, 0.8);
    scene.add(keyLight);

    const grid = new THREE.GridHelper(0.8, 16, 0x94a3b8, 0xd6dbe5);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);

    const robot = new THREE.Group();
    robotRef.current = robot;
    scene.add(robot);

    const loader = new STLLoader();
    const loadedMeshes = [];

    meshParts.forEach((part) => {
      loader.load(part.file, (geometry) => {
        geometry.computeVertexNormals();
        const material = new THREE.MeshStandardMaterial({
          color: part.color,
          roughness: 0.62,
          metalness: 0.12
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.name = part.name;
        mesh.position.set(...part.position);
        mesh.rotation.set(...part.rotation);
        robot.add(mesh);
        loadedMeshes.push(mesh);
        if (part.name.includes("Wheel")) wheelsRef.current.push(mesh);
      });
    });

    const resizeObserver = new ResizeObserver(() => {
      if (!host.clientWidth || !host.clientHeight) return;
      camera.aspect = host.clientWidth / host.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(host.clientWidth, host.clientHeight);
    });
    resizeObserver.observe(host);

    let animationFrame;
    function animate() {
      controls.update();
      renderer.render(scene, camera);
      animationFrame = requestAnimationFrame(animate);
    }
    animate();

    return () => {
      cancelAnimationFrame(animationFrame);
      resizeObserver.disconnect();
      controls.dispose();
      loadedMeshes.forEach((mesh) => {
        mesh.geometry.dispose();
        mesh.material.dispose();
      });
      renderer.dispose();
      if (renderer.domElement.parentNode === host) {
        host.removeChild(renderer.domElement);
      }
    };
  }, []);

  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) return;

    const yaw = Number(pose?.yaw ?? 0);
    robot.rotation.z = THREE.MathUtils.degToRad(yaw);

    const nextX = Number(pose?.x ?? 0);
    const nextY = Number(pose?.y ?? 0);
    const last = lastPoseRef.current;
    const distance = Math.hypot(nextX - last.x, nextY - last.y);
    const direction = distance > 0 ? 1 : 0;
    last.distance += direction * distance / 0.035;
    last.x = nextX;
    last.y = nextY;
    wheelsRef.current.forEach((wheel) => {
      wheel.rotation.z = last.distance;
    });
  }, [pose]);

  return (
    <div className="rounded-md border border-console-line bg-white p-4 shadow-soft">
      <div className="mb-3 flex items-center justify-between">
        <div>
          <h2 className="text-lg font-semibold tracking-normal">Robot model</h2>
          <div className="text-sm text-slate-500">URDF mesh view</div>
        </div>
        <span className="inline-flex items-center gap-2 rounded-md border border-console-line bg-console-panel px-2 py-1 text-xs font-semibold text-slate-600">
          <span className={`h-2.5 w-2.5 rounded-full ${motorsReady ? "bg-emerald-500" : "bg-amber-400"}`} aria-hidden="true" />
          {motorsReady ? "Live feedback" : "Waiting"}
        </span>
      </div>
      <div ref={hostRef} className="h-[360px] w-full overflow-hidden rounded-md border border-console-line bg-[#f8fafc]" />
    </div>
  );
}

export default RobotModelViewer;
