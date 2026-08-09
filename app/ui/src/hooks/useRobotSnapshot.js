import { useEffect, useState } from "react";
import { createRobotStateSocket, fetchRobotSnapshot, getRobotSnapshot } from "../services/robotApi.js";

function useRobotSnapshot() {
  const [robot, setRobot] = useState(() => getRobotSnapshot());
  const [source, setSource] = useState("mock");
  const [error, setError] = useState(null);

  useEffect(() => {
    let isMounted = true;
    let socket;

    fetchRobotSnapshot()
      .then((snapshot) => {
        if (!isMounted) return;
        setRobot(snapshot);
        setSource("backend");
        setError(null);
      })
      .catch((err) => {
        if (!isMounted) return;
        setSource("mock");
        setError(err.message);
      });

    try {
      socket = createRobotStateSocket();
      socket.onmessage = (event) => {
        if (!isMounted) return;
        setRobot(JSON.parse(event.data));
        setSource("backend");
        setError(null);
      };
      socket.onerror = () => {
        if (!isMounted) return;
        setSource("mock");
      };
    } catch (err) {
      setError(err.message);
    }

    return () => {
      isMounted = false;
      if (socket) {
        socket.close();
      }
    };
  }, []);

  return { robot, setRobot, source, error };
}

export default useRobotSnapshot;
