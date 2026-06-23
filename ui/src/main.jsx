import React, { useState } from "react";
import { createRoot } from "react-dom/client";
import { Activity, Camera, Home, Map, Settings, Server } from "lucide-react";
import AppLayout from "./layout/AppLayout.jsx";
import HomePage from "./pages/HomePage.jsx";
import DeviceStatusPage from "./pages/DeviceStatusPage.jsx";
import MapsPage from "./pages/MapsPage.jsx";
import SettingsPage from "./pages/SettingsPage.jsx";
import VisualizationPage from "./pages/VisualizationPage.jsx";
import useRobotSnapshot from "./hooks/useRobotSnapshot.js";
import "./styles/index.css";

const tabs = [
  { id: "home", label: "Home", icon: Home },
  { id: "status", label: "Device Status", icon: Activity },
  { id: "maps", label: "Maps", icon: Map },
  { id: "visualization", label: "Visualization", icon: Camera },
  { id: "settings", label: "Settings", icon: Settings },
  { id: "backend", label: "Backend", icon: Server, hidden: true }
];

function App() {
  const [activeTab, setActiveTab] = useState("home");
  const robotState = useRobotSnapshot();

  const pages = {
    home: <HomePage {...robotState} />,
    status: <DeviceStatusPage robot={robotState.robot} />,
    maps: <MapsPage robot={robotState.robot} />,
    visualization: <VisualizationPage robot={robotState.robot} />,
    settings: <SettingsPage robot={robotState.robot} />
  };

  return (
    <AppLayout tabs={tabs.filter((tab) => !tab.hidden)} activeTab={activeTab} onTabChange={setActiveTab}>
      {pages[activeTab]}
    </AppLayout>
  );
}

createRoot(document.getElementById("root")).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
