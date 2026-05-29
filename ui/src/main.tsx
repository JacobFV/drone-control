import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { StationProvider } from "./store/StationContext";
import { App } from "./App";
import "./styles.css";

const container = document.getElementById("root");
if (!container) throw new Error("root element missing");

createRoot(container).render(
  <StrictMode>
    <StationProvider>
      <App />
    </StationProvider>
  </StrictMode>,
);
