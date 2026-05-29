import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { SessionProvider } from "./store/SessionContext";
import { App } from "./App";
import "./styles.css";

const container = document.getElementById("root");
if (!container) throw new Error("root element missing");

createRoot(container).render(
  <StrictMode>
    <SessionProvider>
      <App />
    </SessionProvider>
  </StrictMode>,
);
