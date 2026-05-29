import { defineConfig, type Plugin } from "vite";
import react from "@vitejs/plugin-react";

// Electron loads the build over file:// (origin "null"), where the crossorigin
// attribute Vite adds to module scripts triggers a CORS failure and the bundle
// silently fails to load. Strip it so the SPA loads from disk.
function stripCrossorigin(): Plugin {
  return {
    name: "strip-crossorigin",
    transformIndexHtml(html) {
      return html.replace(/ crossorigin/g, "");
    },
  };
}

// base: './' so the production build can be loaded over file:// by Electron.
export default defineConfig({
  base: "./",
  plugins: [react(), stripCrossorigin()],
  build: {
    outDir: "dist",
    emptyOutDir: true,
    target: "es2022",
  },
});
