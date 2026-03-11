import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";
import raw from "vite-plugin-raw";

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    react(),
    raw({
      match: /\.hbs$/,
    }),
  ],
  base: "./",
  define: {
    __APP_VERSION__: JSON.stringify(process.env.npm_package_version ?? "0.0.0"),
  },
  build: {
    outDir: "dist-react",
  },
  server: {
    port: 1337,
    strictPort: true,
  },
});
