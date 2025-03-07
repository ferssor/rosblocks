import { app, BrowserWindow } from "electron";
import path from "path";
import { isDev } from "./util.js";

const URL = 'http://localhost:1337'

app.on("ready", () => {
  const mainWindow = new BrowserWindow({});
  if (isDev()) {
    mainWindow.loadURL(URL)
  } else {
    mainWindow.loadFile(path.join(app.getAppPath(), "/dist-react/index.html"));
  }
});
