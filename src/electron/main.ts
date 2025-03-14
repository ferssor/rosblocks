import { app, BrowserWindow, dialog, ipcMain } from "electron";
import path from "path";
import { isDev } from "./util.js";
import { getPreloadPath } from "./pathResolve.js";
import { exec } from "child_process";
import fs from 'fs'

const URL = 'http://localhost:1337'

app.on("ready", () => {
  const mainWindow = new BrowserWindow({
    webPreferences: {
      preload: getPreloadPath() 
    }
  });
  if (isDev()) {
    mainWindow.loadURL(URL)
  } else {
    mainWindow.loadFile(path.join(app.getAppPath(), "/dist-react/index.html"));
  }
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit()
})

ipcMain.handle('open-dialog', async () => {
  const result = await dialog.showOpenDialog({
    properties: ['openDirectory']
  });
  return result.filePaths[0] || '';
});

ipcMain.handle('create-workspace', async (_, workspacePath) => {
  try {
    if (!fs.existsSync(workspacePath)) {
      fs.mkdirSync(workspacePath, { recursive: true });
    }

    return new Promise((resolve, reject) => {
      exec(`cd ${workspacePath} && colcon build`, (error, stdout, stderr) => {
        if (error) {
          console.error(`Erro ao rodar colcon build: ${stderr}`);
          reject(stderr);
        } else {
          console.log(`Build concluído: ${stdout}`);
          resolve({ created: true});
        }
      });
    });
  } catch (error) {
    if (error) {
      console.error('Erro ao criar pasta:', error);
    }
  }
});
