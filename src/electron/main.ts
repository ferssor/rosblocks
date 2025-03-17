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
      const srcPath = path.join(workspacePath, 'src');
      fs.mkdirSync(workspacePath, { recursive: true });
      fs.mkdirSync(srcPath, { recursive: true });
    }

    return new Promise((resolve, reject) => {
      exec(`cd ${workspacePath} && colcon build`, (error, stdout, stderr) => {
        if (error) {
          console.error(`Erro ao rodar colcon build: ${stderr}`);
          reject({ created: false, error: stderr });
        } else {
          console.log(`Build concluído: ${stdout}`);
          resolve({ created: true, workspacePath: workspacePath});
        }
      });
    });
  } catch (error) {
    if (error instanceof Error) {
      console.error('Erro ao criar pasta:', error);
      return { created: false, error: error.message };
    }
  }
});

ipcMain.handle('validate-workspace', (_, workspacePath) => {
  try {
    const requiredDirs = ['src', 'install', 'log', 'build']
    const missingDirs = requiredDirs.filter(dir => !fs.existsSync(path.join(workspacePath, dir)));
    const result = missingDirs.length > 0 ? false : true

    return {valid: result}
  } catch (error) {
    if (error instanceof Error) {
      console.error('Erro ao validar o workspace', error)
    }
  }
});
