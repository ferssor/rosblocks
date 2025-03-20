import { app, BrowserWindow, dialog, ipcMain } from "electron";
import path from "path";
import { isDev } from "./util.js";
import { getPreloadPath } from "./pathResolve.js";
import { exec } from "child_process";
import fs from "fs";

const URL = "http://localhost:1337";

app.on("ready", () => {
  const mainWindow = new BrowserWindow({
    webPreferences: {
      preload: getPreloadPath(),
    },
  });
  if (isDev()) {
    mainWindow.loadURL(URL);
  } else {
    mainWindow.loadFile(path.join(app.getAppPath(), "/dist-react/index.html"));
  }
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});

ipcMain.handle("open-dialog", async () => {
  const result = await dialog.showOpenDialog({
    properties: ["openDirectory"],
  });
  return result.filePaths[0] || "";
});

ipcMain.handle("create-workspace", async (_, workspacePath) => {
  try {
    if (!fs.existsSync(workspacePath)) {
      const srcPath = path.join(workspacePath, "src");
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
          resolve({ created: true, workspacePath: workspacePath });
        }
      });
    });
  } catch (error) {
    if (error instanceof Error) {
      console.error("Erro ao criar pasta:", error);
      return { created: false, error: error.message };
    }
  }
});

ipcMain.handle("validate-workspace", (_, workspacePath) => {
  try {
    const requiredDirs = ["src", "install", "log", "build"];
    const missingDirs = requiredDirs.filter(
      (dir) => !fs.existsSync(path.join(workspacePath, dir))
    );
    const result = missingDirs.length > 0 ? false : true;

    return { valid: result };
  } catch (error) {
    if (error instanceof Error) {
      console.error("Erro ao validar o workspace", error);
    }
  }
});

ipcMain.handle("get-packages", async (_, workspacePath) => {
  try {
    const srcPath = path.join(workspacePath, "src");
    if (!fs.existsSync(srcPath)) {
      console.error(`Path does not exist:', ${srcPath}`);
      return { error: "Source path does not exist!" };
    }
    const entries = fs.readdirSync(srcPath, { withFileTypes: true });
    const packages = entries
      .filter((entry) => entry.isDirectory() && !entry.name.startsWith("."))
      .map((folder) => {
        const folderPath = path.join(srcPath, folder.name);
        const stats = fs.statSync(folderPath);
        const numberOfItems = fs.readdirSync(folderPath).length;
        const totalSize = fs.readdirSync(folderPath).reduce((size, file) => {
          const filePath = path.join(folderPath, file);
          return (
            size +
            (fs.statSync(filePath).isFile() ? fs.statSync(filePath).size : 0)
          );
        }, 0);

        let packageType: string = "unknown";
        switch (true) {
          case fs.existsSync(path.join(folderPath, "setup.py")):
            packageType = "python";
            break;
          case fs.existsSync(path.join(folderPath, "CMakeLists.txt")):
            packageType = "cpp";
            break;
          default:
            packageType = "unknown";
        }

        return {
          name: folder.name,
          fullPath: folderPath,
          modifiedAt: stats.mtime.toISOString(),
          createdAt: stats.birthtime.toISOString(),
          numberOfItems,
          totalSize,
          packageType,
        };
      });
    return packages ?? [];
  } catch (error) {
    if (error instanceof Error) {
      console.error("Erro ao obter pacotes:", error);
      return { error: error.message };
    }
  }
});

ipcMain.handle(
  "create-package",
  async (_, workspacePath: string, packageName: string, dependency: string) => {
    if (!fs.existsSync(workspacePath)) {
      return { created: false, error: "Workspace path does not exist." };
    }
    if (!/^[a-zA-Z0-9_-]+$/.test(packageName)) {
      return { created: false, error: "Invalid package name." };
    }

    const typeOfPackage = dependency.includes("py")
      ? "ament_python"
      : "ament_cmake";

    try {
      const command = `cd ${path.join(
        workspacePath,
        "src"
      )} && ros2 pkg create ${packageName} --build-type ${typeOfPackage} --dependencies ${dependency}`;

      return new Promise((resolve, reject) => {
        exec(command, { cwd: workspacePath }, (error, stdout, stderr) => {
          if (error) {
            console.log(`Failed to create package, ${stderr.trim()}`);
            reject({ created: false, error: stderr.trim() });
          } else {
            const packagePath = path.join(workspacePath, "src", packageName);
            console.log(`Package created successfully:  ${stdout}`);
            resolve({ created: true, packagePath: packagePath });
          }
        });
      });
    } catch (error) {
      if (error instanceof Error) {
        console.error("Error while creating the package:", error);
        return { created: false, error: error.message };
      }
      return { created: false, error: "An unknown error occurred." };
    }
  }
);
