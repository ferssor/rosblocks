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
  return {
    workspaceLocation: result.filePaths[0] || "",
    canceled: result.canceled,
  };
});

ipcMain.handle("create-workspace", async (_, workspacePath) => {
  try {
    if (fs.existsSync(workspacePath)) {
      console.error(`Workspace already exists: ${workspacePath}`);
      return { created: false, error: "Workspace name already exists." };
    }

    const srcPath = path.join(workspacePath, "src");
    fs.mkdirSync(workspacePath, { recursive: true });
    fs.mkdirSync(srcPath, { recursive: true });

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
  return { created: false, error: "An unknown error occurred." };
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

    const srcPath = path.join(workspacePath, "src");
    const packagePath = path.join(srcPath, packageName);

    if (fs.existsSync(packagePath)) {
      console.error(`Package already exists: ${packagePath}`);
      return {
        created: false,
        error: "Package name already exists in the workspace.",
      };
    }

    const typeOfPackage = dependency.includes("py")
      ? "ament_python"
      : "ament_cmake";

    try {
      const command = `cd ${srcPath} && ros2 pkg create ${packageName} --build-type ${typeOfPackage} --dependencies ${dependency}`;

      return new Promise((resolve, reject) => {
        exec(command, { cwd: workspacePath }, (error, stdout, stderr) => {
          if (error) {
            console.log(`Failed to create package, ${stderr.trim()}`);
            reject({ created: false, error: stderr.trim() });
          } else {
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

ipcMain.handle(
  "get-nodes",
  async (_, packagePath: string, packageName: string) => {
    const pkgPath = path.join(packagePath, packageName);
    if (!pkgPath) {
      console.error("Package path does not exist!");
      return [];
    }

    const nodes = fs
      .readdirSync(pkgPath)
      .filter((node) => node.endsWith(".py") && node !== "__init__.py");

    return nodes.map((node) => {
      const fullPath = path.join(pkgPath, node);
      const relativePath = path.relative(packagePath, fullPath);

      return {
        name: path.basename(node, ".py"),
        fullPath,
        relativePath,
        content: fs.readFileSync(fullPath, "utf-8"),
      };
    });
  }
);

ipcMain.handle(
  "create-node",
  async (
    _,
    nodeName: string,
    nodeType: string,
    packagePath: string,
    packageName: string
  ) => {
    const pkgPath = path.join(packagePath, packageName);
    const extension = nodeType === "python" ? ".py" : ".cpp";
    console.log(extension);

    if (!pkgPath) {
      console.error("Package path does not exist!");
      return { created: false, error: "Package path does not exist." };
    }

    if (!/^[a-zA-Z0-9_-]+$/.test(nodeName)) {
      return { created: false, error: "Invalid node name." };
    }

    const filePath = path.join(pkgPath, `${nodeName}${extension}`);

    if (fs.existsSync(filePath)) {
      console.error(`Node file already exists: ${filePath}`);
      return {
        created: false,
        error: "Node name already exists in the package.",
      };
    }

    try {
      fs.writeFileSync(filePath, "", { mode: 0o755 });
      console.log(`Node created successfully at: ${filePath}`);
      return { created: true, pkgPath };
    } catch (error) {
      if (error instanceof Error) {
        return { created: false, error: "An unknown error occurred." };
      }
    }
  }
);

ipcMain.handle(
  "import-package",
  async (_, url: string, workspacePath: string) => {
    if (!url || !workspacePath) {
      return { imported: false, error: "URL or workspace path is missing." };
    }

    const srcPath = path.join(workspacePath, "src");

    if (!fs.existsSync(srcPath)) {
      return {
        imported: false,
        error: "Workspace source path does not exist.",
      };
    }

    try {
      const command = `git clone ${url}`;
      return new Promise((resolve, reject) => {
        exec(command, { cwd: srcPath }, (error, stdout, stderr) => {
          if (error) {
            console.error(`Failed to clone package: ${stderr.trim()}`);
            reject({ imported: false, error: stderr.trim() });
          } else {
            console.log(`Package cloned successfully: ${stdout.trim()}`);
            resolve({ imported: true });
          }
        });
      });
    } catch (error) {
      if (error instanceof Error) {
        console.error("Error while importing the package:", error);
        return { imported: false, error: error.message };
      }
      return { imported: false, error: "An unknown error occurred." };
    }
  }
);

ipcMain.handle("delete-package", async (_, path: string) => {
  if (!path) {
    return { deleted: false, error: "URL or workspace path is missing." };
  }

  if (!fs.existsSync(path)) {
    return {
      deleted: false,
      error: "Workspace source path does not exist.",
    };
  }

  try {
    await fs.promises.rm(path, { recursive: true, force: true });
    return { deleted: true };
  } catch (error) {
    if (error instanceof Error) {
      console.error("Error while deleting the package:", error);
      return { deleted: false, error: error.message };
    }
    return { deleted: false, error: "An unknown error occurred." };
  }
});

ipcMain.handle("get-interfaces", async () => {
  try {
    const interfaces = await new Promise<string[]>((resolve, reject) => {
      exec(
        "ros2 interface list",
        (error: Error | null, stdout: string, stderr: string) => {
          if (error) {
            console.error("Failed to fetch ROS2 interfaces:", stderr);
            reject(stderr);
          } else {
            resolve(stdout.split("\n").filter((line) => line.trim() !== ""));
          }
        }
      );
    });

    const formattedInterfaces = interfaces
      .filter((interfacePath) => {
        const parts = interfacePath.split("/");
        return parts[parts.length - 2] === "msg";
      })
      .map((interfacePath) => {
        const parts = interfacePath.split("/");
        const interfaceName = parts[parts.length - 1];
        const modulePath = parts.slice(0, -1).join(".").trim();
        return {
          name: interfaceName,
          location: `from ${modulePath} import ${interfaceName}`,
        };
      });

    return formattedInterfaces;
  } catch (error) {
    console.error("Error while fetching ROS2 interfaces:", error);
    return [];
  }
});

ipcMain.handle(
  "create-blocks",
  async (
    _,
    interfaceName,
    scriptName,
    relativePath: string,
    nodePath,
    blocks
  ) => {
    console.log({ interfaceName, scriptName, relativePath, nodePath, blocks });
    try {
      if (!nodePath && relativePath) {
        return {
          created: false,
          error: "The path is required!",
        };
      }

      if (!interfaceName && scriptName) {
        return {
          created: false,
          error: "The interface and script name is required!",
        };
      }

      const path = nodePath.replace(relativePath, "");

      // Update package.xml
      const packageXmlPath = `${path}/package.xml`;
      if (!fs.existsSync(packageXmlPath)) {
        return {
          created: false,
          error: "package.xml file does not exist in the specified path.",
        };
      }

      const packageXmlContent = fs.readFileSync(packageXmlPath, "utf-8");
      const updatedPackageXmlContent = packageXmlContent.replace(
        /<depend>(.*?)<\/depend>/,
        (match) => `${match}\n  <depend>${interfaceName}</depend>`
      );

      fs.writeFileSync(packageXmlPath, updatedPackageXmlContent, "utf-8");
      console.log(`Updated package.xml with <depend>${interfaceName}</depend>`);

      // Update setup.py
      const setupPyPath = `${path}/setup.py`;
      if (!fs.existsSync(setupPyPath)) {
        return {
          created: false,
          error: "setup.py file does not exist in the specified path.",
        };
      }

      const setupPyContent = fs.readFileSync(setupPyPath, "utf-8");
      const relativePathFormatted = relativePath
        .replace(/\//g, ".")
        .replace(/\.py$/, "");
      const newScriptLine = `"${scriptName} = ${relativePathFormatted}:main",`;

      const updatedSetupPyContent = setupPyContent.replace(
        /('console_scripts':\s*\[)([\s\S]*?)(\])/,
        (match, start, middle, end) => {
          const trimmedMiddle = middle.trim();
          const updatedMiddle = trimmedMiddle
            ? `${trimmedMiddle},\n      ${newScriptLine}`
            : `      ${newScriptLine}`;
          return `${start}${updatedMiddle}\n  ${end}`;
        }
      );

      fs.writeFileSync(setupPyPath, updatedSetupPyContent, "utf-8");
      console.log(`Updated setup.py with console script: ${newScriptLine}`);

      // Write blocks on the file at nodePath
      if (!fs.existsSync(nodePath)) {
        return {
          created: false,
          error: "The file at nodePath does not exist.",
        };
      }

      fs.writeFileSync(nodePath, blocks, "utf-8");
      console.log(`Written blocks content to file at: ${nodePath}`);

      return { created: true };
    } catch (error) {
      console.error("Error while creating the blocks", error);
      return {
        created: false,
        error:
          error instanceof Error ? error.message : "An unknown error occurred.",
      };
    }
  }
);
