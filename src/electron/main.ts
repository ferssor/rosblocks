import { app, BrowserWindow, dialog, ipcMain } from "electron";
import path from "path";
import { isDev } from "./util.js";
import { getPreloadPath } from "./pathResolve.js";
import { exec } from "child_process";
import fs from "fs";

const URL = "http://localhost:1337";

function getShellType() {
  const shellPath = process.env.SHELL || process.env.ComSpec || "";
  let shellType = "";
  switch (true) {
    case shellPath.includes("zsh"):
      shellType = "zsh";
      break;
    case shellPath.includes("bash"):
      shellType = "bash";
      break;
    case shellPath.includes(".sh"):
      shellType = "sh";
      break;
    case shellPath.toLowerCase().includes("powershell") ||
      !!process.env.PSModulePath:
      shellType = "ps1";
      break;
    default:
      shellType = "unknown";
      break;
  }
  return shellType;
}

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
          console.log(`[BUILD]: Build concluído: ${stdout}`);
          const setupFile = `${workspacePath}/install/setup.${getShellType()}`;
          if (fs.existsSync(setupFile) && fs.statSync(setupFile).size > 0) {
            exec(
              `${getShellType()} -c "source ${setupFile}"`,
              (error, stdout, stderr) => {
                if (error) {
                  console.error(
                    `Erro ao rodar setup do ${workspacePath}: ${stderr}`
                  );
                } else {
                  console.log(`Build do ${workspacePath} concluído: ${stdout}`);
                }
              }
            );
          } else {
            console.log(
              "[ERROR]: The setup file on install folder doens't exists!"
            );
          }

          exec(
            `${getShellType()} -c "source /opt/ros/jazzy/setup.${getShellType()}"`,
            (error, stdout, stderr) => {
              if (error) {
                console.error(`Erro ao rodar build do ROS: ${stderr}`);
              } else {
                console.log(`Build do ROS concluído: ${stdout}`);
              }
            }
          );
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
      const blockFile = path.join(
        packagePath,
        packageName,
        node.replace(".py", ".blocks")
      );
      const fullPath = path.join(pkgPath, node);
      const relativePath = path.relative(packagePath, fullPath);

      return {
        name: path.basename(node, ".py"),
        fullPath,
        relativePath,
        content: fs.readFileSync(blockFile, "utf-8"),
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

    if (!pkgPath) {
      console.error("Package path does not exist!");
      return { created: false, error: "Package path does not exist." };
    }

    if (!/^[a-zA-Z0-9_-]+$/.test(nodeName)) {
      return { created: false, error: "Invalid node name." };
    }

    const filePath = path.join(pkgPath, `${nodeName}${extension}`);
    const blockFilePath = path.join(pkgPath, `${nodeName}.blocks`);

    if (fs.existsSync(filePath)) {
      console.error(`Node file already exists: ${filePath}`);
      return {
        created: false,
        error: "Node already exists in the package.",
      };
    }

    if (fs.existsSync(blockFilePath)) {
      console.error(`Blocks file already exists: ${blockFilePath}`);
      return {
        created: false,
        error: "Blocks file already exists in the package.",
      };
    }

    try {
      fs.writeFileSync(filePath, "", { mode: 0o755 });
      fs.writeFileSync(blockFilePath, "");
      console.log(`Node created successfully at: ${filePath}`);
      console.log(`Blocks created successfully at: ${blockFilePath}`);
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
  "build-package",
  async (_, packagePath: string, packageName: string) => {
    if (!packagePath && !packageName) {
      console.log("The package path and name is required!");
      return {
        wasBuilded: false,
        error: "The package path and name is required!",
      };
    }

    try {
      if (!fs.existsSync(packagePath)) {
        return { wasBuilded: false, error: "The package name does not exist." };
      }

      const workspacePath = packagePath.replace(`/src/${packageName}`, "");

      console.log(
        `Building package at path: ${workspacePath} with name: ${packageName}`
      );

      return new Promise((resolve, reject) => {
        exec(
          `cd ${workspacePath} && colcon build --packages-select ${packageName}`,
          (error, stdout, stderr) => {
            if (error) {
              console.error(`Erro ao rodar colcon build: ${stderr}`);
              reject({ wasBuilded: false, error: stderr });
            } else {
              console.log(`Build concluído: ${stdout}`);
              const setupFile = `${workspacePath}/install/setup.${getShellType()}`;

              if (fs.existsSync(setupFile) && fs.statSync(setupFile).size > 0) {
                exec(
                  `${getShellType()} -c "source ${setupFile}"`,
                  (error, stdout, stderr) => {
                    if (error) {
                      console.error(
                        `Erro ao rodar setup do ${packagePath}: ${stderr}`
                      );
                    } else {
                      console.log(
                        `Build do ${workspacePath} concluído: ${stdout}`
                      );
                    }
                  }
                );
              } else {
                console.log(
                  "[ERROR]: The setup file on install folder doens't exists!"
                );
              }

              exec(
                `${getShellType()} -c "source /opt/ros/jazzy/setup.${getShellType()}"`,
                (error, stdout, stderr) => {
                  if (error) {
                    console.error(`Erro ao rodar build do ROS: ${stderr}`);
                  } else {
                    console.log(`Build do ROS concluído: ${stdout}`);
                  }
                }
              );
              resolve({ wasBuilded: true });
            }
          }
        );
      });
    } catch (error) {
      console.error("Error while building the package", error);
      return {
        wasBuilded: false,
        error:
          error instanceof Error ? error.message : "An unknown error occurred.",
      };
    }
  }
);

ipcMain.handle("create-blocks", async (_, nodePath: string, blocks, code) => {
  try {
    if (!nodePath && !blocks && !code) {
      return {
        created: false,
        error: "The path, blocks and code is required!",
      };
    }

    // Write code on the node file at nodePath
    if (!fs.existsSync(nodePath)) {
      return {
        created: false,
        error: "The file at nodePath does not exist.",
      };
    }

    fs.writeFileSync(nodePath, code, "utf-8");
    console.log(`Written code content to node file at: ${nodePath}`);

    // Write blocks on the blocks file at nodePath
    const blockFilePath = nodePath.replace(".py", ".blocks");
    if (!fs.existsSync(blockFilePath)) {
      return {
        created: false,
        error: "The block file at nodePath does not exist.",
      };
    }

    fs.writeFileSync(blockFilePath, blocks, "utf-8");
    console.log(`Written blocks content to file at: ${blockFilePath}`);
    return { created: true };
  } catch (error) {
    console.error("Error while creating the blocks", error);
    return {
      created: false,
      error:
        error instanceof Error ? error.message : "An unknown error occurred.",
    };
  }
});

ipcMain.handle(
  "add-dependency",
  async (_, relativePath: string, nodePath: string, interfaceName: string) => {
    if (!relativePath && !nodePath && !interfaceName) {
      console.log(
        "The package path, script name and interface name is required!"
      );
      return {
        wasAdded: false,
        error: "The package path and name is required!",
      };
    }

    try {
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

      if (!packageXmlContent.includes(`<depend>${interfaceName}</depend>`)) {
        const updatedPackageXmlContent = packageXmlContent.replace(
          /<\/license>/,
          `</license>\n  <depend>${interfaceName}</depend>`
        );

        fs.writeFileSync(packageXmlPath, updatedPackageXmlContent, "utf-8");
        console.log(
          `Updated package.xml with <depend>${interfaceName}</depend>`
        );
      } else {
        console.log(
          `<depend>${interfaceName}</depend> already exists in package.xml`
        );
      }
      return { wasAdded: true };
    } catch (error) {
      console.error("Error while adding the dependencies", error);
      return {
        wasAdded: false,
        error:
          error instanceof Error ? error.message : "An unknown error occurred.",
      };
    }
  }
);

ipcMain.handle(
  "add-script",
  async (_, relativePath: string, nodePath: string, scriptName: string) => {
    if (!relativePath && !nodePath && !scriptName) {
      console.log("The package path, script name is required!");
      return {
        wasAdded: false,
        error: "The package path and name is required!",
      };
    }

    try {
      const path = nodePath.replace(relativePath, "");
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

      if (!setupPyContent.includes(newScriptLine)) {
        const updatedSetupPyContent = setupPyContent
          .replace(/('console_scripts':\s*\[)/, `$1\n      `)
          .replace(
            /('console_scripts':\s*\[)([\s\S]*?)(\])/,
            (match, start, middle, end) => {
              const cleanedMiddle = middle
                .replace(/,+/g, ",")
                .replace(/,\s*$/, "")
                .trim();

              const updatedMiddle = cleanedMiddle
                ? `${cleanedMiddle},\n      ${newScriptLine}`
                : `      ${newScriptLine}`;

              return `${start}${updatedMiddle}\n  ${end}`;
            }
          );

        fs.writeFileSync(setupPyPath, updatedSetupPyContent, "utf-8");
        console.log(`Updated setup.py with console script: ${newScriptLine}`);
      } else {
        console.log(
          `The script line "${newScriptLine}" already exists in setup.py`
        );
      }
      return { wasAdded: true };
    } catch (error) {
      console.error("Error while adding the dependencies", error);
      return {
        wasAdded: false,
        error:
          error instanceof Error ? error.message : "An unknown error occurred.",
      };
    }
  }
);

ipcMain.handle(
  "delete-node",
  async (_, nodeName: string, nodePath: string, packageName: string) => {
    if (!nodeName || !nodePath || !packageName) {
      console.error("Node name, package path and package name are required.");
      return { wasDeleted: false, error: "Required parameters are missing." };
    }

    const blockFilePath = nodePath.replace(".py", ".blocks");

    if (!fs.existsSync(nodePath)) {
      console.error(`Node file does not exist: ${nodePath}`);
      return {
        wasDeleted: false,
        error: "Node does not exist in the package.",
      };
    }

    if (!fs.existsSync(blockFilePath)) {
      console.error(`Blocks file does not exist: ${blockFilePath}`);
      return {
        wasDeleted: false,
        error: "Blocks file does not exist in the package.",
      };
    }

    try {
      fs.unlinkSync(nodePath);
      fs.unlinkSync(blockFilePath);
      console.log(`Node deleted successfully at: ${nodePath}`);
      console.log(`Blocks deleted successfully at: ${blockFilePath}`);
      return { wasDeleted: true };
    } catch (error) {
      if (error instanceof Error) {
        return { wasDeleted: false, error: "An unknown error occurred." };
      }
    }
  }
);

ipcMain.handle(
  "remove-dependency",
  async (
    _,
    relativePath: string,
    nodePath: string,
    scriptName: string,
    interfaceName: string
  ) => {
    if (!relativePath && !nodePath && !scriptName && !interfaceName) {
      console.log(
        "The package path, script name and interface name is required!"
      );
      return {
        wasRemoved: false,
        error: "The package path and name is required!",
      };
    }

    try {
      const path = nodePath.replace(relativePath, "");

      // Update package.xml
      const packageXmlPath = `${path}/package.xml`;
      if (!fs.existsSync(packageXmlPath)) {
        return {
          wasRemoved: false,
          error: "package.xml file does not exist in the specified path.",
        };
      }

      const packageXmlContent = fs.readFileSync(packageXmlPath, "utf-8");

      if (packageXmlContent.includes(`<depend>${interfaceName}</depend>`)) {
        const updatedPackageXmlContent = packageXmlContent
          .replace(`<depend>${interfaceName}</depend>`, "")
          .replace(/\n\s*\n/g, "\n"); // Remove extra empty lines

        fs.writeFileSync(packageXmlPath, updatedPackageXmlContent, "utf-8");
        console.log(
          `Removed <depend>${interfaceName}</depend> from package.xml`
        );
      } else {
        console.log(
          `<depend>${interfaceName}</depend> does not exist in package.xml`
        );
      }

      // Update setup.py
      const setupPyPath = `${path}/setup.py`;
      if (!fs.existsSync(setupPyPath)) {
        return {
          wasRemoved: false,
          error: "setup.py file does not exist in the specified path.",
        };
      }

      const setupPyContent = fs.readFileSync(setupPyPath, "utf-8");
      const relativePathFormatted = relativePath
        .replace(/\//g, ".")
        .replace(/\.py$/, "");
      const scriptLine = `"${scriptName} = ${relativePathFormatted}:main",`;

      if (setupPyContent.includes(scriptLine)) {
        const updatedSetupPyContent = setupPyContent
          .replace(scriptLine, "")
          .replace(/,\s*\n\s*\]/g, "\n  ]") // Fix trailing commas
          .replace(/,\s*,/g, ",") // Fix double commas
          .replace(/\[\s*,/g, "[") // Fix leading commas
          .replace(/\n\s*\n/g, "\n"); // Remove extra empty lines

        fs.writeFileSync(setupPyPath, updatedSetupPyContent, "utf-8");
        console.log(`Removed console script: ${scriptLine} from setup.py`);
      } else {
        console.log(
          `The script line "${scriptLine}" does not exist in setup.py`
        );
      }

      return { wasRemoved: true };
    } catch (error) {
      console.error("Error while removing dependencies", error);
      return {
        wasRemoved: false,
        error:
          error instanceof Error ? error.message : "An unknown error occurred.",
      };
    }
  }
);

ipcMain.handle(
  "execute-node",
  async (_, packageName: string, nodeName: string, packagePath: string) => {
    if (!packageName || !nodeName || !packagePath) {
      return {
        executed: false,
        error: "Package name, node name, and workspace path are required.",
      };
    }

    try {
      const workspacePath = path.join(
        packagePath.replace(`/src/${packageName}`, ""),
        "install"
      );
      const setupScript = path.join(workspacePath, `setup.${getShellType()}`);

      if (!fs.existsSync(setupScript)) {
        return {
          executed: false,
          error: `Setup script not found at ${setupScript}. Make sure the package is built.`,
        };
      }

      // Open a new terminal window and execute the ROS node
      const sourceSetup = `source ${setupScript} && source /opt/ros/jazzy/setup.${getShellType()}`;
      const command = `gnome-terminal -- ${getShellType()} -c "${sourceSetup} && ros2 run ${packageName} ${nodeName}; echo 'Press Enter to close'; read"`;

      exec(command, (error, stdout, stderr) => {
        if (error) {
          console.error(`Error executing node: ${stderr}`);
          return { success: false, error: stderr };
        }
        console.log(`Node execution started: ${stdout}`);
      });

      return { executed: true };
    } catch (error) {
      console.error("Error while executing the node", error);
      return {
        executed: false,
        error:
          error instanceof Error ? error.message : "An unknown error occurred.",
      };
    }
  }
);
