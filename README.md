# ROSBLOCKS

A visual desktop editor to create and manage programming blocks intuitively. ROSBLOCKS allows you to develop programming logic using a block-based interface, with support for automatic Python code generation.

## Introduction

Watch the demo video to understand the main features:

<video width="100%" controls>
  <source src="public/editor.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

> See how to navigate the workspace, select packages, create blocks, and generate Python code automatically.

## Key Features

- **Visual Editor with Blockly**: Intuitive block-based interface for visual logic
- **Workspace Management**: Support for multiple workspaces and packages
- **Python Code Generation**: Convert your blocks into Python code automatically
- **Node Editor**: Detailed editor for configuring each block
- **Desktop App**: Electron application for Windows, macOS, and Linux
- **Monaco Editor**: Integrated code editor to view/edit generated code

## ️Project Structure

```
rosblocks/
├── src/
│   ├── app/                      # Main React application
│   │   └── main.tsx              # React application entry
│   ├── electron/                 # Electron code
│   │   ├── main.ts               # Main process
│   │   ├── preload.cts           # Preload script
│   │   └── util.ts               # Utilities
│   └── types.d.ts
├── public/
│   └── editor.mp4               # Demo video
├── dist-electron/               # Compiled Electron build
├── dist-react/                  # Compiled React build
├── package.json
├── tsconfig.json
├── vite.config.ts               # Vite configuration
└── electron-builder.json        # Packaging configuration
```

## Architecture

### Layered Architecture

```
┌─────────────────────────────────────┐
│    User Interface Layer (React)     │
│ (Editor, SelectPackage, NodeEditor) │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│   Block Logic Processing Layer      │
│ (Blockly, BlockGenerator, NodeMgmt) │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│    Code Generation Layer            │
│      (blocksToPython.ts)            │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│   Electron/System Layer             │
│  (IPC, Filesystem, Processes)       │
└─────────────────────────────────────┘
```

### Data Flow

1. **Workspace Selection**: User selects a valid workspace (with `_ws` suffix)
2. **Package Loading**: System loads available packages from workspace
3. **Block Editing**: User edits blocks in Blockly editor
4. **Code Generation**: Blocks are converted to Python code
5. **Persistence**: Changes are saved to filesystem via Electron IPC

### Main Components

| Component | Responsibility |
|-----------|-----------------|
| **SelectWorkspace** | Allow workspace selection and validation |
| **SelectPackage** | Manage package list and create new packages |
| **NodeManager** | Manage package nodes/blocks |
| **NodeEditor** | Visually edit blocks with Blockly |
| **blocksToPython** | Generate Python code from blocks |
| **EditorHeader** | Top bar with general options |

## Getting Started

### Prerequisites

- Node.js >= 18
- npm or yarn
- Git

### Installation

```bash
# Clone the repository
git clone https://github.com/ferssor/rosblocks.git
cd rosblocks

# Install dependencies
npm install
```

### Development

To run the application in development mode with hot-reload:

```bash
npm run dev
```

This command:
- Starts the Vite server for the React application (`dev:react`)
- Compiles and executes Electron in development mode (`dev:electron`)
- Allows real-time editing with automatic reload

### Build

To create production builds:

```bash
# General build (compiles TypeScript and React)
npm run build

# Distribution for macOS (arm64)
npm run dist:mac

# Distribution for Windows (x64)
npm run dist:win

# Distribution for Linux (x64)
npm run dist:linux
```

## Usage

### 1. Starting the Application

When you open the application, you'll see the **Workspace Selection** screen.

### 2. Selecting a Workspace

- Click on "Select Workspace"
- Choose a folder with the `_ws` suffix (e.g., `my_project_ws`)
- The system will validate the workspace structure

### 3. Choosing a Package

After selecting a valid workspace:
- A list of available packages will be loaded
- Click on a package to edit it
- Or create a new package by clicking "New Package"

### 4. Editing Blocks

On the editing screen:
- **Blockly Editor** (left): Drag blocks from the toolbox to create your logic
- **Node Manager** (right): View and configure all nodes in your package
- **Node Editor**: Double-click on a node to edit its properties

### 5. Generating Code

- Python code is generated automatically from your blocks
- View the generated code in the integrated code editor
- The code can be exported or copied for use in other projects

### 6. Saving Changes

All changes are saved automatically to the workspace.

## ️Technologies Used

### Frontend
- **React 18**: UI framework
- **Vite**: Fast build tool
- **Blockly**: Visual blocks engine
- **react-blockly**: React integration for Blockly
- **Ant Design**: UI components
- **Monaco Editor**: Advanced code editor

### Desktop
- **Electron 35**: Desktop application framework
- **TypeScript**: Typed language

### Build & Deploy
- **electron-builder**: Electron application packaging
- **ESLint**: Code linting
- **npm-run-all**: Parallel script execution

## Code Conventions

- **TypeScript**: All TypeScript code with explicit types
- **React**: Functional components with Hooks
- **CSS Modules**: Component-scoped styles
- **Naming**: camelCase for variables/functions, PascalCase for components

## Troubleshooting

### Workspace does not appear as valid
- Make sure the folder ends with `_ws`
- Verify that the workspace structure is correct

### Blocks do not appear in the editor
- Reload the application (`F5` or `Ctrl+R`)
- Verify that blocks are correctly defined in `customBlocks.ts`

### Python code is not generated
- Verify that all blocks are correctly connected
- Validate block structure in `blocksToPython.ts`
