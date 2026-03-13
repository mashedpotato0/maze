# Maze Generator & Solver

A real-time maze generator and pathfinding visualizer written in C++ using OpenGL (GLUT/freeglut). Generates random mazes using recursive backtracking, then solves them with your choice of three pathfinding algorithms.

## Features

- **Real-time maze generation** — Watch the maze being constructed cell by cell using recursive backtracking (DFS with stack) on a 100×100 grid.
- **Three solving algorithms** (selectable via in-app menu):
  - **A\*** — Heuristic-guided (Manhattan distance), fastest to find optimal path
  - **Dijkstra's Algorithm** — Guaranteed shortest path, explores by cost
  - **Breadth-First Search (BFS)** — Unweighted shortest path
- **Performance stats overlay** — Shows algorithm used, nodes visited, time taken (ms), and path length after solving.
- Color-coded visualization:
  - 🟡 Current generation cell
  - 🟢 Start cell
  - 🟣 End cell
  - 🔵 Explored cells
  - 🔴 Solution path

## Controls

| Key | Action |
|---|---|
| `S` | Open algorithm selection menu |
| `R` | Regenerate a new random maze |
| `Esc` | Close the algorithm menu |
| `↑ / ↓` | Navigate menu |
| `Enter` | Confirm algorithm selection and solve |

## Build

**Dependencies:** freeglut (OpenGL Utility Toolkit), OpenGL

```bash
# Ubuntu/Debian
sudo apt install freeglut3-dev

# Arch Linux
sudo pacman -S freeglut

# Compile
g++ maze.cpp -o maze -lGL -lGLU -lglut
```

## Run

```bash
./maze
```

The maze will start generating automatically. Once complete, press `S` to solve it.
