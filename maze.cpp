#include <GL/glut.h>
#include <vector>
#include <stack>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <iostream>
#include <cmath>

const int WIDTH = 1000;
const int HEIGHT = 1000;
const int ROWS = 100;
const int COLS = 100;

struct Cell {
    int x, y;
    bool visited = false;
    bool walls[4] = {true, true, true, true}; 
    bool inPath = false;
    bool explored = false;
    
    float gCost = 0;
    float hCost = 0;
    float fCost = 0;
    Cell* parent = nullptr;

    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
};

struct PathNode {
    Cell* cell;
    float cost;
    
    bool operator>(const PathNode& other) const {
        return cost > other.cost;
    }
};

enum SolveState {
    GENERATING,
    READY_TO_SOLVE,
    SOLVING,
    SOLVED
};

enum Algorithm {
    ASTAR,
    DIJKSTRA,
    BFS
};

std::vector<std::vector<Cell>> grid(ROWS, std::vector<Cell>(COLS));
std::stack<Cell*> cellStack;
Cell* current;
Cell* start;
Cell* end;

SolveState state = GENERATING;
Algorithm currentAlgorithm = ASTAR;
std::vector<Cell*> solutionPath;
int nodesVisited = 0;
double solveTime = 0.0;
bool showStats = false;

bool showMenu = false;
int menuSelection = 0;
const char* algorithmNames[] = {"A* Algorithm", "Dijkstra's Algorithm", "Breadth-First Search"};

void initGrid() {
    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            grid[y][x].x = x;
            grid[y][x].y = y;
            grid[y][x].visited = false;
            grid[y][x].inPath = false;
            grid[y][x].explored = false;
            grid[y][x].gCost = 0;
            grid[y][x].hCost = 0;
            grid[y][x].fCost = 0;
            grid[y][x].parent = nullptr;
            for (int i = 0; i < 4; i++) {
                grid[y][x].walls[i] = true;
            }
        }
    }
    current = &grid[0][0];
    current->visited = true;
    start = &grid[0][0];
    end = &grid[ROWS-1][COLS-1];
    srand(time(0));
}

Cell* getUnvisitedNeighbor(Cell* cell) {
    std::vector<Cell*> neighbors;
    int x = cell->x;
    int y = cell->y;

    if (y > 0 && !grid[y - 1][x].visited) neighbors.push_back(&grid[y - 1][x]);
    if (x < COLS - 1 && !grid[y][x + 1].visited) neighbors.push_back(&grid[y][x + 1]);
    if (y < ROWS - 1 && !grid[y + 1][x].visited) neighbors.push_back(&grid[y + 1][x]);
    if (x > 0 && !grid[y][x - 1].visited) neighbors.push_back(&grid[y][x - 1]);

    if (!neighbors.empty()) {
        int r = rand() % neighbors.size();
        return neighbors[r];
    }
    return nullptr;
}

void removeWalls(Cell* a, Cell* b) {
    int dx = a->x - b->x;
    int dy = a->y - b->y;

    if (dx == 1) { a->walls[3] = false; b->walls[1] = false; }
    else if (dx == -1) { a->walls[1] = false; b->walls[3] = false; }

    if (dy == 1) { a->walls[0] = false; b->walls[2] = false; }
    else if (dy == -1) { a->walls[2] = false; b->walls[0] = false; }
}

std::vector<Cell*> getValidNeighbors(Cell* cell) {
    std::vector<Cell*> neighbors;
    int x = cell->x;
    int y = cell->y;

    if (y > 0 && !cell->walls[0]) neighbors.push_back(&grid[y - 1][x]); // top
    if (x < COLS - 1 && !cell->walls[1]) neighbors.push_back(&grid[y][x + 1]); // right
    if (y < ROWS - 1 && !cell->walls[2]) neighbors.push_back(&grid[y + 1][x]); // bottom
    if (x > 0 && !cell->walls[3]) neighbors.push_back(&grid[y][x - 1]); // left

    return neighbors;
}

float heuristic(Cell* a, Cell* b) {
    return std::abs(a->x - b->x) + std::abs(a->y - b->y); 
}

void reconstructPath(Cell* endCell) {
    solutionPath.clear();
    Cell* current = endCell;
    while (current != nullptr) {
        solutionPath.push_back(current);
        current = current->parent;
    }
    std::reverse(solutionPath.begin(), solutionPath.end());
    
    for (Cell* cell : solutionPath) {
        cell->inPath = true;
    }
}

void solveAStar() {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            grid[y][x].explored = false;
            grid[y][x].inPath = false;
            grid[y][x].gCost = 0;
            grid[y][x].hCost = 0;
            grid[y][x].fCost = 0;
            grid[y][x].parent = nullptr;
        }
    }
    
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> openSet;
    std::unordered_set<Cell*> closedSet;
    
    start->gCost = 0;
    start->hCost = heuristic(start, end);
    start->fCost = start->gCost + start->hCost;
    
    openSet.push({start, start->fCost});
    nodesVisited = 0;
    
    while (!openSet.empty()) {
        Cell* current = openSet.top().cell;
        openSet.pop();
        
        if (closedSet.find(current) != closedSet.end()) continue;
        
        closedSet.insert(current);
        current->explored = true;
        nodesVisited++;
        
        if (current == end) {
            reconstructPath(end);
            state = SOLVED;
            auto endTime = std::chrono::high_resolution_clock::now();
            solveTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            showStats = true;
            return;
        }
        
        std::vector<Cell*> neighbors = getValidNeighbors(current);
        for (Cell* neighbor : neighbors) {
            if (closedSet.find(neighbor) != closedSet.end()) continue;
            
            float tentativeGCost = current->gCost + 1;
            
            if (tentativeGCost < neighbor->gCost || neighbor->gCost == 0) {
                neighbor->parent = current;
                neighbor->gCost = tentativeGCost;
                neighbor->hCost = heuristic(neighbor, end);
                neighbor->fCost = neighbor->gCost + neighbor->hCost;
                
                openSet.push({neighbor, neighbor->fCost});
            }
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    solveTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    state = SOLVED;
    showStats = true;
}

void solveDijkstra() {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            grid[y][x].explored = false;
            grid[y][x].inPath = false;
            grid[y][x].gCost = INFINITY;
            grid[y][x].parent = nullptr;
        }
    }
    
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> pq;
    start->gCost = 0;
    pq.push({start, 0});
    nodesVisited = 0;
    
    while (!pq.empty()) {
        Cell* current = pq.top().cell;
        float currentCost = pq.top().cost;
        pq.pop();
        
        if (current->explored) continue;
        
        current->explored = true;
        nodesVisited++;
        
        if (current == end) {
            reconstructPath(end);
            state = SOLVED;
            auto endTime = std::chrono::high_resolution_clock::now();
            solveTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            showStats = true;
            return;
        }
        
        std::vector<Cell*> neighbors = getValidNeighbors(current);
        for (Cell* neighbor : neighbors) {
            if (neighbor->explored) continue;
            
            float newCost = currentCost + 1;
            if (newCost < neighbor->gCost) {
                neighbor->gCost = newCost;
                neighbor->parent = current;
                pq.push({neighbor, newCost});
            }
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    solveTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    state = SOLVED;
    showStats = true;
}

void solveBFS() {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            grid[y][x].explored = false;
            grid[y][x].inPath = false;
            grid[y][x].parent = nullptr;
        }
    }
    
    std::queue<Cell*> queue;
    queue.push(start);
    start->explored = true;
    nodesVisited = 0;
    
    while (!queue.empty()) {
        Cell* current = queue.front();
        queue.pop();
        nodesVisited++;
        
        if (current == end) {
            reconstructPath(end);
            state = SOLVED;
            auto endTime = std::chrono::high_resolution_clock::now();
            solveTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            showStats = true;
            return;
        }
        
        std::vector<Cell*> neighbors = getValidNeighbors(current);
        for (Cell* neighbor : neighbors) {
            if (!neighbor->explored) {
                neighbor->explored = true;
                neighbor->parent = current;
                queue.push(neighbor);
            }
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    solveTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    state = SOLVED;
    showStats = true;
}

void drawCell(Cell& cell) {
    float w = WIDTH / (float)COLS;
    float h = HEIGHT / (float)ROWS;
    float x = cell.x * w;
    float y = cell.y * h;

    if (cell.inPath && state == SOLVED) {
        glColor3f(1.0, 0.0, 0.0); 
        glBegin(GL_QUADS);
        glVertex2f(x + 1, y + 1);
        glVertex2f(x + w - 1, y + 1);
        glVertex2f(x + w - 1, y + h - 1);
        glVertex2f(x + 1, y + h - 1);
        glEnd();
    } else if (cell.explored && state == SOLVED) {
        glColor3f(0.3, 0.3, 1.0); 
        glBegin(GL_QUADS);
        glVertex2f(x + 1, y + 1);
        glVertex2f(x + w - 1, y + 1);
        glVertex2f(x + w - 1, y + h - 1);
        glVertex2f(x + 1, y + h - 1);
        glEnd();
    }
    
    if (&cell == start) {
        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_QUADS);
        glVertex2f(x + 2, y + 2);
        glVertex2f(x + w - 2, y + 2);
        glVertex2f(x + w - 2, y + h - 2);
        glVertex2f(x + 2, y + h - 2);
        glEnd();
    } else if (&cell == end) {
        glColor3f(1.0, 0.0, 1.0);
        glBegin(GL_QUADS);
        glVertex2f(x + 2, y + 2);
        glVertex2f(x + w - 2, y + 2);
        glVertex2f(x + w - 2, y + h - 2);
        glVertex2f(x + 2, y + h - 2);
        glEnd();
    }
    
    if (state == GENERATING && cell == *current) {
        glColor3f(1.0, 1.0, 0.0); 
        glBegin(GL_QUADS);
        glVertex2f(x + 2, y + 2);
        glVertex2f(x + w - 2, y + 2);
        glVertex2f(x + w - 2, y + h - 2);
        glVertex2f(x + 2, y + h - 2);
        glEnd();
    }

    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(2.0);
    if (cell.walls[0]) { glBegin(GL_LINES); glVertex2f(x, y); glVertex2f(x + w, y); glEnd(); }
    if (cell.walls[1]) { glBegin(GL_LINES); glVertex2f(x + w, y); glVertex2f(x + w, y + h); glEnd(); }
    if (cell.walls[2]) { glBegin(GL_LINES); glVertex2f(x + w, y + h); glVertex2f(x, y + h); glEnd(); }
    if (cell.walls[3]) { glBegin(GL_LINES); glVertex2f(x, y + h); glVertex2f(x, y); glEnd(); }
}

void drawText(float x, float y, const char* text) {
    glRasterPos2f(x, y);
    for (const char* c = text; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
}

void drawMenu() {
    if (!showMenu) return;
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.0, 0.0, 0.0, 0.7);
    glBegin(GL_QUADS);
    glVertex2f(0, 0);
    glVertex2f(WIDTH, 0);
    glVertex2f(WIDTH, HEIGHT);
    glVertex2f(0, HEIGHT);
    glEnd();
    glDisable(GL_BLEND);
    
    glColor3f(0.2, 0.2, 0.2);
    glBegin(GL_QUADS);
    glVertex2f(WIDTH/2 - 200, HEIGHT/2 - 100);
    glVertex2f(WIDTH/2 + 200, HEIGHT/2 - 100);
    glVertex2f(WIDTH/2 + 200, HEIGHT/2 + 100);
    glVertex2f(WIDTH/2 - 200, HEIGHT/2 + 100);
    glEnd();
    
    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(2.0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(WIDTH/2 - 200, HEIGHT/2 - 100);
    glVertex2f(WIDTH/2 + 200, HEIGHT/2 - 100);
    glVertex2f(WIDTH/2 + 200, HEIGHT/2 + 100);
    glVertex2f(WIDTH/2 - 200, HEIGHT/2 + 100);
    glEnd();
    
    glColor3f(1.0, 1.0, 1.0);
    drawText(WIDTH/2 - 150, HEIGHT/2 - 70, "Choose Solving Algorithm:");
    
    for (int i = 0; i < 3; i++) {
        if (i == menuSelection) {
            glColor3f(1.0, 1.0, 0.0);
        } else {
            glColor3f(1.0, 1.0, 1.0);
        }
        drawText(WIDTH/2 - 120, HEIGHT/2 - 30 + i * 30, algorithmNames[i]);
    }
    
    glColor3f(0.7, 0.7, 0.7);
    drawText(WIDTH/2 - 180, HEIGHT/2 + 70, "Use UP/DOWN arrows to select, ENTER to confirm");
}

void drawStats() {
    if (!showStats) return;
    
    glColor3f(0.0, 1.0, 0.0);
    char buffer[256];
    
    sprintf(buffer, "Algorithm: %s", algorithmNames[currentAlgorithm]);
    drawText(10, 30, buffer);
    
    sprintf(buffer, "Nodes Visited: %d", nodesVisited);
    drawText(10, 50, buffer);
    
    sprintf(buffer, "Time Taken: %.2f ms", solveTime);
    drawText(10, 70, buffer);
    
    sprintf(buffer, "Path Length: %d", (int)solutionPath.size());
    drawText(10, 90, buffer);
    
    drawText(10, HEIGHT - 30, "Press 'R' to regenerate maze, 'S' to solve again");
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    
    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            drawCell(grid[y][x]);
        }
    }
    
    drawMenu();
    drawStats();
    
    glutSwapBuffers();
}

void update(int) {
    if (state == GENERATING) {
        Cell* next = getUnvisitedNeighbor(current);
        if (next) {
            next->visited = true;
            cellStack.push(current);
            removeWalls(current, next);
            current = next;
        } else if (!cellStack.empty()) {
            current = cellStack.top();
            cellStack.pop();
        } else {
            state = READY_TO_SOLVE;
            std::cout << "Maze generation complete! Press 'S' to solve." << std::endl;
        }
    }
    
    glutPostRedisplay();
    glutTimerFunc(0, update, 0);
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 's':
        case 'S':
            if (state == READY_TO_SOLVE || state == SOLVED) {
                showMenu = true;
                showStats = false;
            }
            break;
        case 'r':
        case 'R':
            // reset and regen maze
            initGrid();
            state = GENERATING;
            showStats = false;
            showMenu = false;
            solutionPath.clear();
            break;
        case 27: // ESC
            if (showMenu) {
                showMenu = false;
            }
            break;
        case 13: // ENTER
            if (showMenu) {
                currentAlgorithm = (Algorithm)menuSelection;
                showMenu = false;
                state = SOLVING;
                
                switch (currentAlgorithm) {
                    case ASTAR:
                        solveAStar();
                        break;
                    case DIJKSTRA:
                        solveDijkstra();
                        break;
                    case BFS:
                        solveBFS();
                        break;
                }
            }
            break;
    }
    glutPostRedisplay();
}

void specialKeys(int key, int x, int y) {
    if (showMenu) {
        switch (key) {
            case GLUT_KEY_UP:
                menuSelection = (menuSelection - 1 + 3) % 3;
                break;
            case GLUT_KEY_DOWN:
                menuSelection = (menuSelection + 1) % 3;
                break;
        }
        glutPostRedisplay();
    }
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, WIDTH, HEIGHT, 0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Maze Generator & Solver");

    glClearColor(0, 0, 0, 1);
    initGrid();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);
    glutTimerFunc(0, update, 0);

    std::cout << "Controls:" << std::endl;
    std::cout << "S - Open solver menu" << std::endl;
    std::cout << "R - Regenerate maze" << std::endl;
    std::cout << "ESC - Close menu" << std::endl;
    std::cout << "UP/DOWN - Navigate menu" << std::endl;
    std::cout << "ENTER - Select algorithm" << std::endl;

    glutMainLoop();
    return 0;
}