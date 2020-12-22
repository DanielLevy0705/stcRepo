#include <list>
#include <stack>
#include "krembot.ino.h"


int col, row;
int ** occupancyGrid;
int ** arr;
int ** final_grid;
Directions ** neighbor_matrix;
Real resolution;
CVector2 origin;
int height, width;
int dfs_width, dfs_height;
CVector2 pos;
CVector2 pos_medium_grid;
int col_medium,row_medium;
CDegrees degreeX;

enum State {
    move,
    turn
} state = turn;



void STC_controller::setup() {
    krembot.setup();
    krembot.Led.write(0,255,0);
    occupancyGrid = mapMsg.occupancyGrid;
    resolution = mapMsg.resolution;
    ROBOT_SIZE = (int)(robotSize/mapMsg.resolution);
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;

    save_grid_to_file("/home/eliran/krembot_ws/STC/grid.txt",occupancyGrid,height,width);
    pos = posMsg.pos;
    pos_medium_grid = posMsg.pos;
    degreeX = posMsg.degreeX;
    pos_to_col_row(pos, &col,&row);
    pos_to_col_row(pos_medium_grid,&col_medium,&row_medium);
    save_grid_to_file_with_robot_location("/home/eliran/krembot_ws/STC/grid-with-robot.txt",occupancyGrid,
                                          height,width,col,row);
    arr = create_grid(occupancyGrid,height,width);
    final_grid = create_resolution_grid(arr,height/ROBOT_SIZE,width/ROBOT_SIZE);
    init_matrix_neighbor(width/ROBOT_SIZE/2,height/ROBOT_SIZE/2);
    int xOrigin = pos.GetX();
    int yOrigin = pos.GetY();
    pos_to_row_col_robot_grid(pos,xOrigin,yOrigin);
    pos_to_row_col_stc_grid(pos,xOrigin,yOrigin);

    //Cell cell(origin.GetX(),origin.GetY());
    Cell cell(xOrigin,yOrigin);
    dfs_width = width/ROBOT_SIZE/2;
    dfs_height = width/ROBOT_SIZE/2;
    bool visited[dfs_width*dfs_height];
    int ** visitedRobotGrid = initVisitedRobotGrid(width,height);
    DFS(cell,visited);
    int xOrig = pos_medium_grid.GetX();
    int yOrig = pos_medium_grid.GetY();
    pos_to_row_col_robot_grid(pos_medium_grid,xOrig,yOrig);
    Cell cell_medium(xOrig,yOrig);
    addAdjCells(cell_medium,arr,height/ROBOT_SIZE, width/ROBOT_SIZE,false);
    std::list<Cell*> *adj = cell_medium.getAdjacencyList();
    std::list<Cell*> *list = get_neighbor_direction(&cell_medium,adj);
    //save_dm("/home/eliran/krembot_ws/STC/dm.txt",width,height,resolution);
    save_grid_to_file("/home/eliran/krembot_ws/STC/new_grid.txt",arr,height/ROBOT_SIZE, width/ROBOT_SIZE);
    save_grid_to_file("/home/eliran/krembot_ws/STC/walking_grid.txt",final_grid,height/ROBOT_SIZE/2, width/ROBOT_SIZE/2);

}

void STC_controller::loop() {
    krembot.loop();
    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
}


Cell::Cell(int _xPos, int _yPos) {
    xPos = _xPos;
    yPos = _yPos;
}

int Cell::getXPos() const {
    return xPos;
}

int Cell::getYPos() const {
    return yPos;
}

std::list<Cell*> *Cell::getAdjacencyList() {
    return &adjacencyList;
}

void Cell::addNeighbor(Cell *cell) {
    adjacencyList.push_back(cell);
}

void STC_controller::init_matrix_neighbor(int width_size,int height_size){
    neighbor_matrix = new Directions*[width_size];
    for (int i =0; i< width_size; i++){
        neighbor_matrix[i] = new Directions[height_size];
    }
}


bool STC_controller::checkUpDirection(Cell _cell, int **grid, int given_height) {
    int xPos = _cell.getXPos();
    int yPos = _cell.getYPos();
    int up = xPos + 1;
    if (up > (given_height - 1) || grid[up][yPos] == 1)
        return false;
    return true;
}

bool STC_controller::checkRightDirection(Cell _cell, int **grid, int given_width) {
    int xPos = _cell.getXPos();
    int yPos = _cell.getYPos();
    int right = yPos + 1;
    if (right > (given_width - 1) || grid[xPos][right] == 1)
        return false;
    return true;
}

bool STC_controller::checkDownDirection(Cell _cell, int **grid) {
    int yPos = _cell.getYPos();
    int xPos = _cell.getXPos();
    int down = xPos - 1;
    if (down < 0 || grid[down][yPos] == 1)
        return false;
    return true;
}

bool STC_controller::checkLeftDirection(Cell _cell, int **grid) {
    int xPos = _cell.getXPos();
    int yPos = _cell.getYPos();
    int left = yPos - 1;
    if (left < 0 || grid[xPos][left] == 1)
        return false;
    return true;
}
int STC_controller::mapIndex2Dto1D(int xPos, int yPos){
    return ((xPos) * dfs_width + yPos);
}
void STC_controller::addAdjCells(Cell &cell, int **grid,int given_height,int given_width, bool isDfsRun) {
    if (checkUpDirection(cell,grid,given_height))
    {
        Cell *_cell = new Cell(cell.getXPos() + 1, cell.getYPos());
        cell.addNeighbor(_cell);
        if(isDfsRun) {
            neighbor_matrix[cell.getXPos()][cell.getYPos()].upDir = true;
        }
    }
    if (checkRightDirection(cell,grid,given_width))
    {
        Cell *_cell = new Cell(cell.getXPos(), cell.getYPos() + 1);
        cell.addNeighbor(_cell);
        if (isDfsRun){
            neighbor_matrix[cell.getXPos()][cell.getYPos()].rightDir = true;
        }
    }
    if (checkDownDirection(cell,grid))
    {
        Cell *_cell = new Cell(cell.getXPos() - 1, cell.getYPos());
        cell.addNeighbor(_cell);
        if (isDfsRun){
            neighbor_matrix[cell.getXPos()][cell.getYPos()].downDir = true;
        }
    }
    if (checkLeftDirection(cell,grid))
    {
        Cell *_cell = new Cell(cell.getXPos(), cell.getYPos() - 1);
        cell.addNeighbor(_cell);
        if (isDfsRun){
            neighbor_matrix[cell.getXPos()][cell.getYPos()].leftDir = true;
        }
    }
}


void STC_controller::save_grid_to_file(std::string name, int **grid, int _height, int _width) {
    std::ofstream  m_cOutput;
    m_cOutput.open(name, std::ios_base::trunc | std::ios_base::out);
    for (int row = _height - 1; row >=0; row--)
    {
        for (int col =0; col < _width ;col ++)
        {
            m_cOutput << grid[row][col];
        }
        m_cOutput << std::endl;
    }
    m_cOutput.close();
}

void STC_controller::save_grid_to_file_with_robot_location(std::string name, int **grid, int _height, int _width,
                                                           int robot_col, int robot_row) {
    std::ofstream  m_cOutput;
    m_cOutput.open(name, std::ios_base::trunc | std::ios_base::out);
    int to_print;
    for (int row = _height - 1; row >=0; row--)
    {
        for (int col =0; col < _width ;col ++)
        {
            if ((col == robot_col) && (row == robot_row))
            {
                to_print = 2;
            } else {
                to_print = grid[row][col];
            }
            m_cOutput << to_print;
        }
        m_cOutput << std::endl;
    }
    m_cOutput.close();

}

void STC_controller::pos_to_col_row(CVector2 pos, int *pCol, int *pRow) {
    *pCol = (pos.GetX() - origin.GetX()) / resolution;
    *pRow = (pos.GetY() - origin.GetY()) / resolution;
}

void STC_controller::pos_to_row_col_full_grid(const CVector2& pos, int & row, int & col) const
{
    row = (pos.GetY() - origin.GetY()) / resolution;
    col = (pos.GetX() - origin.GetX()) / resolution;
}
void STC_controller::pos_to_row_col_robot_grid(const CVector2& pos, int & row, int & col) const
{
    pos_to_row_col_full_grid(pos, row, col);
    row = row / (robotSize / resolution);
    col = col / (robotSize / resolution);
}
void STC_controller::pos_to_row_col_stc_grid(const CVector2& pos,int & row, int & col) const
{
    pos_to_row_col_robot_grid(pos, row, col);
    row = row / 2.0f;
    col = col / 2.0f;
}

bool STC_controller::got_to_cell(int _col, int _row) {
    Real threshold = 0.0005;
    CVector2 cell_center_pos;
    cell_center_pos.Set(_col*resolution, _row*resolution);
    cell_center_pos += origin;
    if ((pos-cell_center_pos).SquareLength() < threshold)
    {
        return true;
    } else {
        return false;
    }
}

bool STC_controller::got_to_orientation(CDegrees degree) {
    if (((degreeX - degree).UnsignedNormalize().GetValue() > 0.50) && ((degreeX-degree).UnsignedNormalize()).GetValue() < 359.58)
    {
        return true;
    }
    else {
        return false;
    }
}

void STC_controller::save_dm(std::string name,int width, int height, int resoulution) {
    std::ofstream  m_cOutput;
    m_cOutput.open(name, std::ios_base::trunc | std::ios_base::out);
    m_cOutput << width << std::endl;
    m_cOutput << height << std::endl;
    m_cOutput << resoulution << std::endl;
    m_cOutput.close();

}


int **STC_controller::create_grid(int **grid, int _height, int _width) {
    int new_width = _width /ROBOT_SIZE;
    int new_height = _height /ROBOT_SIZE;
    //Init the new grids by the calculated size
    int **resolution_grid = new int *[new_width];
    for (int i = 0; i < new_width; i++) {
        resolution_grid[i] = new int[new_height];
    }
    //Fill all with zeros
    for (int row = new_height - 1; row >= 0; row--) {
        for (int col = 0; col < new_width; col++) {
            resolution_grid[row][col] = 0;
        }
    }

    int row_pos = 0, col_pos = 0;
    int max_width = ROBOT_SIZE * new_width;
    int max_height = ROBOT_SIZE * new_height;
    for (int row = max_height - 1; row >= 0; row--) {
        for (int col = 0; col < max_width; col++) {
            if (grid[row][col] == 1) {
                //Calculate which cell we need to put 1
                row_pos = row / ROBOT_SIZE;
                col_pos = col / ROBOT_SIZE;
                resolution_grid[row_pos][col_pos] = 1;
            }

        }

    }
    return resolution_grid;

}

int **STC_controller::create_resolution_grid(int **grid, int _height, int _width) {
    int walking_width = _width / 2;
    int walking_height = _height / 2;
    int row_pos =0, col_pos = 0;
    int **walking_grid = new int *[walking_width];
    for (int i = 0; i < walking_width; i++) {
        walking_grid[i] = new int[walking_height];
    }
    for (int row = walking_width - 1; row >= 0; row--) {
        for (int col = 0; col < walking_height; col++) {
            walking_grid[row][col] = 0;
        }
    }
    int max_width = walking_width * 2;
    int max_height = walking_height * 2;
    for (int row = max_height-1; row >= 0; row--) {
        for (int col = 0; col < max_width; col++) {
            if (grid[row][col] == 1) {
                //Calculate which cell we need to put 1
                row_pos = row / 2;
                col_pos = col / 2;
                walking_grid[row_pos][col_pos] = 1;
            }

        }

    }
    return walking_grid;
}

void STC_controller::DFS(Cell &root,bool visited[]) {
    for (int i =0; i<dfs_width*dfs_height; i++){
        visited[i] = false;
    }
    std::stack<Cell*> stack;
    stack.push(&root);
    visited[root.getXPos()*dfs_width + root.getYPos()] = true;
    while (!stack.empty()) {
        int index = 0;
        Cell *cell = stack.top();
        stack.pop();
        addAdjCells(*cell,final_grid,height/ROBOT_SIZE/2, width/ROBOT_SIZE/2,true);
        std::list<Cell*> *adjList = cell->getAdjacencyList();
        for (auto &neighbor : *adjList) {
            index = mapIndex2Dto1D(neighbor->getXPos(), neighbor->getYPos());
            if (!visited[index]){
                stack.push(neighbor);
                visited[index] = true;
            }
        }
    }
    int x = 5;
}

std::list<Cell*> *STC_controller::get_neighbor_direction(Cell *current_cell, std::list<Cell *> *available_neihbors) {
    std::list<Cell*> *list = new std::list<Cell*>;
    CVector2 current_pos = mapResolutionToStc(current_cell->getXPos(),current_cell->getYPos());
    int current_pos_x = current_pos.GetX();
    int current_pos_y = current_pos.GetY();
    for (auto &neighbor : *available_neihbors){
        Directions dir;
        // What direction i have moved in the 25X25 Grid
        dir.upDir = neighbor->getXPos() > current_cell->getXPos();
        dir.leftDir = neighbor->getYPos() < current_cell->getYPos();
        dir.rightDir = neighbor->getYPos() > current_cell->getYPos();
        dir.downDir = neighbor->getXPos() < current_cell->getXPos();
        CVector2 pos = mapResolutionToStc(neighbor->getXPos(),neighbor->getYPos());
        // Check if there is a neighbor in the direction we are moving to OR in the same cell (STC)
        if(dir.upDir && (neighbor_matrix[current_pos_x][current_pos_y].upDir
        || pos.GetX() == current_pos_x)){
            list->push_back(neighbor);
        }
        else if(dir.rightDir && (neighbor_matrix[current_pos_x][current_pos_y].rightDir
        || pos.GetY() == current_pos_y)){
            list->push_back(neighbor);
        }
        else if(dir.downDir && (neighbor_matrix[current_pos_x][current_pos_y].downDir
        || pos.GetX() == current_pos_x)){
            list->push_back(neighbor);
        }
        else if(dir.leftDir && (neighbor_matrix[current_pos_x][current_pos_y].leftDir
        || pos.GetY() == current_pos_y)){
            list->push_back(neighbor);
        }

    }
    return list;
}

CVector2 STC_controller::mapResolutionToStc(int xPos, int yPos) {
    return CVector2(xPos/2,yPos/2);
}

int **STC_controller::initVisitedRobotGrid(int _width, int _height) {
    int new_width = _width /ROBOT_SIZE;
    int new_height = _height /ROBOT_SIZE;
    //Init the new grids by the calculated size
    int **visitedRobotGrid = new int *[new_width];
    for (int i = 0; i < new_width; i++) {
        visitedRobotGrid[i] = new int[new_height];
    }
    for (int row = new_height - 1; row >= 0; row--) {
        for (int col = 0; col < new_width; col++) {
            visitedRobotGrid[row][col] = 0;
        }
    }
    return visitedRobotGrid;
}




