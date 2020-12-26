#include <Krembot/controller/krembot_controller.h>
#include <vector>
#include <queue>
#include <list>
#include <math.h>

struct MapMsg {
    int **occupancyGrid;
    Real resolution;
    CVector2 origin;
    int height, width;

};

struct PosMsg {
    CVector2 pos;
    CDegrees degreeX;
};
/*
 * This struct contains all the degrees the robot can turn to.
 */
struct Degrees {
    CDegrees up_degree = CDegrees(90);
    CDegrees left_degree = CDegrees(180);
    CDegrees down_degree = CDegrees(270);
    CDegrees right_degree = CDegrees(0);
};

/*
 * This struct contains all the directions the robot can travel from his current position on the tree.
 */
struct Directions {
    bool upDir = false, rightDir = false, downDir = false, leftDir = false;
};

/*
 * A class holding the position of the robot on the grid and all his neighbors.
 */
class Cell {
    int xPos;
    int yPos;
    std::list<Cell *> adjacencyList;
public:
    /*
     * Construct the cell with a given row and column.
     */
    Cell(int _xPos, int _yPos);

    /*
     * Get the row pos.
     */
    int getXPos() const;

    /*
     * Get the column pos.
     */
    int getYPos() const;

    /*
     * Get all the neighbors of the robot.
     */
    std::list<Cell *> *getAdjacencyList();

    /*
     * Add a given neighbor to the robot's list.
     */
    void addNeighbor(Cell *cell);
};

class STC_controller : public KrembotController {
private:
    Real robotSize = 0.20;
    bool isFirst = true;
public:
    MapMsg mapMsg;
    PosMsg posMsg;
    int ROBOT_SIZE;
    int DFS_WIDTH;
    int DFS_HEIGHT;
    ParticleObserver Particle;

    ~STC_controller() = default;

    void setup();

    void loop();

    /*
     * This function converts the given row and col and col to the original grid.
     */
    void pos_to_row_col_given_grid(const CVector2 &pos, int &row, int &col) const;

    /*
     * This function converts the row and col to the reduced grid.
     */
    void pos_to_row_col_reduced_grid(const CVector2 &pos, int &row, int &col) const;

    /*
     * This function covnerts the row and col to the stc grid.
     */
    void pos_to_row_col_stc_grid(const CVector2 &pos, int &row, int &col) const;

    /*
     * This function is taken from class and checks if we got to the right orientation.
     */
    bool got_to_orientation(CDegrees degree);

    /*
     * This function is taken from class and checks if we got the cell.
     */
    bool got_to_cell(int _col, int _row);

    /*
     * This function is taken from the class and saves the grid.
     */
    void save_grid_to_file(std::string name, int **grid, int _height, int _width);

    /*
     * This function is taken from the class and saves the grid with robot loc.
     */
    void save_grid_to_file_with_robot_location(std::string name, int **grid, int _height, int _width,
                                               int robot_col, int robot_row);

    /*
     * This function inits the matrix neighbor which will be used in the dfs
     * and the stc grid.
     */
    void init_matrix_neighbor(int width_size, int height_size);

    std::list<Cell *> *get_neighbor_direction(Cell *current_cell, std::list<Cell *> *available_neighbors);

    std::list<Cell *> *
    get_unvisited_neighbors(Cell *current_cell, std::list<Cell *> *available_neighbors, bool visitedRobotGrid[]);

    std::list<Cell *> *get_converted_cells(Cell *current_cell, std::list<Cell *> *available_neighbors);

    /*
     * Given x and y positions, map the 2d index to the 1d index by multi in width (the x pos)
     * and adding the y in the stc grid.
     */
    CVector2 mapResolutionToStc(int xPos, int yPos);

    /*
     * Given x and y positions, map the 2d index to the 1d index by multi in width (the x pos)
     * and adding the y.
     */
    int mapIndex2Dto1D(int xPos, int yPos);

    /*
     * given a cell and a grid, check if the robot can go up.
     */
    bool checkUpDirection(Cell _cell, int **grid, int given_height);

    /*
     * given a cell and a grid, check if the robot can go to the right.
     */
    bool checkRightDirection(Cell _cell, int **grid, int given_width);

    /*
     * given a cell and a grid, check if the robot can go down.
     */
    bool checkDownDirection(Cell _cell, int **grid);

    /*
     * given a cell and a grid, check if the robot can go to the left.
     */
    bool checkLeftDirection(Cell _cell, int **grid);

    /*
     * Given a cell, a visited list, a current grid and the dimensions  (also check
     * if its a dfs run), add all the neighbors of the cell, excluding 1's and out of range
     * cells.
     */
    void addAdjCells(Cell &cell, int **grid, bool visited[], int given_height, int given_width, bool isDfsRun);

    /*
     * Create the transformation to walk on the grid.
     */
    void pos_to_col_row(CVector2 pos, int *pCol, int *pRow);

    /*
     * This function creates the reduced grid (width/ROBOT_SIZE) and returns it.
     */
    int **create_grid(int **grid, int _height, int _width);

    /*
     * This function creates the resolution grid (the stc grid) which the dfs will
     * work and and create the spanning tree.
     */
    int **create_resolution_grid(int **grid, int _height, int _width);

    /*
     * The DFS function builds the spanning tree the robot is going to travel on.
     * Uses the std::stack variable, instead of the recursive method.
     */
    void DFS(Cell &root, bool visited[]);

    /*
     * This function inits the visitedRobotGrid which will be used in bulding the path.
     */
    void initVisitedRobotGrid(bool visitedRobotGrid[], int _width, int _height);

    /*
     * Given a root cell, the function builds the path the robot should
     * go on the grid. the path is made of the reduced grid, and in the
     * traveling itself converted to the right positions on the regular simualtion
     * grid.
     */
    std::list<Cell *> *pathBuilder(Cell *root, bool visitedRobotGrid[]);

    /*
     * Given a current cell, and the next cell the robot should move to, calculate
     * the degree.
     */
    CDegrees calculateWantedDegree(Cell *current_cell, Cell *next_cell);

    void Init(TConfigurationNode &t_node) override {
        KrembotController::Init(t_node);
        if (!krembot.isInitialized()) {
            throw std::runtime_error("krembot.ino.cpp: krembot wasn't initialized in controller");
        }
        Particle.setName(krembot.getName());
    }

    void ControlStep() override {
        if (isFirst) {
            setup();
            isFirst = false;
        }
        loop();
    }
};


REGISTER_CONTROLLER(STC_controller, "STC_controller")