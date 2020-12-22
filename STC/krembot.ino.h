#include <Krembot/controller/krembot_controller.h>
#include <vector>
#include <queue>

struct MapMsg{
    int ** occupancyGrid;
    Real resolution;
    CVector2 origin;
    int height, width;

};

struct PosMsg{
    CVector2 pos;
    CDegrees degreeX;
};
class Cell{
    int xPos;
    int yPos;
    std::list<Cell*> adjacencyList;
public:
    Cell(int _xPos, int _yPos);
    int getXPos() const;
    int getYPos() const;
    std::list<Cell*>* getAdjacencyList();
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
    void pos_to_row_col_full_grid(const CVector2& pos, int & row, int & col) const;
    void pos_to_row_col_robot_grid(const CVector2& pos, int & row, int & col) const;
    void pos_to_row_col_stc_grid(const CVector2& pos,int & row, int & col) const;
    bool got_to_orientation(CDegrees degree);
    bool got_to_cell(int _col, int _row);
    void save_grid_to_file(std::string name, int** grid, int _height, int _width);
    void save_grid_to_file_with_robot_location(std::string name, int** grid, int _height, int _width,
                                               int robot_col, int robot_row);
    int mapIndex2Dto1D(int xPos, int yPos);

    bool checkUpDirection(Cell _cell);
    bool checkRightDirection(Cell _cell);
    bool checkDownDirection(Cell _cell);
    bool checkLeftDirection(Cell _cell);
    void addAdjCells(Cell &cell, bool visited[]);
    void pos_to_col_row(CVector2 pos, int * pCol, int *pRow);
    void save_dm(std::string name, int width,int height,int resolution);
    int **create_grid(int **grid, int _height, int _width);
    int **create_resolution_grid(int **grid, int _height, int _width);
    void DFS(Cell &root, bool visited[]);

    void Init(TConfigurationNode &t_node) override {
        KrembotController::Init(t_node);
        if ( ! krembot.isInitialized() ) {
            throw std::runtime_error("krembot.ino.cpp: krembot wasn't initialized in controller");
        }
        Particle.setName(krembot.getName());
    }
    void ControlStep() override {
        if(isFirst) {
            setup();
            isFirst = false;
        }
        loop();
    }
};


REGISTER_CONTROLLER(STC_controller, "STC_controller")