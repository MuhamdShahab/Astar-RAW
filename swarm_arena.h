#ifndef SWARM_ARENA_H
#define SWARM_ARENA_H

#include <Arduino.h>
#include <Queue>
using namespace std;

extern const int COL; //total length/box length
extern const int ROW; //total height/box height

int** getmap(int entry,int col, int row);

//mainlit used for closed list operations
//for static and dynamic Robots
class node
{
  private:
    int pos_x = -10;
    int pos_y = -10;
  public:
    node(int x_cord,int y_cord);
    int** draw_obstacle_and_boundary(int** arr2);
    int** draw_obstacle(int** arr4);
    friend bool operator== (const node& n1, const node& n2);
    friend bool operator!= (const node& n1, const node& n2);
    void setnoderow(int val){pos_y = val;}
    void setnodecol(int val){pos_x = val;}
    int getnoderow(){return pos_y;}
    int getnodecol(){return pos_x;}
};
void print_node(node& param);


void printmap(int** arr,int col, int row);
void printmap_rev(int** arr,int col, int row);

int** place_obstacle(bool close, int** arr1,int posi_x,int posi_y);
int** our_obstacles(bool close = true, int** arr3 = 0);


class complete_node
{
  private:
    //int tag =0;
    int x = 0;
    int y = 0;
    int Px = 0;
    int Py = 0;
    double gnn = 0;
    double hnn = 0;
    double fnn = 0;

  public:
  complete_node(int x1, int y1,int Px1,int Py1, double gnn1, double hnn1);
  //operator Overloading
  friend bool operator== (const complete_node& n1, const complete_node& n2);
  friend bool operator!= (const complete_node& n1, const complete_node& n2);
  //getters
  //int gettag()const {return tag;}
  int getProw()const {return Py;}
  int getPcol()const {return Px;}
  int getrow()const {return y;}
  int getcol()const{return x;}
  double gethnn()const {return hnn;}
  double getgnn()const {return gnn;}
  double getfnn()const {return fnn;}
  //setters
  //void settag(int val){tag = val;}
  void setProw(int val){Py = val;}
  void setPcol(int val){Px = val;}
  void setrow(int val){ y = val;}
  void setcol(int val){ x = val;}
  void sethnn(float val){ hnn = val;}
  void setgnn(float val){ gnn = val;}
  void setfnn(float val){ fnn = val;}
};
void print_complete_node(complete_node& param);
class comparefnn
{
public:
    bool operator() (const complete_node & a, const complete_node & b)
    {
        return a.getfnn() > b.getfnn();
    }
};
   
bool isgoalsourcevalid(int sx, int sy, int gx, int gy, int left_lim, int right_lim);
bool issourcegoalsame(int sx, int sy, int gx, int gy);
int** markgoalandstart(int** arr6,int sx,int sy,int gx,int gy);
double cost_calculator(double sx, double sy, double gx, double gy);
double heuristic(double sx, double sy, double gx, double gy);

bool checknodeinclosed(node& temp_putri);

void expand_array(int** arr7, int px, int py,double cum_gn,int gx,int gy, int left_lim, int right_lim);

void printonesopenlist();
void printzerosopenlist();
void printclosedlist();
int** Astar(int sx, int sy, int gx, int gy, int** arr5, int left_lim, int right_lim);


#endif