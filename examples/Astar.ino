#include <swarm_arena.h>

void setup()
{
  Serial.begin(115200);
}

const int COL = (600/30);
const int ROW = (150/30);


int ** arena =  getmap(1,COL,ROW); //generate first param map like 1.

int source_x = 0;
int source_y = 0;
int goal_x = 15;
int goal_y = 2;
int left_lim = 0;
int right_lim = 19;

int** my_obstacles(int** arr)
{
  arr = place_obstacle(arr,1,3);
  arr = place_obstacle(arr,7,4);
  arr = place_obstacle(arr,12,1);
  arr = place_obstacle(arr,17,2);
  return arr;
}

void loop()
{
  arena = my_obstacles(arena);//place your obstacles
  astar_result apath = Astar(source_x,source_y,goal_x,goal_y,arena); //gives a 2D array of feasible path with number of coordinates
  Serial.println("A-star found the following path: "); printmap(apath.getpath(),apath.getsize(),2); //prints the coordinates with getsize number of columns and 2(x and y cordinates)
  Serial.print("We have to traverse the following number of coordinates: "); Serial.println(apath.getsize());
  while(true)
  {}
}
