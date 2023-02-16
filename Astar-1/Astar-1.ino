#include <swarm_arena.h>

void setup()
{
  Serial.begin(115200);
  Serial.println(ESP.getFreeHeap());
}

const int COL = (600/30);
const int ROW = (150/30);
bool check = true;

int ** arena =  getmap(1,COL,ROW); //generate first param map like 1.
int ** apath; //path to followed by astar

int sx = 0;
int sy = 0;
int gx = 3;
int gy = 3;
int left_lim = 0;
int right_lim = 19;




void loop()
{
  if (check)
  {
  arena = our_obstacles(true,arena); //param 1 true to set closed list for Astar;
  //Serial.println("Printing Arena.");printmap(arena,COL,ROW); //printing ones arena
  apath = Astar(sx,sy,gx,gy,arena,left_lim,right_lim);
  //Serial.println("Printing Path by Astar.");
  //printmap(apath,2,2); //prints path x,y for 10 points
  check = false;
  }
}
