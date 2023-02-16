#include <swarm_arena.h>
void setup()
{
  Serial.begin(115200);
  }

const int COL = (800/10)+1;
const int ROW = (120/10)+1;

bool check = true;

int ** arena =  getmap();

void loop()
{
  if (check)
  {
  arena = our_obstacles(arena);
  printmap(arena);
  check = false;
  }

}
