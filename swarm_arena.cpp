#include "swarm_arena.h"

bool found_dest = false;
int** getmap(int entry, int col, int row)
{
  int** arr = new int*[row];
  for (int i = 0; i < row; ++i) {
    arr[i] = new int[col];
    for (int j = 0; j < col; ++j) {
      arr[i][j] = entry;
    }
  }
  //Serial.println("Map of Ones Created Successfully");
  return arr;
}

//node methods plus constructors
// node::node(int x_cord,int y_cord)
// {
//   if((x_cord >=0) && (x_cord <COL) && (y_cord >=0) && (y_cord <ROW))
//   {
//     pos_x = x_cord;
//     pos_y = y_cord;
//   }
//   else
//   Serial.println("Can't add node Outside the limits.");
// }
// int** node::draw_obstacle(int** arr4)
// {
//   arr4[pos_y][pos_x] = 0;
//   return arr4;
// }
// bool operator== (const node& n1, const node& n2) //operator OVerloading for Equal
// {
//     return (n1.pos_x == n2.pos_x &&
//             n1.pos_y == n2.pos_y);
// }
// bool operator!= (const node& n1, const node& n2)//Operator Overloading for Unequal
// {
//     return (n1.pos_x != n2.pos_x ||
//             n1.pos_y != n2.pos_y);
// }
// void print_node(node& param)
// {
//   Serial.print("\tNode(x,y): ");
//   Serial.print(param.getnodecol());
//   Serial.print(" , ");
//   Serial.println(param.getnoderow());
// }
// functions to display array
void printmap(int** arr,int col, int row)
{
  for (int i = 0; i < row; ++i) {
    Serial.print("\t");
    for (int j = 0; j < col; ++j) {
      Serial.print(arr[i][j]);
      if(j < col-1)
        Serial.print(" ");
    }
      Serial.println();
  }
}
void printmap_rev(int** arr,int col, int row)
{
  for (int i = row-1; i >=0; --i) {
    Serial.print("\t");
    for (int j = 0; j < col; ++j) {
      Serial.print(arr[i][j]);
      if(j < col-1)
        Serial.print(" ");
    }
      Serial.println();
  }
}
//create obstacle and put it in  closed list
int** place_obstacle(int** arr1,int posi_x,int posi_y)
{ 
  //node obj(posi_x,posi_y);
  arr1[posi_y][posi_x] = 0;
  return arr1;
}
int** our_obstacles(int** arr3)
{
  arr3 = place_obstacle(arr3,1,3);
  arr3 = place_obstacle(arr3,7,4);
  arr3 = place_obstacle(arr3,12,1);
  arr3 = place_obstacle(arr3,17,2);
  return arr3;
}

/////////////////////////////////////////////
///////// Astar things starts here /////////
///////////////////////////////////////////

//each node has some features
complete_node::complete_node(int x1, int y1,int Px1, int Py1, double gnn1, double hnn1)
{
    //tag = tag1;
    x = x1;
    y = y1;
    Px =  Px1;
    Py = Py1;
    gnn = gnn1;
    hnn = hnn1;
    fnn = gnn + hnn;
}
bool operator== (const complete_node& n1, const complete_node& n2) //operator OVerloading for Equal
{
    return (n1.x == n2.x &&
            n1.y == n2.y);
}
bool operator!= (const complete_node& n1, const complete_node& n2)//Operator Overloading for Unequal
{
    return (n1.x != n2.x ||
            n1.y != n2.y);
}
void print_complete_node(complete_node &param)
{
  Serial.print("     Destination(x,y): ");
  Serial.print(param.getcol());
  Serial.print(" , ");
  Serial.print(param.getrow());
  Serial.print("");
  Serial.print("     Source(x,y): ");
  Serial.print(param.getPcol());
  Serial.print(" , ");
  Serial.print(param.getProw());
  Serial.print("     Gnn: ");
  Serial.print(param.getgnn());
  Serial.print("     Hnn: ");
  Serial.print(param.gethnn());
  Serial.print("     Fnn: ");
  Serial.println(param.getfnn());
}


bool isgoalsourcevalid(int sx, int sy, int gx, int gy, int left_lim, int right_lim)
{
  if((sx >= left_lim) && (sx <= right_lim) && (gx >= left_lim) && (gx <= right_lim) &&
  (sy >= 0) && (sy < ROW ) && (gy >= 0) && (gy < ROW))
  {
    Serial.println("Passed: Source & Goal Valid.");
    return true;
  }
  else
  {
    Serial.println("Failed: Source or Goal InValid.");
    return false;
  }
}
bool issourcegoalsame(int sx, int sy, int gx, int gy)
{
  if((sx == gx) && (sy == gy))
  {
    Serial.println("Failed: Goal and Source same");
    return false;
  }
  else
  {
    Serial.println("Passed: Goal and Source not same");
    return true;
  }
}
int** markgoalandstart(int** arr6,int sx,int sy,int gx,int gy)
{
  arr6[sy][sx] = 7; //start point
  arr6[gy][gx] = 9; //goal point
  Serial.println("Printing Map after Goal and Start Placement : { 7 is Start, 9 is Goal & 0 is obstacle }");printmap(arr6,COL, ROW);
  return arr6;
}
double cost_calculator(double sx, double sy, double gx, double gy)
{
  return sqrt(((gx-sx)*(gx-sx)) + ((gy-sy)*(gy-sy)));
}
double heuristic(double sx, double sy, double gx, double gy)
{
  return sqrt(((gx-sx)*(gx-sx)) + ((gy-sy)*(gy-sy)));
}
bool isdestination(int sx, int sy, int gx, int gy)
{
  if ((sx == gx) && (sy == gy))
  {
    return true;
  }
  else
  {
    return false;
  }
}

priority_queue<complete_node, vector<complete_node>,comparefnn> openedlist;
queue<complete_node> closedlist;



int** trackthepath(int gx, int gy)
{

  int** pathi = getmap(0,2,2);
  return pathi;
}
complete_node existingputar(-1,-1,-1,-1,-1,-1);

bool notinclosed(complete_node &param) //return true if not found in closed
{
  bool out = true;
  int size = closedlist.size();
  if(size >0)
  {
    int i =0;
    while(i<size)
    {
      complete_node mynode = closedlist.front();
      closedlist.pop();
      if((param.getcol() == mynode.getcol())  && (param.getrow() == mynode.getrow()) && out)
      {
        existingputar = param;
        out = false;
      }
      closedlist.emplace(mynode);
      i++;
    }
    return out;
  }
  else
  {
    return out;
  }
}

void updateexistinginclosed(complete_node &param) //upadtes the existing on in closed
{
  int size = closedlist.size();
  if(size >0)
  {
    int i =0;
    while(i<size)
    {
      complete_node mynode = closedlist.front();
      closedlist.pop();
      if(mynode == existingputar)
      {
        mynode = param;
      }
      closedlist.emplace(mynode);
      i++;
    }
  }
  else
  {}
}

bool notinopened(complete_node &param) //return true if not found in open
{
  bool out = true;
  int size = openedlist.size();
  if(size >0)
  {
    queue<complete_node> openedlisttemp;
    while(!openedlist.empty())
    {
      openedlisttemp.emplace(openedlist.top());
      openedlist.pop();
    }
    int i =0;
    while(i<size)
    {
      complete_node mynode = openedlisttemp.front();
      openedlisttemp.pop();
      if((param.getcol() == mynode.getcol())  && (param.getrow() == mynode.getrow()) && out)
      {
        existingputar = param;
        out = false;
      }
      openedlisttemp.emplace(mynode);
      i++;
    }
    while(!openedlisttemp.empty())
    {
      openedlist.emplace(openedlisttemp.front());
      openedlisttemp.pop();
    }
    return out;
  }
  else
  {
    return out;
  }
}

void updateexistinginopened(complete_node &param)
{
  int size = openedlist.size();
  if(size >0)
  {
    queue<complete_node> openedlisttemp;
    while(!openedlist.empty())
    {
      openedlisttemp.emplace(openedlist.top());
      openedlist.pop();
    }
    int i =0;
    while(i<size)
    {
      complete_node mynode = openedlisttemp.front();
      openedlisttemp.pop();
      if(mynode == existingputar)
      {
        mynode = param;
      }
      openedlisttemp.emplace(mynode);
      i++;
    }
    while(!openedlisttemp.empty())
    {
      openedlist.emplace(openedlisttemp.front());
      openedlisttemp.pop();
    }
  }
  else
  {}
}

void generate_successors(int** arr, complete_node& papa,int gx,int gy, int left_lim, int right_lim) //generates the child for the given node and put them back in OPEN
{
  //Serial.println("1");
  double ind_hn = 0;
  int rowi =0;
  int coli = 0;
  double pathgn =0;
  int px = papa.getcol();
  int py = papa.getrow();

  double fnn =0;
  //Serial.println("Expand_Array: ");
  for(int i = 1;i>=-1; --i) //for row iterators in child
  {
    rowi = py+i;
    if((rowi >= 0) && (rowi < ROW  ))//if rows are in -upper and lower limits
    { 
      for(int j=1;j>=-1;--j) //for column iterators
      {
        coli = px+j;
        if((coli >= left_lim) && (coli <= right_lim )) //if columns are in left/right limits
        {
          if((arr[rowi][coli] != 0) && (papa.getcol() != coli) || (papa.getrow() != rowi))
          {
            complete_node putar(coli,rowi,papa.getcol(), papa.getrow(),0,0);
            putar.setcol(coli);putar.setrow(rowi);
            putar.setPcol(papa.getcol());putar.setProw(papa.getrow());
            //print_complete_node(putar);
            //d-i) if successor is goal stop search
            if(isdestination(putar.getcol(),putar.getrow(),gx,gy))
            {
              
              Serial.println("Success: The Goal matched with successor. Hurayyyy!");
              found_dest = true;
            }
            //d-ii) compute both g and h for successor
            else
            { 
              if(notinclosed(putar)) //putar is not in closed
              {
                pathgn =  papa.getgnn() + cost_calculator(px,py,coli,rowi); //succesor current cost
                putar.setgnn(pathgn);
                ind_hn = heuristic(coli,rowi,gx,gy);
                putar.sethnn(ind_hn);
                putar.setfnn(pathgn+ind_hn);

                if(notinopened(putar))
                {
                  //Serial.println("New Child: Adding to Openlist");
                  openedlist.emplace(putar);
                }
                else //if it is in open
                {
                  if(putar.getgnn() < existingputar.getgnn()) //already in open has cost less than new coming
                  {
                    Serial.println("Updating exisiting in opened.");
                    updateexistinginopened(putar);
                  }
                  else
                  {
                    //skip the child
                  }
                }
              }
              else //putar is already in closed
              {
                if(putar.getgnn() < existingputar.getgnn()) //already in close has cost less than new coming
                {
                  //Serial.print("Updating exisiting in closed."<<endl;
                  updateexistinginclosed(putar);
                }
                else
                {
                    //skip the child
                }
              }
            }
          }
          else
          {
            //Serial.println("\tSkipped: Encountered node as Obstacle/Parent.");
          }
        }
        else
        {
          //Serial.println("Skipped : Child in X is outside limit.");
        }
      }
    }
    else
    {
      //Serial.println("Skipped: Child in Y Axis deviated.");
    }
  }
}
void printopenedlist()
{
  Serial.println("Opened List: ");
  int size = openedlist.size();
  if(size >0)
  {
    queue<complete_node> openedlisttemp;
    while(!openedlist.empty())
    {
      openedlisttemp.emplace(openedlist.top());
      openedlist.pop();
    }
    int i =0;
    while(i<size)
    {
      complete_node mynode = openedlisttemp.front();
      openedlisttemp.pop();
      print_complete_node(mynode);
      openedlisttemp.emplace(mynode);
      i++;
    }
    while(!openedlisttemp.empty())
    {
      openedlist.emplace(openedlisttemp.front());
      openedlisttemp.pop();
    }
  }
  else
  {
  }
}
void printclosedlist()
{
Serial.println("Closed List: ");
  int size = closedlist.size();
  if(size >0)
  {
    int i =0;
    while(i<size)
    {
      complete_node mynode = closedlist.front();
      closedlist.pop();
      print_complete_node(mynode);
      closedlist.emplace(mynode);
      i++;
    }
  }
  else
  {
  }
}
void releaseopenedlistmemory()
{
  while(!openedlist.empty())
  {
    openedlist.pop();
  }
  Serial.println("Opened List memory released.")
}
int** Astar(int sx, int sy, int gx, int gy, int**arr5, int left_lim, int right_lim)
{
  Serial.println("A-Star Started...");
  bool gsv = isgoalsourcevalid(sx, sy, gx, gy, left_lim, right_lim); //returns bool
  bool sgs = issourcegoalsame(sx, sy, gx, gy); //returns bool
  if(gsv && sgs)
  {
    int i = 0;
    arr5 = markgoalandstart(arr5,sx,sy,gx,gy);
    //object for start node
    complete_node start(sx,sy,sx,sy,0,heuristic(sx,sy,gx,gy));
    // 1. Initialize the 
    openedlist.emplace(start); //initializing Open list with the start
    while(!found_dest)
    {
      //object for closed list
      //----- ( a ) node to be expanded with least fn
      complete_node parent = openedlist.top(); 
      //----- ( b ) pop the least fn node
      openedlist.pop();
      //----- ( c ) & ( d ) Generate successors & set their parent
      generate_successors(arr5, parent, gx, gy, left_lim, right_lim);//child expanded
      //----- ( e ) push explored to the closed list
      closedlist.emplace(parent); //sending previously expanded node in closed list.

      Serial.print("Iteration# ");
      Serial.println(i++);
    }
      printclosedlist();
      printopenedlist();
  }
  else
  {
    Serial.println("Unsuccessful: Something went Wrong in A-start! Recheck Goal & Source.");
    return getmap(0,1,1);
  }
  if(!found_dest)
  {
    Serial.println("Unsucessful: Goal not found in Map.");
    return getmap(0,1,1);
  }
  else
  {
    Serial.println("Success: A-Star Finished succesfully, Returning Path.");
    int** cpppath = trackthepath(gx,gy);
    return cpppath;
  }
}