#include "swarm_arena.h"
#include <Queue>
using namespace std;

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
node::node(int x_cord,int y_cord)
{
  if((x_cord >=0) && (x_cord <COL) && (y_cord >=0) && (y_cord <ROW))
  {
    pos_x = x_cord;
    pos_y = y_cord;
  }
  else
  Serial.println("Can't add node Outside the limits.");
}
int** node::draw_obstacle_and_boundary(int** arr2)
{
  for(int j=pos_y-2;j<=pos_y+2;++j)
  {
    for(int k=pos_x-2;k<=pos_x+2;++k)
    {
      if(( j >= 0) && ( j <= ROW-1) && ( k >= 0) && ( k <= COL-1))
        if((j==pos_y-2)||(j==pos_y+2)||(k==pos_x-2)||(k==pos_x+2) )
            arr2[j][k]=0;
        else
            arr2[j][k]=9;
      else{
         //Serial.println("Encountered boundary outside map. Skipping");  
         }
    }
  } 
return arr2;
}
int** node::draw_obstacle(int** arr4)
{
  arr4[pos_y][pos_x] = 0;
  return arr4;
}
bool operator== (const node& n1, const node& n2) //operator OVerloading for Equal
{
    return (n1.pos_x == n2.pos_x &&
            n1.pos_y == n2.pos_y);
}
bool operator!= (const node& n1, const node& n2)//Operator Overloading for Unequal
{
    return (n1.pos_x != n2.pos_x ||
            n1.pos_y != n2.pos_y);
}
void print_node(node& param)
{
  Serial.print("\tNode(x,y): ");
  Serial.print(param.getnodecol());
  Serial.print(" , ");
  Serial.println(param.getnoderow());
}

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

queue<node> closedlist;
//create obstacle and generate its boundary
int** place_obstacle(bool close, int** arr1,int posi_x,int posi_y)
{ 
  node obj(posi_x,posi_y);
  arr1 = obj.draw_obstacle(arr1);
  if(close)
  {
    closedlist.emplace(obj);
    //Serial.println("Sending obstacles data to Closed Queue.");
  }
  else
  {
    Serial.println("Could'nt send Obstacles to Closed Queue.");
  }
  
  return arr1;
}
int** our_obstacles(bool close, int** arr3)
{
  arr3 = place_obstacle(close, arr3,1,3);
  arr3 = place_obstacle(close, arr3,7,4);
  arr3 = place_obstacle(close, arr3,12,1);
  arr3 = place_obstacle(close, arr3,17,2);
  return arr3;
}

//////////////////////////
//Astar things starts here
//////////////////////////

//each node has some features
complete_node::complete_node(int tag1, int x1, int y1,int Px1, int Py1, double gnn1, double hnn1)
  {
    tag = tag1;
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
  Serial.print("\tTag: ");
  Serial.print(param.gettag());
  Serial.print("\tDestination(x,y): ");
  Serial.print(param.getcol());
  Serial.print(" , ");
  Serial.print(param.getrow());
  Serial.print("\tSource(x,y): ");
  Serial.print(param.getPcol());
  Serial.print(" , ");
  Serial.print(param.getProw());
  Serial.print("\tGnn: ");
  Serial.print(param.getgnn());
  Serial.print("\tHnn: ");
  Serial.print(param.gethnn());
  Serial.print("\tFnn: ");
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

int** cpppath = getmap(0,2,2);

bool checknodeinclosed(node &temp_putri)
{
  bool res = true;
  node temp_front = closedlist.front();
  closedlist.pop();
  closedlist.emplace(temp_front);
  while(closedlist.front() != temp_front)
  {
    node temp_front2 = closedlist.front();
    closedlist.pop();
    closedlist.emplace(temp_front2);
    if((temp_putri == temp_front2) && res)
    {
      res = false;
    }
    else
    {
      continue;
    } 
  }
  return res;
}

queue<complete_node> exparraylist;
void expand_array(int** arr7, int px, int py,double cum_gn,int gx,int gy, int left_lim, int right_lim) //generates the child for the given node and put them back in OPEN
{

  double ind_hn = 0;
  int rowi =0;
  int coli = 0;
  double pathgn =0;
  bool nnic = false; //node not in closed()
  Serial.println("Expand_Array: ");
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
          node temp_putar(coli,rowi);
          nnic = checknodeinclosed(temp_putar);
          if(nnic)
          {
            pathgn = cum_gn;
            pathgn += cost_calculator(px,py,coli,rowi);
            ind_hn = heuristic(coli,rowi,gx,gy);
            complete_node putar(1,rowi,coli,px,py,pathgn,ind_hn);
            print_complete_node(putar);
            exparraylist.emplace(putar);
          }
          else
          {
            //Serial.println("\tSkipped: Encountered node as Obstacle/Parent/Explored in Closed list.");
          }
        }
        else
        {
          //Serial.println("Skipped : Can't Create a child outside limit.");
        }
      }
    }
    else
    {
      //Serial.println("Skipped: Child in Y Axis deviated.");
    }
  }
}

queue<complete_node> onesopenlist;
queue<complete_node> zerosopenlist;

void printonesopenlist()//prints the Ones Open List
{
  Serial.println("Ones Open List:");
  complete_node temp_front = onesopenlist.front();
  onesopenlist.pop();
  if(onesopenlist.empty())
  {
    onesopenlist.emplace(temp_front);
    print_complete_node(temp_front);
  }
  else
  {
    onesopenlist.emplace(temp_front);
    print_complete_node(temp_front);
    while (onesopenlist.front() != temp_front)
    {
      complete_node temp_front2 = onesopenlist.front();
      onesopenlist.pop();
      onesopenlist.emplace(temp_front2);
      print_complete_node(temp_front2);
    }
  }
}

void printzerosopenlist()//prints the Zeros Open List
{
  Serial.println("Zeros Open List:");
  complete_node temp_front = zerosopenlist.front();
  zerosopenlist.pop();
  if(zerosopenlist.empty())
  {
    zerosopenlist.emplace(temp_front);
    print_complete_node(temp_front);
  }
  else
  {
    zerosopenlist.emplace(temp_front);
    print_complete_node(temp_front);
    while (zerosopenlist.front() != temp_front)
    {
      complete_node temp_front2 = zerosopenlist.front();
      zerosopenlist.pop();
      zerosopenlist.emplace(temp_front2);
      print_complete_node(temp_front2);
    }
  }
}

void printclosedlist()//prints the Closed list 
{
  Serial.println("Closed List:");
  node temp_front = closedlist.front();
  closedlist.pop();
  if(closedlist.empty())
  {
    closedlist.emplace(temp_front);
    print_node(temp_front);
  }
  else
  {
    closedlist.emplace(temp_front);
    print_node(temp_front);
    while (closedlist.front() != temp_front)
    {
      node temp_front2 = closedlist.front();
      closedlist.pop();
      closedlist.emplace(temp_front2);
      print_node(temp_front2);
    }
  }
}

int** Astar(int sx, int sy, int gx, int gy, int**arr5, int left_lim, int right_lim)
{
  Serial.println("A-Star Started.");
  bool gsv = isgoalsourcevalid(sx, sy, gx, gy, left_lim, right_lim); //returns bool
  bool sgs = issourcegoalsame(sx, sy, gx, gy); //returns bool
  if(gsv && sgs)
  {
    arr5 = markgoalandstart(arr5,sx,sy,gx,gy);
    //object for start node
    complete_node start(1,sx,sy,sx,sy,0,heuristic(sx,sy,gx,gy));
    onesopenlist.emplace(start); //initializing Open list with the start
    printonesopenlist();

    //object for closed list
    node open_front(onesopenlist.front().getPcol(),onesopenlist.front().getProw()); 
    closedlist.emplace(open_front); //sending previously expanded node in closed list.
    //expanding the childs for the Toppest Element in onesopenlist
    expand_array(arr5, onesopenlist.front().getcol(),onesopenlist.front().getrow(),onesopenlist.front().getfnn(), gx, gy, left_lim, right_lim);//child expanded
    
    //setting the tag zero for expanded ones and sending them to zeros and removing from ones
    onesopenlist.front().settag(0);
    zerosopenlist.emplace(onesopenlist.front());
    onesopenlist.pop();
    while(!exparraylist.empty()) //updating open with childs
      {
        onesopenlist.emplace(exparraylist.front());
        exparraylist.pop();
      }
    printonesopenlist();
    printzerosopenlist();
    printclosedlist();
  }
  else
  {
    Serial.println("Something went Wrong!");
  }
  return cpppath;
}
