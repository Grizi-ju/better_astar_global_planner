/* 
The improved Astar algorithm(planner) applied in ROS
Author：Grizi-ju
Tutorial：https://www.bilibili.com/video/BV1z94y1d7p3?spm_id_from=333.999.0.0
Reference：http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include "better_astar.h"

#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(BAstar_planner::BAstarPlannerROS, nav_core::BaseGlobalPlanner)

int value;
int mapSize;
bool *OGM;
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();
float tBreak; // coefficient for breaking ties
ofstream MyExcelFile("BA_result.xlsx", ios::trunc);

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

inline vector<int> findFreeNeighborCell(int CellID);

//Default Constructor
namespace BAstar_planner
{

  BAstarPlannerROS::BAstarPlannerROS()
  {
  }
  BAstarPlannerROS::BAstarPlannerROS(ros::NodeHandle &nh)
  {
    ROSNodeHandle = nh;
  }

  BAstarPlannerROS::BAstarPlannerROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void BAstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialized_ = false;
    if (!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      _plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      _frame_id = costmap_ros->getGlobalFrameID();

      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();

      width = costmap_->getSizeInCellsX();
      height = costmap_->getSizeInCellsY();
      resolution = costmap_->getResolution();
      mapSize = width * height;
      tBreak = 1 + 1 / (mapSize);
      value = 0;

      OGM = new bool[mapSize];
      for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
      {
        for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
        {
          unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
          //cout<<cost;
          if (cost == 0)
            OGM[iy * width + ix] = true;
          else
            OGM[iy * width + ix] = false;
        }
      }

      MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;

      ROS_INFO("BAstar planner initialized successfully");
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool BAstarPlannerROS::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                  std::vector<geometry_msgs::PoseStamped> &plan)
  {

    if (!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
              goal.pose.position.x, goal.pose.position.y);

    plan.clear();

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);

    // convert the start and goal positions

    float startX = start.pose.position.x;
    float startY = start.pose.position.y;

    float goalX = goal.pose.position.x;
    float goalY = goal.pose.position.y;

    getCorrdinate(startX, startY);
    getCorrdinate(goalX, goalY);

    int startCell;
    int goalCell;

    if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
    {
      startCell = convertToCellIndex(startX, startY);

      goalCell = convertToCellIndex(goalX, goalY);

      MyExcelFile << startCell << "\t" << start.pose.position.x << "\t" << start.pose.position.y << "\t" << goalCell << "\t" << goal.pose.position.x << "\t" << goal.pose.position.y;
    }
    else
    {
      ROS_WARN("the start or goal is out of the map");
      return false;
    }

    /////////////////////////////////////////////////////////

    // call global planner

    if (isStartAndGoalCellsValid(startCell, goalCell))
    {

      vector<int> bestPath;
      bestPath.clear();

      bestPath = BAstarPlanner(startCell, goalCell);

      //if the global planner find a path
      if (bestPath.size() > 0)
      {

        // convert the path

        for (int i = 0; i < bestPath.size(); i++)
        {

          float x = 0.0;
          float y = 0.0;

          int index = bestPath[i];

          convertToCoordinate(index, x, y);

          geometry_msgs::PoseStamped pose = goal;

          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = 0.0;

          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;

          plan.push_back(pose);
        }

        float path_length = 0.0;

        std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();

        geometry_msgs::PoseStamped last_pose;
        last_pose = *it;
        it++;
        for (; it != plan.end(); ++it)
        {
          path_length += hypot((*it).pose.position.x - last_pose.pose.position.x,
                               (*it).pose.position.y - last_pose.pose.position.y);
          last_pose = *it;
        }
        cout << "The global path length: " << path_length << " meters" << endl;
        MyExcelFile << "\t" << path_length << "\t" << plan.size() << endl;
        //publish the plan
        nav_msgs::Path path;
        path.poses.resize(plan.size());

        if (plan.empty())
        {
          //still set a valid frame so visualization won't hit transform issues
          path.header.frame_id = _frame_id;
          path.header.stamp = ros::Time::now();
        }
        else
        {
          path.header.frame_id = plan[0].header.frame_id;
          path.header.stamp = plan[0].header.stamp;
        }

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < plan.size(); i++)
        {
          path.poses[i] = plan[i];
        }

        _plan_pub.publish(path);
        return true;
      }

      else
      {
        ROS_WARN("The planner failed to find a path, choose other goal position");
        return false;
      }
    }

    else
    {
      ROS_WARN("Not valid start or goal");
      return false;
    }
  }
  void BAstarPlannerROS::getCorrdinate(float &x, float &y)
  {

    x = x - originX;
    y = y - originY;
  }

  int BAstarPlannerROS::convertToCellIndex(float x, float y)
  {

    int cellIndex;

    float newX = x / resolution;
    float newY = y / resolution;

    cellIndex = getCellIndex(newY, newX);

    return cellIndex;
  }

  void BAstarPlannerROS::convertToCoordinate(int index, float &x, float &y)
  {

    x = getCellColID(index) * resolution;

    y = getCellRowID(index) * resolution;

    x = x + originX;
    y = y + originY;
  }

  bool BAstarPlannerROS::isCellInsideMap(float x, float y)
  {
    bool valid = true;

    if (x > (width * resolution) || y > (height * resolution))
      valid = false;

    return valid;
  }

  void BAstarPlannerROS::mapToWorld(double mx, double my, double &wx, double &wy)
  {
    costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * resolution;
    wy = costmap->getOriginY() + my * resolution;
  }

  vector<int> BAstarPlannerROS::BAstarPlanner(int startCell, int goalCell)
  {

    vector<int> bestPath;

    //float g_score [mapSize][2];
    float g_score[mapSize];

    for (uint i = 0; i < mapSize; i++)
      g_score[i] = infinity;

    timespec time1, time2;
    /* take current time here */
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

    bestPath = findPath(startCell, goalCell, g_score);

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

    cout << "time to generate best global path by the new planner A* = " << (diff(time1, time2).tv_sec) * 1e3 + (diff(time1, time2).tv_nsec) * 1e-6 << " microseconds" << endl;

    MyExcelFile << "\t" << (diff(time1, time2).tv_sec) * 1e3 + (diff(time1, time2).tv_nsec) * 1e-6;

    return bestPath;
  }

  /*******************************************************************************/
  //Function Name: findPath
  //Inputs: the map layout, the start and the goal Cells and a boolean to indicate if we will use break ties or not
  //Output: the best path
  //Description: it is used to generate the robot free path
  /*********************************************************************************/
  vector<int> BAstarPlannerROS::findPath(int startCell, int goalCell, float g_score[])
  {
    value++;
    vector<int> bestPath;
    vector<int> emptyPath;
    cells CP;

    multiset<cells> OPL;
    int currentCell;

    //calculate g_score and f_score of the start position
    g_score[startCell] = 0;
    CP.currentCell = startCell;
    CP.fCost = g_score[startCell] + calculateHCost(startCell, goalCell);

    //add the start cell to the open list
    OPL.insert(CP);
    currentCell = startCell;

    //while the open list is not empty continuie the search or g_score(goalCell) is equal to infinity
    while (!OPL.empty() && g_score[goalCell] == infinity)
    {
      //choose the cell that has the lowest cost fCost in the open set which is the begin of the multiset
      currentCell = OPL.begin()->currentCell;
      //remove the currentCell from the openList
      OPL.erase(OPL.begin());
      //search the neighbors of the current Cell
      vector<int> neighborCells;
      neighborCells = findFreeNeighborCell(currentCell);
      for (uint i = 0; i < neighborCells.size(); i++) //for each neighbor v of current cell
      {
        // if the g_score of the neighbor is equal to INF: unvisited cell
        if (g_score[neighborCells[i]] == infinity)
        {
          g_score[neighborCells[i]] = g_score[currentCell] + getMoveCost(currentCell, neighborCells[i]);
          addNeighborCellToOpenList(OPL, neighborCells[i], goalCell, g_score);
        } //end if
      }   //end for
    }     //end while

    if (g_score[goalCell] != infinity) // if g_score(goalcell)==INF : construct path
    {
      bestPath = constructPath(startCell, goalCell, g_score);
      return bestPath;
    }
    else
    {
      cout << "Failure to find a path !" << endl;
      return emptyPath;
    }
  }

  /*******************************************************************************/
  //Function Name: constructPath
  //Inputs: the start and the goal Cells
  //Output: the best path
  //Description: it is used to construct the robot path
  /*********************************************************************************/
  vector<int> BAstarPlannerROS::constructPath(int startCell, int goalCell, float g_score[])
  {
    vector<int> bestPath;
    vector<int> path;

    path.insert(path.begin() + bestPath.size(), goalCell);
    int currentCell = goalCell;

    while (currentCell != startCell)
    {
      vector<int> neighborCells;
      neighborCells = findFreeNeighborCell(currentCell);

      vector<float> gScoresNeighbors;
      for (uint i = 0; i < neighborCells.size(); i++)
        gScoresNeighbors.push_back(g_score[neighborCells[i]]);

      int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
      currentCell = neighborCells[posMinGScore];

      //insert the neighbor in the path
      path.insert(path.begin() + path.size(), currentCell);
    }
    for (uint i = 0; i < path.size(); i++)
      bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);

    return bestPath;
  }

  /*******************************************************************************/
  //Function Name: calculateHCost
  //Inputs:the cellID and the goalCell
  //Output: the distance between the current cell and the goal cell
  //Description: it is used to calculate the hCost
  /*********************************************************************************/
  float BAstarPlannerROS::calculateHCost(int cellID, int goalCell)
  {    
    double h;    //heuristic(n)
    double distance;  //distance
    double w;    //weight(n)

    int x1=getCellRowID(goalCell);
    int y1=getCellColID(goalCell);
    int x2=getCellRowID(cellID);
    int y2=getCellColID(cellID);

    w = 3.0;
    //distance = abs(x1-x2)+abs(y1-y2);         //Manhatten Distance
    //distance = sqrt(pow(x1,x2)+pow(y1,y2));       //Euclidean Distance
    distance = min(abs(x1-x2),abs(y1-y2))  *sqrt(2)  + max(abs(x1-x2),abs(y1-y2)) - min(abs(x1-x2),abs(y1-y2));   //Diagonal Distance
    
    h = w * distance;

    return h;    //return h to return 0    meaning that improved A* algorithm to Dijkstra
  }


  /*******************************************************************************/
  //Function Name: addNeighborCellToOpenList
  //Inputs: the open list, the neighbors Cell, the g_score matrix, the goal cell
  //Output:
  //Description: it is used to add a neighbor Cell to the open list
  /*********************************************************************************/
  void BAstarPlannerROS::addNeighborCellToOpenList(multiset<cells> &OPL, int neighborCell, int goalCell, float g_score[])
  {
    cells CP;
    CP.currentCell = neighborCell; //insert the neighbor cell
    CP.fCost = g_score[neighborCell] + calculateHCost(neighborCell, goalCell);
    OPL.insert(CP);
    //multiset<cells>::iterator it = OPL.lower_bound(CP);
    //multiset<cells>::iterator it = OPL.upper_bound(CP);
    //OPL.insert( it, CP  );
  }

  /*******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/

  vector<int> BAstarPlannerROS::findFreeNeighborCell(int CellID)
  {

    int rowID = getCellRowID(CellID);
    int colID = getCellColID(CellID);
    int neighborIndex;
    vector<int> freeNeighborCells;

    for (int i = -1; i <= 1; i++)
      for (int j = -1; j <= 1; j++)
      {
        //check whether the index is valid
        if ((rowID + i >= 0) && (rowID + i < height) && (colID + j >= 0) && (colID + j < width) && (!(i == 0 && j == 0)))
        {
          neighborIndex = getCellIndex(rowID + i, colID + j);
          if (isFree(neighborIndex))
            freeNeighborCells.push_back(neighborIndex);
        }
      }
    return freeNeighborCells;
  }

  /*******************************************************************************/
  //Function Name: isStartAndGoalCellsValid
  //Inputs: the start and Goal cells
  //Output: true if the start and the goal cells are valid
  //Description: check if the start and goal cells are valid
  /*********************************************************************************/
  bool BAstarPlannerROS::isStartAndGoalCellsValid(int startCell, int goalCell)
  {
    bool isvalid = true;
    bool isFreeStartCell = isFree(startCell);
    bool isFreeGoalCell = isFree(goalCell);
    if (startCell == goalCell)
    {
      //cout << "The Start and the Goal cells are the same..." << endl;
      isvalid = false;
    }
    else
    {
      if (!isFreeStartCell && !isFreeGoalCell)
      {
        //cout << "The start and the goal cells are obstacle positions..." << endl;
        isvalid = false;
      }
      else
      {
        if (!isFreeStartCell)
        {
          //cout << "The start is an obstacle..." << endl;
          isvalid = false;
        }
        else
        {
          if (!isFreeGoalCell)
          {
            //cout << "The goal cell is an obstacle..." << endl;
            isvalid = false;
          }
          else
          {
            if (findFreeNeighborCell(goalCell).size() == 0)
            {
              //cout << "The goal cell is encountred by obstacles... "<< endl;
              isvalid = false;
            }
            else
            {
              if (findFreeNeighborCell(startCell).size() == 0)
              {
                //cout << "The start cell is encountred by obstacles... "<< endl;
                isvalid = false;
              }
            }
          }
        }
      }
    }
    return isvalid;
  }

  float BAstarPlannerROS::getMoveCost(int i1, int j1, int i2, int j2)
  {
    float moveCost = INFINIT_COST; //start cost with maximum value. Change it to real cost of cells are connected
    //if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
    if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
    {
      //moveCost = DIAGONAL_MOVE_COST;
      moveCost = 1.4;
    }
    //if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
    else
    {
      if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
      {
        //moveCost = MOVE_COST;
        moveCost = 1;
      }
    }
    return moveCost;
  }

  float BAstarPlannerROS::getMoveCost(int CellID1, int CellID2)
  {
    int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

    i1 = getCellRowID(CellID1);
    j1 = getCellColID(CellID1);
    i2 = getCellRowID(CellID2);
    j2 = getCellColID(CellID2);

    return getMoveCost(i1, j1, i2, j2);
  }

  //verify if the cell(i,j) is free
  bool BAstarPlannerROS::isFree(int i, int j)
  {
    int CellID = getCellIndex(i, j);
    return OGM[CellID];
  }

  //verify if the cell(i,j) is free
  bool BAstarPlannerROS::isFree(int CellID)
  {
    return OGM[CellID];
  }
};

bool operator<(cells const &c1, cells const &c2) { return c1.fCost < c2.fCost; }