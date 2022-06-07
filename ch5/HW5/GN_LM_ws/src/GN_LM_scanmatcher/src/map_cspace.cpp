/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "map.h"

class CellData
{
  public:
    map_t* map_;
    unsigned int i_, j_;             // 地图中该栅格的x,y
    unsigned int src_i_, src_j_;     // 障碍物的x,y
};

/*
 * 这里的CachedDistanceMap实际上是一个以(0,0)为中心的局部距离地图
 * 这个地图的每一个点都存储着该点的坐标到(0,0)的欧几里得距离。
 * 这里的CacheDistanceMap是用来计算栅格离障碍物距离的时候减少计算量的。
 * 相当于开始进行一次计算的话，后面只要查表就可以计算出来距离，而不需要反复的调用sqrt来求解
*/
class CachedDistanceMap
{
  public:
    CachedDistanceMap(double scale, double max_dist) :
      distances_(NULL), scale_(scale), max_dist_(max_dist)
    {
      //最大距离对应的cell的个数
      cell_radius_ = max_dist / scale;

      // 二维数组指针 distances_: i为行， j为列
      distances_ = new double *[cell_radius_+2];
      for(int i=0; i<=cell_radius_+1; i++)
      {
        distances_[i] = new double[cell_radius_+2];
        for(int j=0; j<=cell_radius_+1; j++)
        {
            distances_[i][j] = sqrt(i*i + j*j);
        }
      }
    }

    ~CachedDistanceMap()
    {
      if(distances_)
      {
        for(int i=0; i<=cell_radius_+1; i++)
            delete[] distances_[i];    // 先delete行
        delete[] distances_;           // 再delete列
      }
    }
    double** distances_;
    double scale_;
    double max_dist_;
    int cell_radius_;
};

// 重载 < 运算符，定义优先级队列排序方式，这里是小顶堆，即距离越近的栅格越先取出来
bool operator<(const CellData& a, const CellData& b)
{
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}

CachedDistanceMap* get_distance_map(double scale, double max_dist)
{
  static CachedDistanceMap* cdm = NULL;     // 静态变量，因为我们自始至终只需要维护一张地图

  if(!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist))
  {
    if(cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist);
  }

  return cdm;
}

//计算到障碍物的距离
/**
 * @brief enqueue
 * @param map       对应的地图
 * @param i         该点对应的x坐标
 * @param j         该点对应的y坐标
 * @param src_i     该点对应的障碍物的x坐标
 * @param src_j     该点对应的障碍物的y坐标
 * @param Q
 * @param cdm
 * @param marked
 */
void enqueue(map_t* map, int i, int j,
	     int src_i, int src_j,
	     std::priority_queue<CellData>& Q,
	     CachedDistanceMap* cdm,
	     unsigned char* marked)
{
  //如果已经被计算过了 则直接返回
  if(marked[MAP_INDEX(map, i, j)])
    return;

  //这里的距离是栅格的距离
  unsigned int di = abs(i - src_i);
  unsigned int dj = abs(j - src_j);
  double distance = cdm->distances_[di][dj];      // 使用搜索的方式获得平方距离，而不是直接计算 sqrt(di*di + dj*dj)

  if(distance > cdm->cell_radius_)                // 如果大于障碍物最大作用距离，就直接返回
    return;

  //转换为实际距离
  map->cells[MAP_INDEX(map, i, j)].occ_dist = distance * map->resolution;

  double z = map->cells[MAP_INDEX(map,i,j)].occ_dist;
  // score是高斯的
  map->cells[MAP_INDEX(map, i, j)].score = exp(-(z * z) /  (2 * map->likelihood_sigma * map->likelihood_sigma));

  CellData cell;
  cell.map_ = map;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;      // 令障碍物的位置保持不变，去改变点的位置，直到该点离障碍物足够远或该点已经被遍历过

  Q.push(cell);

  marked[MAP_INDEX(map, i, j)] = 1;
}


// Update the cspace distance values

/**
 * @brief map_update_cspace
 * 更新地图的距离值 这个函数用来计算用来定位的地图中的每个栅格到最近障碍物的距离
 * 其中障碍物的栅格的距离为0 然后通过dfs进行搜索来计算每一个栅格到障碍物的距离
 * @param map
 * @param max_occ_dist
 */
void map_update_cspace(map_t *map, double max_occ_dist)
{
  unsigned char* marked;
  // 创建优先级队列，来放栅格数据CellData。每次我们从该优先级队列中取出栅格数据的时候，都会先取最大的栅格
  // 栅格数据的大小是我们自己定义的，越大说明离障碍物的距离越小
  // 为此我们重载了<符号，见 bool operator<(const CellData& a, const CellData& b)
  // 距离障碍物越近的栅格越先取出来，以这种方式取出栅格，可以保证这里栅格的occ_dist是离最近障碍物的距离，这样才能保证我们最后势场的结果是最正确的
  std::priority_queue<CellData> Q;       // see: c++优先队列(priority_queue)用法详解 https://blog.csdn.net/weixin_36888577/article/details/79937886

  // marked: 标志栅格有没有被访问过，一旦被访问过，就做下标记，之后就不必访问了
  marked = new unsigned char[map->size_x*map->size_y];
  // memset：char型初始化函数 头文件：<string.h> 或 <memory.h>
  // 函数原型：void *memset(void *s , int ch , size_t  n )
  // 函数解释：将s中的前n个字节用ch替换并且返回s    常用于数组清空  see: https://blog.csdn.net/dan15188387481/article/details/49621447
  memset(marked, 0, sizeof(unsigned char) * map->size_x*map->size_y);

  map->max_occ_dist = max_occ_dist;

  //得到一个CachedDistanceMap    只计算一次栅格和栅格之间的距离，后面只需要查表就可以得到距离了，节省计算资源
  CachedDistanceMap* cdm = get_distance_map(map->resolution, map->max_occ_dist);

  //这个sigma已经在外面设置过了 在handmapmsg里面就会设置
  map->min_score = exp(-max_occ_dist * max_occ_dist / (2 * map->likelihood_sigma * map->likelihood_sigma));

  // Enqueue all the obstacle cells
  // 所有的障碍物都放入队列中
  CellData cell;
  cell.map_ = map;

  //计算出来所有的边界障碍物 只有边界障碍物才用来进行匹配 其他的障碍物都当成no-information

    /*所有障碍物的栅格  离障碍物的距离都标志为0  非障碍物的栅格都标记为max_occ_dist*/
    for(int i=0; i<map->size_x; i++)
    {
        cell.src_i_ = cell.i_ = i;         // scr_i_: 该点对应的障碍物坐标  i_: 该点对应的坐标
        for(int j=0; j<map->size_y; j++)
        {
            if(map->cells[MAP_INDEX(map, i, j)].occ_state == CELL_STATUS_OCC)
            {
                map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;    // 我们认为有激光点的栅格离障碍物的距离为0
                map->cells[MAP_INDEX(map,i,j)].score = 1.0;         // 将所有有障碍物的栅格的score设为最大，即1
                cell.src_j_ = cell.j_ = j;
                marked[MAP_INDEX(map, i, j)] = 1;                   // 将marked[]标为1，表示已经访问过了
                Q.push(cell);                                       // 然后将其放到优先级队列当中
            }
            else
                map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
        }
    }

  // 取出栅格，进行宽度优先遍历
  while(!Q.empty())
  {
    CellData current_cell = Q.top();

    /*往上、下、左、右四个方向拓展*/
    // 如果存在数据，就放入队列，去计算它离障碍物的距离和score
    if(current_cell.i_ > 0)
      enqueue(map, current_cell.i_-1, current_cell.j_,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if(current_cell.j_ > 0)
      enqueue(map, current_cell.i_, current_cell.j_-1,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if((int)current_cell.i_ < map->size_x - 1)
      enqueue(map, current_cell.i_+1, current_cell.j_,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if((int)current_cell.j_ < map->size_y - 1)
      enqueue(map, current_cell.i_, current_cell.j_+1,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    Q.pop();
  }

  delete[] marked;
}
