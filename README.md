# RecastNavigation

## 介绍

项目路径：[recastnavigation/recastnavigation: Industry-standard navigation-mesh toolset for games](https://github.com/recastnavigation/recastnavigation)

项目中分成三个模块

1. Recast 核心构建库，将 3D 场景集合体转化为导航网格（NavMesh）
2. Detour 运行时库，提供在生成的导航网格上的路径查询、射线检测等运行时功能
3. DetourCrowd 群体模拟，处理大量 Agent 的移动、碰撞避免和局部导航

从 Demo 来看，当点击构架之后，会触发 handleBuild函数

1. rcCreateHeightfield 生成高度场
2. rcRasterizeTriangles 将三角形光栅化
3. rcBuildCompactHeightfield 压缩高度场
4. rcBuildRegions 生成区域
5. rcBuildContours 提取轮廓
6. rcBuildPolyMesh 生成多边形网格
7. dtCreateNavMesh 将 Recast 生成的数据转换成 Detour 可用的 NavMesh

> 在 RecastRasterization.cpp 中实现光栅化，通过体素来表示 3D 世界

> 在 RecastRegion.cpp 中实现生成区域，使用 分水岭算法(Watershed Transform) 切分区域，保证生成的导航多边形是联通切合理的（Walkshed、Monotone、Layer）

> 在 RecastContour.cpp 中还实现了 轮廓提起，将体素边界转换为矢量多边形，设计轮廓简化算法

关于核心寻路实现在 DetourNavMeshQuery.cpp 文件中

- 查看 findPath 函数，可以看到通过标准 A* 算法实现的寻路
- 查看 findStraightPath 函数，可以查看通过 Funnel Algorithm（漏斗算法/拉绳算法）实现的，用于将 A* 找出的多边形序列拉直成平滑路径

## 核心数据结构

首先，在 RecastNavigation 的世界中，世界垂直地面向上的轴为 Y 轴，水平方向是 X 和 Z 轴

### Recast 模块

主要功能：

- 将任意输入的三角形网格（3D 模型）栅格化为体素
- 过滤不可行走区域
- 构建紧凑高度场
- 划分区域并生成轮廓
- 构建多边形网格（PolyMesh）

**rcConfig**

- 基础网格参数

| 属性名称 | 作用 |
| --- | --- |
| bmin[3], bmax[3] | 世界空间中包围盒的最小点和最大点 |
| cs | Cell Size, 体素在 XZ 平面上的尺寸 |
| ch | Cesll Height, 体素在 Y 轴（高度）上的尺寸 |
| tileSize | Tile 的宽高，用于 DetourTileCache 或构建大型 Tiled NavMesh |
| borderSize | 边界填充尺寸，在生成网格为 Tile 时为瓦片边缘预留的额外体素，确保跨瓦片的 Agent 能够正确连接 |

- Agent 能力设置

| 属性名称 | 作用 |
| --- | --- |
| walkableSlopAngle | 最大可行走的坡度角度 |
| walkeableHeight | 角色可通过的最小高度，比如低矮的管道，玩家无法通过 |
| walkableClimb | 角色能跨越的最大台阶高度 |
| walkeableRadius | 角色半径，用于定义网格边缘的收缩量，Recast 会将可行走的区域从墙壁向内收缩这个距离，保证 Agent 的中心点贴着网格边缘走时，身体不会穿模 |

- 区域与轮廓生成，控制如何将体素转化为多边形，影响网格的复杂度和拓扑

| 属性名称 | 作用 |
| --- | --- |
| minRegionArea | 最小的断开区域面积，剔除那些太小以至于无法通过或者没有意义的孤立可行走小块 |
| mergeRegionArea | 允许合并的最小区域面积，如果一个小于这个值， Recast 会尝试将其合并到临近的较大区域中，以减少不必要的多边形碎片 |
| maxEdgeLen | 轮廓边缘的最大允许长度，强制在长直边上增加顶点，有助于让网格更好的贴合地面。 0 表示禁用 |
| maxSimplificationError | 轮廓简化时的最大偏差，控制多边形边缘与原始体素的吻合程度 |
| maxVertsPerPoly | 每个生成的 NavMesh 多边形的最大顶点数，通常为 6 即六边形网格。这决定了最终网格是全是三角形，还是允许多边形 |

- 细节网格生成

| 属性名称 | 作用 |
| --- | --- |
| detailSampleDist | 采样距离，决定在多边形内部以多大的间距采样高度信息。值越小，越精细 |
| detailSmapleMaxError | 最大高度偏差，如果简化后的细节网格与原始高度长偏差超过此值，会添加更多顶点来修正 |

**rcHeightfield** 动态高度场，表示障碍物空间

**rcCompactHeightfield** 紧凑高度场，表示可通行空间

**rcContourSet** 轮廓集合

**rcPolyMesh** 多边形网格

**rcPolyMeshDetail** 细节网格

### Recast 运行时路径查询模块

主要功能：

- 加载和管理导航网格数据
- 路径查找（A* 算法）
- 导航查询（最近多边形、射线检测等）

| 类/结构 | 说明 |
| --- | --- |
| dtNavMesh | 导航网格，基于瓦片的凸多边形网格 |
| dtNavMeshQuery | 路径查询对象 |
| dtQueryFilter | 多边形过滤器，定义区域代价 |
| dtMeshTile | 导航网格瓦片 |
| dtPoly | 多边形定义 |
| dtOffMeshConnection | 离线网格连接（跳跃点等） |

### DetourCrowd 群体模拟模块

主要功能：

- 多智能体的本地导航行为
- 避障和碰撞避免
- 群体模拟

| 类/结构 | 说明 |
| --- | --- |
| dtCrowd |  群体管理器 |
| dtCrowdAgent |  智能体 |
| dtCrowdAgentParams |  智能体配置参数 |
| dtPathCorridor |  路径走廊 |
| dtObstacleAvoidanceQuery |  避障查询 |
| dtLocalBoundary |  本地边界 |
| dtProximityGrid |  近邻网格 |

### DetourTileCache 瓦片缓存模块

主要功能：

- 导航网格流式加载（适合大型开放世界）
- 动态障碍物支持
- 导航网格的增量更新

| 类/结构 | 说明 |
| --- | --- |
| dtTileCache | 瓦片缓存管理器 |
| dtCompressedTile | 压缩瓦片 |
| dtTileCacheObstacle | 动态障碍物 |
| dtTileCacheAlloc | 内存分配器接口 |
| dtTileCacheCompressor | 压缩器接口 |

## 构建导航网格

从 `Sample_SoloMesh::handleBuild` 函数为入口，当改点击构建之后会触发该函数

执行步骤大致如下

1. 初始化构建配置 `rcConfig`
2. 栅格化输入多边形，创建**高度场**（`rcHeightfield`）
   1. 分配高度场内存 `rcAllocHeightfield()`
   2. 初始化高度场结构 `rcCreateHeightfield()`
   3. 根据坡度，记可行走三角形 `rcMarkWalkableTriangles()`
   4. 将三角形栅格化为体素 `rcRasterizeTriangles()`
3. 过滤可行走表面
   1. 过滤低悬障碍物（台阶、路缘） `rcFilterLowHangingWalkableObstacles()`
   2. 过滤悬崖边缘 `rcFilterLedgeSpans()`
   3. 过滤低净空区域 `rcFilterWalkableLowHeightSpans()`
4. 划分可行走区域，构建**紧凑高度场**（`rcCompactHeightfield`）
   1. 构建紧凑高度场
      1. 分配紧凑高度场内存 `rcAllocCompactHeightfield()`
      2. 构建紧凑高度场（表示开放空间） `rcBuildCompactHeightfield()`
      3. 收缩可行走区域（按智能体半径） `rcErodeWalkableArea()`
      4. 标记自定义凸多边形区域 `rcMarkConvexPolyArea()`
   2. 区域划分，三种算法可选
      1. 分水岭算法(Watershed)，效果最好，速度较慢 `rcBuildDistanceField() + rcBuildRegions()`
      2. 单调算法(Monoton)，速度最快，可能产生细长多边形 `rcBuildRegionsMonotone()`
      3. 分层算法(Layer)，这种方案，适合瓦片地图 `rcBuildLayerRegions()`
5. 追踪并简化区域轮廓（`rcContourSet`）
   1. 分配轮廓集内存 `rcAllocContourSet()`
   2. 从区域构建简化轮廓 `rcBuildContours()` 
6. 从轮廓构建多边形网格（`rcPolyMesh`）
   1. 分配多边形网格内存 `rcAllocPolyMesh()`
   2. 将轮廓三角化为多边形网格 `rcBuildPolyMesh()`
7. 创建细节网格（`rcPolyMeshDetail`）
   1. 分配细节网格内存 `rcAllocPolyMeshDetail()`
   2. 构建带高度细节的网格 `rcBuildPolyMeshDetail()`
8. 创建 Detor 导航网格数据（`dtNavMesh`）
   1. 将 Recast 数据转换为 Detour 格式 `dtCreateNavMeshData()`
   2. 分配导航网格内存 `dtAllocNavMesh()`
   3. 初始化导航网格 `dtNavMesh::init()`
   4. 初始化查询对象 `dtNavMeshQuery::init()`

### rcMarkWalkableTriangles 

根据坡度标记可行走三角形

是导航网格生成流程中的第一个**关键预处理函数**，其核心作用是：根据三角形的坡度角度，标记哪些三角形是可行走的，并且为后续的 `rcRasterizeTriangles` 提供每个三角形的区域表示 (Area ID)

```cpp
void rcMarkWalkableTriangles(
    rcContext* context,           // 上下文（用于日志/计时）
    const float walkableSlopeAngle,  // 可行走的最大坡度角（度）
    const float* verts,           // 顶点数组 [x0,y0,z0, x1,y1,z1, ...]
    const int numVerts,           // 顶点数量
    const int* tris,              // 三角形索引数组 [t0_v0, t0_v1, t0_v2, t1_v0, ...]
    const int numTris,            // 三角形数量
    unsigned char* triAreaIDs     // 输出：每个三角形的区域ID
);
```

核心思想就是：判断一个三角面的法向量与世界向上方向（Y轴）的夹角
- 如果夹角 小于 `walkableSlopeAngle`，则该三角形是可行走的
- 如果夹角 大于等于 `walkableSlopeAngle`，则意味着该面是墙壁/悬崖，是不可行走的 

设三角形的法向量是 N = (Nx, Ny, Nz), 世界向上方向为 U = (0, 1, 0)，那么 N · U = cos(θ) = Nx * 0 + Ny * 1 + Nz * 0 = Ny

所以 cos(θ) = Ny

由于 cos 在 0 ~ 180 是单调递减的，所以 θ < `walkableSlopeAngle` 可以转换为 `cos(θ)` > `cos(walkableSlopeAngle)`，也就是 `Ny` > `cos(walkableSlopeAngle)`

```cpp
const float walkableThr = cosf(walkableSlopeAngle / 180.0f * RC_PI);

for (int i = 0; i < numTris; ++i)
{
    const int* tri = &tris[i * 3];
    // calcTriNormal 计算法线向量 
    calcTriNormal(&verts[tri[0] * 3], &verts[tri[1] * 3], &verts[tri[2] * 3], norm);
    if (norm[1] > walkableThr)
    {
        // 标记序号为 i 的三角形为可行走
        triAreaIDs[i] = RC_WALKABLE_AREA;
    }
}
```

> `calcTriNormal` 的计算比较简单，就是根据传入的三个点，计算两个向量，叉乘得到法线，然后归一化处理即可

### rcRasterizeTriangles

将三角形栅格化为体素

为什么需要将三角形体素化？

游戏中的 3D 场景由任意形状的 **三角形网格** 组成，可能重叠、有缝隙、有不规则的拓扑结构，如果直接在上面做路径规划非常复杂

体素化的目的就是为了将不规则变化为规则的网格、简单的邻接关系、便于分析，更方便的处理重叠、缝隙等几何问题

```cpp
bool rcRasterizeTriangles(
    rcContext* context,           // 上下文（日志/计时）
    const float* verts,           // 顶点数组 [x0,y0,z0, x1,y1,z1, ...]
    const int nv,                 // 顶点数量（未使用）
    const int* tris,              // 三角形索引数组
    const unsigned char* triAreaIDs, // 每个三角形的区域ID（来自 rcMarkWalkableTriangles）
    const int numTris,            // 三角形数量
    rcHeightfield& heightfield,   // 输出：高度场
    const int flagMergeThreshold  // Span 合并阈值（通常是 walkableClimb）
);
```

关于 `rcHeightfield` 用于存储高度场

```cpp
struct rcHeightfield {
    int width;          // X 方向单元格数量
    int height;         // Z 方向单元格数量
    float bmin[3];      // 包围盒最小点
    float bmax[3];      // 包围盒最大点
    float cs;           // Cell Size (XZ平面单元格大小)
    float ch;           // Cell Height (Y方向单元格高度)
    rcSpan** spans;     // 二维数组，每个单元格的 Span 链表
};

struct rcSpan {
    unsigned int smin : 13;  // Span 底部高度索引
    unsigned int smax : 13;  // Span 顶部高度索引
    unsigned int area : 6;   // 区域类型 ID
    rcSpan* next;            // 同一列中下一个 Span
};
```

一个 (x, z) 单元格可以有多个垂直方向的 Span

> 垂直地面的轴是 y 轴，别忘了

```cpp
      Y (高度)
      │
      │    ┌───┐ Span 2 (smin=8, smax=10, area=1)
      │    └───┘
      │
      │    ┌───┐ Span 1 (smin=3, smax=5, area=1)
      │    └───┘
      │
      │    ┌───┐ Span 0 (smin=0, smax=2, area=1)
      ├────┴───┴────────► X
     ╱
    Z

    一个 (x,z) 单元格可以有多个垂直方向的 Span
    （用于表示多层结构，如桥梁下方的道路）
```

真正实现体素化的函数是 `rasterizeTri`，该函数接收三角形的三个顶点坐标

1. 通过 AABB 包围盒检查，快速剔除不在高度场中的 三角形
2. 沿 Z 轴切片，然后言责 X 轴切片，将三角形切片成多个小块
3. 对于落入诶个单元格的多边形片段，计算其 Y 坐标范围
4. 通过 addSpan 添加并合并到 Span

> 通过 Sutherland-Hodgman 多边形裁剪算法，对三角形进行切割

```
合并前：                           合并后：
         │                                │
         │   ┌───┐ 新 Span                │   ┌───┐
         │   └───┘                        │   │   │
         │   ┌───┐ 旧 Span 1     ──►      │   │   │ 合并后的 Span
         │   └───┘                        │   │   │
         │   ┌───┐ 旧 Span 2              │   └───┘
         │   └───┘                        │
         └──────────                      └──────────

三个重叠的 Span 被合并成一个大 Span
```

```cpp
static bool rasterizeTri(
    const float* v0, const float* v1, const float* v2,  // 三角形三个顶点
    const unsigned char areaID,           // 区域标识
    rcHeightfield& heightfield,           // 输出高度场
    const float* heightfieldBBMin,        // 高度场包围盒最小点
    const float* heightfieldBBMax,        // 高度场包围盒最大点
    const float cellSize,                 // 单元格大小 (XZ平面)
    const float inverseCellSize,          // 1.0 / cellSize
    const float inverseCellHeight,        // 1.0 / cellHeight
    const int flagMergeThreshold          // Span合并阈值
);
```

核心思路是将三角形逐步裁剪成与网格单元格对其的小多边形片段，具体的裁剪是通过 `dividePoly` 计算得到的

```cpp
static void dividePoly(
    const float* inVerts, int inVertsCount,    // 输入多边形
    float* outVerts1, int* outVerts1Count,     // 输出：分割线一侧的多边形
    float* outVerts2, int* outVerts2Count,     // 输出：分割线另一侧的多边形
    float axisOffset,                          // 分割线位置
    rcAxis axis                                // 分割轴 (X, Y, 或 Z)
);
```

大概示意图如下

```
         Z 轴
         │
         │      A (z=3)
      3 ─┼─────╱╲
         │    ╱  ╲
  分割线 ──┼───P────Q───  z = 2.0 (axisOffset)
      2 ─┼  ╱      ╲
         │ ╱        ╲
      1 ─┼B──────────C (z=1)
         │
         └─────────────► X 轴

原始三角形: A(z=3), B(z=1), C(z=1)

dividePoly 输出:
├── outVerts1 (z >= 2): [A, P, Q]     ← 分割线上方
└── outVerts2 (z < 2):  [P, B, C, Q]  ← 分割线下方

P, Q 是分割线与三角形边的交点
```

### rcFilterLowHangingWalkableObstacles

该函数主要是为了处理当一个较薄的障碍物（如路缘石、台阶、小石头）位于可行走地面上方且高度在 walkableClimb 范围内时，应该将这个障碍物的顶面也标记为可行走

```
场景1: 台阶/路缘石
                          
    ┌───────┐             
    │ 障碍物 │ ← 被标记为不可行走 (因为坡度太陡)
    └───────┘             
════════════════ ← 地面 (可行走)

问题: 如果 Agent 可以"踏上"这个低矮障碍物，
      它应该被标记为可行走！
```

```
场景2: 体素化后的高度场视图

    Y (高度)
    │
  5 ├─────────
    │   ┌───┐ Span B (smax=4, area=NULL) ← 不可行走的障碍物顶面
  4 ├───┤   │
    │   └───┘
  3 ├─────────
    │   ┌───┐ Span A (smax=2, area=WALKABLE) ← 可行走的地面
  2 ├───┤   │
    │   └───┘
  1 ├─────────
    └─────────► (x, z) 单元格

如果 Span B 的顶部 (smax=4) 与 Span A 的顶部 (smax=2) 差值 <= walkableClimb,
那么 Agent 应该能"爬上去"，所以 Span B 也应该标记为可行走！
```

函数参数如下

```cpp
void rcFilterLowHangingWalkableObstacles(
    rcContext* context,       // 上下文
    const int walkableClimb,  // 最大可攀爬高度 (单位: 体素格)
    rcHeightfield& heightfield // 高度场
);
```

遍历所有的单元格，对单元格的 Span 进行遍历，如果当前 Span 不可行走，但是下方的 Span 可行走，并且高度差小于等于 walkableClimb ，那么标记当前 Span 为可行走

```cpp
if (!walkable && previousWasWalkable && (int)span->smax - (int)previousSpan->smax <= walkableClimb)
{
    span->area = previousAreaID;
}

// 保留原始值
previousWasWalkable = walkable;
previousAreaID = span->area;
```

为什么这里要保存 previousWasWalkable 原始值？是为了防止错误传播，针对的就是下面这种情况

```
错误情况（如果用修改后的值）:

    │   ┌───┐ Span C (area=NULL)
  6 ├───┤   │
    │   └───┘
    │   ┌───┐ Span B (area=NULL → WALKABLE)
  4 ├───┤   │
    │   └───┘
    │   ┌───┐ Span A (area=WALKABLE)
  2 ├───┤   │

处理 Span B: A 可行走 + 高度差=2 ≤ walkableClimb → B 变可行走 ✓
处理 Span C: 如果用修改后的 B(可行走) → C 也会变可行走 ✗ (错误!)

正确做法：使用 B 的原始状态（不可行走），C 不会被错误标记
```

### rcFilterLedgeSpans

过滤悬崖边缘，将靠近悬崖/陡坡边缘的 Span 标记为不可行走

即使一个 Span 的表面本身是平坦可行走的，如果它的邻居存在大的高度落差（悬崖），Agent 站在这里会有掉落风险，因此应标记为不可行走

```cpp
void rcFilterLedgeSpans(
    rcContext* context,
    const int walkableHeight,  // Agent 身高 (单位: 体素格)
    const int walkableClimb,   // 最大可攀爬高度 (单位: 体素格)
    rcHeightfield& heightfield
);
```

- 判断悬崖的三种情况
  - 邻居在网格边界外
  - 邻居完全空旷
  - 邻居地面高度差过大
- 陡坡检测
  - 左右邻居可达，但是左右邻居之前的高度差超过阈值

做 悬崖 和 陡坡 检测，主要是为了 安全性 和 真实性

当 Agent 通过 导航网格 移动时，如果途径悬崖可能会掉落，如果提前过滤掉悬崖附近的网格，Agent 移动过程更加安全，不存在掉落悬崖的风险

### rcFilterWalkableLowHeightSpans

将头顶空间不足的 Span 标记为不可行走

即使一个 Span 的地面是平坦的、不在悬崖边缘，如果它上方的空间不足以让 Agent 站立，那么它也不应该被标记为可行走

```cpp
void rcFilterWalkableLowHeightSpans(
    rcContext* context,
    const int walkableHeight,  // Agent 身高 (单位: 体素格数)
    rcHeightfield& heightfield
);
```

```
    Y 轴 (高度)
    │
    │   ┌─────────────┐ ← 上方 Span (span->next)
    │   │  障碍物     │
    │   │             │
    │   └─────────────┘ ← span->next->smin = ceiling
    │                     
    │         ↕ 头顶空间 (ceiling - floor)
    │                     
    │   ╔═════════════╗ ← span->smax = floor
    │   ║  当前 Span   ║
    │   ║             ║
    │   ╚═════════════╝ ← span->smin
    │
    └─────────────────────►

floor   = span->smax        (Agent 脚踩的地面高度)
ceiling = span->next->smin  (头顶障碍物的底部高度)
头顶空间 = ceiling - floor
```

所以代码的判断逻辑很简单

```cpp
const int floor = (int)(span->smax);
const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;
if (ceiling - floor < walkableHeight)
{
    span->area = RC_NULL_AREA;  // 空间不足，不可行走
}
```

### rcBuildCompactHeightfield

将稀疏的链表式高度场（rcHeightfield）转换为紧凑的数组式高度场（rcCompactHeightfield），并建立邻接关系

稀疏链表式高度场存在一些问题

1. 链表遍历效率低
2. 内存分散
3. 无法直接索引第 N 个 Span
4. 没有邻接信息

紧凑数组式高度场的优势

1. 数组顺序访问，缓存友好
2. 内存连续，效率高
3. 可以直接索引任意 Span
4. 内置临界关系，方便后续区域划分

```cpp
// 紧凑高度场
struct rcCompactHeightfield {
    rcCompactCell* cells;    // 单元格数组
    rcCompactSpan* spans;    // Span 数组（连续存储）
    unsigned char* areas;    // 区域 ID 数组
    // ...
};

struct rcCompactCell {
    unsigned int index : 24;  // 该单元格第一个 Span 在 spans 数组中的索引
    unsigned int count : 8;   // 该单元格的 Span 数量
};

struct rcCompactSpan {
    unsigned short y;         // Span 底部高度
    unsigned char h;          // Span 高度（头顶空间）
    unsigned int con : 24;    // 4个方向的邻接信息 (每个6位)
};
```

`rcBuildCompactHeightfield` 函数定义如下

```cpp
bool rcBuildCompactHeightfield(
    rcContext* context,
    const int walkableHeight,  // Agent 身高 (体素格)
    const int walkableClimb,   // 最大可攀爬高度 (体素格)
    const rcHeightfield& heightfield,       // 输入: 原始高度场
    rcCompactHeightfield& compactHeightfield // 输出: 紧凑高度场
);
```

从 稀疏链表式高度场 转化为 紧凑数组式高度场 分为两步

1. 统计与复制 Span
2. 建立邻接关系

每个 rcCompactSpan 有一个 24 位的 con 字段，存储 4 个方向的邻接信息，每个方向 6 位

> `RC_NOT_CONNECTED` 值为 63，表示不连接；其他 0 ~ 62 记录邻居单元格中 Span 的索引 

两个单元格之间可以建立连接需要满足

1. 有足够的通行距离
2. 高度差可攀爬

### rcErodeWalkableArea

收缩可行走区域（按智能体半径）

将可行走区域向内收缩（腐蚀），以便为 Agent 的半径预留空间

这是一个形态学腐蚀操作，确保生成的导航网格与障碍物之间保持 Agent 半径的安全距离。

```cpp
bool rcErodeWalkableArea(
    rcContext* context,
    const int erosionRadius,                    // 腐蚀半径 (通常 = Agent 半径 / cellSize)
    rcCompactHeightfield& compactHeightfield    // 输入/输出: 紧凑高度场
);
```

计算过程分为 3 步

1. 标记边界，遍历所有的 Span
2. 计算距离场
3. 应用腐蚀

### rcMarkConvexPolyArea

标记自定义凸多边形区域

### rcBuildDistanceField

分水岭算法(Watershed)，效果最好，速度较慢

### rcBuildRegionsMonotone

单调算法(Monoton)，速度最快，可能产生细长多边形

### rcBuildLayerRegions

分层算法(Layer)，这种方案，适合瓦片地图

### rcBuildContours

从区域构建简化轮廓

### rcBuildPolyMesh

从轮廓构建多边形网格

### rcBuildPolyMeshDetail

构建带高度细节的网格

### dtCreateNavMeshData

将 Recast 数据转换为 Detour 格式

### dtNavMesh::init

初始化导航网格

### dtNavMeshQuery::init

初始化查询对象

## 寻路




## 群体避障



