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
| walkableClim | 角色能跨越的最大台阶高度 |
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

### rcRasterizeTriangles

将三角形栅格化为体素

### rcFilterLowHangingWalkableObstacles

过滤低悬障碍物（台阶、路缘）

### rcFilterLedgeSpans

过滤悬崖边缘

### rcFilterWalkableLowHeightSpans

过滤低净空区域

### rcBuildCompactHeightfield

构建紧凑高度场

### rcErodeWalkableArea

收缩可行走区域（按智能体半径）

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



