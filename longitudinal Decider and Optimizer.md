PATH_BOUNDS_DECIDER --> OSQP_PATH_OPTIMIZER --> 五个模块

## 1 SPEED_BOUNDS_PRIORI_DECIDER

### 	1.1 SPEED_BOUNDS_PRIORI_DECIDER（主函数）

```
Process(PilotCarRoadInfo const& state,
                                   PilotPlanRoute&         route,
                                   ReferenceLineInfo&      reference_line_info)
 作用：将上游信息提取出来，过滤掉多余障碍物
 输入：车辆状态信息(state) , 规划路径(route) , 参考线信息(reference_line_info)
 输出：重载的下面Process()
```

```
Status SpeedBoundsDecider::Process(PathData const&                                   path_data,
                                   common::PathPoint const&                          planning_init_point,
                                   ReferenceLine const&                              reference_line,
                                   ObstacleQuerier const&                            obstacle_querier,
                                   std::shared_ptr<const Obstacle>                   follow_obstacle,
                                   float64_t const                                   stage_cruise_speed,
                                   UserSetting const&                                user_setting,
                                   std::shared_ptr<const HmiCommand>                 hmi_cmd,
                                   SpeedData&                                        speed_data,
                                   STGraph&                                          st_graph,
                                   STSemantic&                                       st_semantics,
                                   PilotSpeedLimitInfo&                              pilot_speed_limit_info,
                                   holo::pilot::PlanningState&                       planning_state,
                                   std::shared_ptr<CompetitorContainer const> const& competitor_ptr)
作用：主函数
			  1. 确定s，t范围、重置st语义、在初次决策中清除纵向回溯状态
			  2. 沿离散路径计算速度限制
			  3. 确定离散边界
			  4. 将关注的障碍物映射到st图
			  5. 为需要避让障碍物和静态障碍物打标签
			  6. 计算纵向fallback distance.
			  7. 建立st图
输入：
输出：status::OK()
```

```
bool_t SpeedBoundsDecider::determineDecisionHorizon(SpeedLimit const& speed_limits,
                                                    float64_t const   total_path_data_length,
                                                    float64_t&        decision_horizon) const
作用：辅助函数，判断决策边界, 确定t (decision_horizon)
输入：速度限制(speed_limits), 路径长度(total_path_data_length), 决策边际(decision_horizon一般为时间)
输出：决策边界是否超出范围

decision_horizon被赋值两次
```

```
bool_t SpeedBoundsDecider::checkStandstillCondition(Obstacle const&          obstacle,
                                                    common::PathPoint const& planning_init_point) const
作用：辅助函数，判断自车是否需要停车
输入：投影在参考线上的障碍物(obstacle) , 规划起始点(planning_init_point)
输出：
```

```
bool_t SpeedBoundsDecider::updateDriveoffRequestState(std::shared_ptr<const HmiCommand> hmi_cmd)
作用：辅助函数，更新停车要求状态
输入：
输出：
```

```
bool_t SpeedBoundsDecider::updateSafeStopTrigger(std::shared_ptr<const HmiCommand> hmi_cmd, SpeedData& speed_data)
作用：辅助函数，更新安全停车指令
输入：hmi_cmd ,  speed点
输出：
```

```
float64_t SpeedBoundsDecider::setSpeedFallbackDistance(ObstacleQuerier const& obstacle_querier) const
作用：设置引导速度回退的最小距离，分为对向距离和同向距离
输入：关注的障碍物信息(设置引导速度回退的最小距离)
输出：fallback distance距离
```

```
void SpeedBoundsDecider::ignoreCurrentLaneBackObstacle(ReferenceLineInfo const& reference_line_info) const
作用：辅助函数，过滤当前自车所在道路自车后方障碍物
输入：参考线信息(reference_line_info)
输出：
```

```
void SpeedBoundsDecider::ignoreObstacle(ReferenceLineInfo const& reference_line_info, uint32_t id) const
作用：辅助函数，过滤障碍物 , 将横纵向决策都设为ignore，过滤后方行人
输入：参考线信息(reference_line_info) , 障碍物ID(id)
输出：
```

```
void SpeedBoundsDecider::printDebugInfo(STGraph const& st_graph) const
作用：辅助函数，输出调试信息
输入：ST图(st_graph)
输出：
```

```
void SpeedBoundsDecider::calcStaticPlanSpeed(UserSetting const&   user_setting,
                                             PilotSpeedLimitInfo& speed_limit_info) const
作用：辅助函数，计算static规划速度
输入：用户设置行车状态(user_setting)，限速信息(speed_limit_info)
输出：
```

```
void SpeedBoundsDecider::setAlarmCode(common::PathPoint const&        planning_init_point,
                                      std::shared_ptr<const Obstacle> follow_obstacle,
                                      holo::pilot::PlanningState&     planning_state) const
 作用：辅助函数，设置报警代码对象，验证代码可行性？
 输入：
 输出：
```

### 	1.2 speed_limit_decider

```
Status SpeedLimitDecider::GetSpeedLimits(ObstacleQuerier const& obstacle_querier, SpeedLimit& speed_limit_data) const
作用：主函数，获得最佳路径(路径规划出的路径)的速度限制。
				1.根据智能指令限制速度。
                2.根据场景和用户指挥官限制速度。
                3.根据平行驱动情况限制速度。
                4.根据参考和路径横向加速度约束限制速度。
                5.根据通行空间限制速度。
                6.根据障碍物相互作用限制速度。
                7.根据车辆纵向加速度和减速度限制速度。
                8.根据车辆纵向跳动限制速度。
                9.根据tsr限制速度
输入：规划路径上的障碍物(obstacle_querier), 限速数据(speed_limit_data)
输出：
```

```
bool_t SpeedLimitDecider::limitIntelligentInstruction(SpeedLimit& speed_limit_data, std::string& msg) const
作用：辅助函数，根据智能指令限制速度。
输入：
输出：
```

```
void SpeedLimitDecider::limitScenarioCommander(SpeedLimit&     speed_limit_data,
                                               float64_t const planning_init_speed,
                                               float64_t const max_lon_vel,
                                               float64_t const stage_cruise_speed,
                                               float64_t const user_setting_speed) const
作用：辅助函数
输入：
输出：
```

```
void SpeedLimitDecider::limitTSR(SpeedLimit& speed_limit_data, float64_t const tsr_speed) const
作用：辅助函数
输入：
输出：
```

```
void SpeedLimitDecider::limitParallelDriving(SpeedLimit&            speed_limit_data,
                                             ObstacleQuerier const& obstacle_querier) const
作用：辅助函数
输入：
输出：
```

```
static std::vector<std::pair<float64_t, float64_t>> calcCurveSpeedLimitTable(float64_t const max_lat_acc)
作用：辅助函数
输入：
输出：
```

```
void SpeedLimitDecider::limitLateralAcceleration(SpeedLimit& speed_limit_data, float64_t const max_lat_acc) const
作用：辅助函数
输入：
输出：
```

```
void SpeedLimitDecider::limitPassableSpace(SpeedLimit& speed_limit_data) const
作用：辅助函数
输入：
输出：
```

```
void SpeedLimitDecider::limitObstacleInteraction(SpeedLimit&            speed_limit_data,
                                                 ObstacleQuerier const& obstacle_querier) const
作用：辅助函数
输入：
输出：
```

```
void SpeedLimitDecider::limitLongitudinalAcceleration(SpeedLimit&     speed_limit_data,
                                                      float64_t const max_lon_acc,
                                                      float64_t const max_lon_dec) const
作用：辅助函数
输入：
输出：
```

```
void SpeedLimitDecider::limitLongitudinalJerk(SpeedLimit& speed_limit_data, float64_t const max_lon_jerk) const
作用：辅助函数
输入：
输出：
```

```
bool_t SpeedLimitDecider::checkExitCondition(SpeedLimit const& speed_limit_init,
                                             SpeedLimit const& speed_limit_final) const
作用：辅助函数
输入：
输出：
```

```
bool_t SpeedLimitDecider::addFusedStaticSpeedLimitSTSemantics(SpeedLimit const& speed_limit_final) const
作用：辅助函数
输入：
输出：
```



### 	1.3 st_boundary_mapper

```
Status STBoundaryMapper::ComputeSTBoundary(ObstacleQuerier const& obstacle_querier) const
作用：主函数，依据上游规划路径获取所有道路参与者ST图边界
			  1. 如果对障碍物尚未做出纵向决策，则将其绘制到ST图上。
			  2. 如果障碍物已经有纵向决策(5种)，则微调st边界。
输入：所有障碍物信息(obstacle_querier)
输出：
```

```
bool_t STBoundaryMapper::computeSTBoundaryWithoutDecision(Obstacle& obstacle) const
作用：辅助函数，获取没有决策信息的障碍物在st图上的upper和lower点。
输入：某个障碍物信息(obstacle)
输出：
```

```
bool_t STBoundaryMapper::computeSTBoundaryWithDecision(Obstacle& obstacle, ObstacleDecision const& decision) const
作用：辅助函数，在ST图中获取并调整障碍物的上下界点
输入：障碍物信息，和对其的决策
输出：
```

```
bool_t STBoundaryMapper::mapStopDecision(Obstacle& stop_obstacle, ObstacleDecision const& decision) const
作用：辅助函数，
输入：
输出：
```

```
bool_t STBoundaryMapper::addAgentSemantics(Obstacle const& obstacle) const
作用：辅助函数，添加智能体语义follow yield stop overtake unknown 
输入：障碍物信息(obstacle)
输出：
```

```
bool_t STBoundaryMapper::getOverlapBoundaryPoints(Obstacle const&       obstacle,
                                                  std::vector<STPoint>& lower_points,
                                                  std::vector<STPoint>& upper_points) const
作用：辅助函数，将给定对象映射到ST图形。 
输入：障碍物信息(取obstacle参考路径信息)，st图的上下边界点(lower_points, upper_points)
输出：
```

```
float64_t STBoundaryMapper::binarySearchOverlapS(bool_t                 forward,
                                                 float64_t              start,
                                                 float64_t              end,
                                                 float64_t              resolution,
                                                 DiscretizedPath const& sampled_discrete_path,
                                                 Box2d const&           obs_box,
                                                 float64_t const        l_buffer) const
作用：辅助函数，用二值搜索法查找输入障碍物的上下重叠
输入：前后点的s是搜索目标(forward), 二分搜索的下边界(start), 二分搜索的上边界(end),  精确结果(resolution)， 自车离散轨迹(sampled_discrete_path),  障碍物的二维矩形(obs_box),  用于重叠检查的横向缓冲器(l_buffer)
输出：返回重叠的s
```

```
bool_t STBoundaryMapper::searchBidirectional(DiscretizedPath const& sampled_discrete_path,
                                             Box2d const&           obs_box,
                                             float64_t const        l_buffer,
                                             float64_t&             lower_s,
                                             float64_t&             upper_s) const
作用：辅助函数，通过双向近似搜索获得精确的上下界
输入：采样的规划离散路径(sampled_discrete_path),  特定时间障碍物的边界框(obs_box),  在不同情况下为小汽车提供额外的横向缓冲(l_buffer), s的上下边界(lower_s, upper_s)
输出：
```

```
bool_t STBoundaryMapper::checkOverlap(CurvePoint const& path_point,
                                      Box2d const&      obs_box,
                                      float64_t const   l_buffer) const
作用：辅助函数，通过给定的路径点和特定时间的单个对象边界框检查是否发生碰撞。
输入：计划路径的路径点，代表自我汽车后桥的中心(path_point), 障碍物边界(obs_box), 在不同情况下为小汽车提供额外的横向缓冲(l_buffer)
输出：
```



### 	1.4 st_boundary_mapper_intersection

```
Status STBoundaryMapperIntersection::ComputeSTBoundary(ObstacleQuerier const& obstacle_querier) const
作用：主函数，
输入：
输出：
```

```
bool_t STBoundaryMapperIntersection::computeSTBoundaryWithoutDecision(Obstacle&                  obstacle,
                                                                      CompetitiveAreaType const& type) const
作用：辅助函数，
输入：
输出：
```

```
bool_t STBoundaryMapperIntersection::computeSTBoundaryWithDecisionIntersection(Obstacle&                  obstacle,
                                                                               ObstacleDecision const&    decision,
                                                                               CompetitiveAreaType const& type) const
作用：辅助函数，
输入：
输出：
```

```
bool_t STBoundaryMapperIntersection::getOverlapBoundaryPointsIntersection(Obstacle const&            obstacle,
                                                                          std::vector<STPoint>&      lower_points,
                                                                          std::vector<STPoint>&      upper_points,
                                                                          CompetitiveAreaType const& type) const
作用：辅助函数，
输入：
输出：
```

```
bool_t STBoundaryMapperIntersection::checkOverlapIntersection(CurvePoint const& path_point,
                                                              Box2d const&      obs_box,
                                                              float64_t const   l_buffer,
                                                              float64_t const   s_buffer) const
作用：辅助函数，
输入：
输出：
```

```
bool_t STBoundaryMapperIntersection::searchBidirectionalIntersection(DiscretizedPath const& sampled_discrete_path,
                                                                     Box2d const&           obs_box,
                                                                     float64_t const        l_buffer,
                                                                     float64_t const        s_buffer,
                                                                     float64_t&             lower_s,
                                                                     float64_t&             upper_s) const
作用：辅助函数，
输入：
输出：
```

```
float64_t STBoundaryMapperIntersection::binarySearchOverlapSIntersection(bool_t                 forward,
                                                                         float64_t              start,
                                                                         float64_t              end,
                                                                         float64_t              resolution,
                                                                         DiscretizedPath const& sampled_discrete_path,
                                                                         Box2d const&           obs_box,
                                                                         float64_t const        l_buffer,
                                                                         float64_t const        s_buffer) const
作用：辅助函数，
输入：
输出：
```

```
bool_t STBoundaryMapperIntersection::addAgentSemanticsIntersection(Obstacle const&            obstacle,
                                                                   CompetitiveAreaType const& type) const
作用：辅助函数，
输入：
输出：
```



## 2 PATH_TIME_HEURISTIC_OPTIMIZER

## 3 SPEED_DECIDER

## 4 SPEED_BOUNDS_FINAL_DECIDER

## 5 NONLINEAR_SPEED_OPTIMIZER