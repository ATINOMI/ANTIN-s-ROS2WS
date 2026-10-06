#include <gtest/gtest.h>
#include <fstream>
#include <limits>
#include "mini_nav_core/navigator/astar_navigator.hpp"
#include "mini_nav_core/navigator/path_tracker.hpp"
#include "mini_nav_core/map/rolling_obstacle_grid.hpp"

using namespace mini_nav_core;

TEST(CollisionGeometry, CellAreaAndControllerAgreeAtUnsafeStart) {
    Costmap2D raw(80, 80, .05, -2, -2, 0);
    raw.SetCost(45, 51, 254);
    const std::vector<PathPoint> points;
    CollisionGeometry geometry{raw, points, .26};
    const PathPoint robot{.04, .40};
    CollisionConflict conflict;
    ASSERT_FALSE(geometry.IsClear(robot, robot, 0.0, &conflict));
    EXPECT_STREQ(conflict.kind, "occupied_cell");
    EXPECT_LE(conflict.distance, conflict.required);
    PathTracker tracker;
    tracker.SetPath({robot, {1, .4}}, 0);
    EXPECT_EQ(tracker.Step({.04, .4, 0}, {.04, .4, 0}, {}, geometry, geometry, 0).status,
              TrackingStatus::kCollisionRisk);
    MapLocation start{}, goal{};
    raw.WorldToMap(robot.x, robot.y, start.x, start.y);
    raw.WorldToMap(1, .4, goal.x, goal.y);
    EXPECT_TRUE(AStarPlanner().Plan(raw, geometry, robot, start, goal).empty());
}

TEST(CollisionGeometry, ContinuousPointsRetainSafetyAndIgnoreIndexQuantization) {
    Costmap2D coverage(80, 80, .05, -2, -2, 0);
    const std::vector<PathPoint> points{{.29, 0}};
    CollisionGeometry geometry{coverage, points, .26, .03};
    CollisionConflict conflict;
    EXPECT_FALSE(geometry.IsClear({0, 0}, {0, 0}, 0.0, &conflict));
    EXPECT_STREQ(conflict.kind, "dynamic_point");
    EXPECT_NEAR(conflict.distance, .29, 1e-9);
    EXPECT_NEAR(conflict.required, .29, 1e-9);
    EXPECT_TRUE(geometry.IsClear({-.001, 0}, {-.001, 0}));
    EXPECT_FALSE(geometry.IsClear({-.1, 0}, {.1, 0}));
    CollisionGeometry uncertain{coverage, points, .26, .04};
    EXPECT_FALSE(uncertain.IsClear({-.001, 0}, {-.001, 0}));
}

TEST(CollisionGeometry, ActualStartConnectionMustBeChecked) {
    Costmap2D map(40, 40, .1, -2, -2, 0);
    std::vector<PathPoint> points{{.2, .05}};
    CollisionGeometry geometry{map, points, .1};
    EXPECT_TRUE(geometry.IsClear({.05, .05}, {.05, .05}));
    EXPECT_TRUE(AStarPlanner().Plan(map, geometry, {.14, .05}, {20,20}, {10,20}).empty());
}

TEST(CollisionGeometry, DynamicEndpointsFollowClearingRollingAndExpiry) {
    RollingObstacleGrid grid(40, 40, .05, {});
    grid.CenterOn(0, 0);
    ASSERT_TRUE(grid.MarkObstacle(.29, 0, 1));
    EXPECT_EQ(grid.ObstaclePoints().size(), 1u);
    unsigned int x, y;
    auto coverage = grid.CollisionGrid();
    coverage.WorldToMap(.29, 0, x, y);
    EXPECT_EQ(coverage.GetCost(x, y), 0);
    grid.CenterOn(.1, 0);
    EXPECT_EQ(grid.ObstaclePoints().size(), 1u);
    EXPECT_DOUBLE_EQ(grid.ObstaclePoints().front().x, .29);
    grid.IntegrateRay(0, 0, .5, 0, 2);
    EXPECT_TRUE(grid.ObstaclePoints().empty());
    grid.MarkObstacle(.29, 0, 2);
    grid.Expire(5, 2);
    EXPECT_TRUE(grid.ObstaclePoints().empty());
    EXPECT_EQ(grid.CollisionGrid().GetCost(x-2, y), 255);
}

TEST(CollisionGeometry, FrozenGazeboSceneUsesOneContinuousSafetyVerdict) {
    std::ifstream metadata(std::string(COLLISION_FIXTURE_DIR) + "/collision_risk_points.txt");
    unsigned int w, h; double resolution, ox, oy; localization::Pose2D pose{}; std::size_t count;
    ASSERT_TRUE(static_cast<bool>(metadata >> w >> h >> resolution >> ox >> oy >> pose.x >> pose.y >> pose.yaw >> count));
    Costmap2D raw(w, h, resolution, ox, oy, 255);
    std::ifstream cells(std::string(COLLISION_FIXTURE_DIR) + "/collision_risk_static.bin", std::ios::binary);
    for (std::size_t i = 0; i < raw.GetCellCount(); ++i) {
        char value; ASSERT_TRUE(static_cast<bool>(cells.get(value)));
        MapLocation cell{}; raw.IndexToMap(i, cell); raw.SetCost(cell.x, cell.y, static_cast<unsigned char>(value));
    }
    std::vector<PathPoint> points(count);
    for (auto & point : points) ASSERT_TRUE(static_cast<bool>(metadata >> point.x >> point.y));
    CollisionGeometry geometry{raw, points, .26, .03};
    const PathPoint robot{pose.x, pose.y};
    ASSERT_TRUE(geometry.IsClear(robot, robot));
    auto display_raw = raw;
    for (const auto & point : points) {
        unsigned int x,y;
        if (display_raw.WorldToMap(point.x, point.y, x,y) && display_raw.GetCost(x,y) != 255)
            display_raw.SetCost(x,y,254);
    }
    EXPECT_FALSE(IsCircularSweepClear(display_raw, robot, robot, .26));
    InflationParameters inflation;
    inflation.inscribed_radius = .22549849949589046;
    inflation.inflation_radius = .7; inflation.cost_scaling_factor = 3;
    const auto planning = InflateCostmap(display_raw, inflation);
    MapLocation start{}; raw.WorldToMap(pose.x, pose.y, start.x, start.y);
    const auto path = AStarPlanner().Plan(planning, geometry, robot, start, {43,14}, .5);
    ASSERT_FALSE(path.empty());
    const auto smooth = SimplifyAndSmoothPath(raw, planning, path, .26, 2, true, &geometry);
    ASSERT_FALSE(smooth.empty());
    for (std::size_t i=1; i<smooth.size(); ++i) EXPECT_TRUE(geometry.IsClear(smooth[i-1],smooth[i]));
    PathTrackerParameters parameters;
    parameters.max_linear_speed=.15; parameters.max_angular_speed=.55;
    parameters.max_linear_acceleration=.3; parameters.command_reaction_time=1;
    PathTracker tracker(parameters);
    tracker.SetPath({robot, {pose.x+std::cos(pose.yaw-.2),pose.y+std::sin(pose.yaw-.2)}},pose.yaw);
    bool blocked=false;
    for (int i=0;i<80;++i) {
        const auto cmd=tracker.Step(pose,pose,{},geometry,geometry,i*.1);
        if (blocked) { EXPECT_NE(cmd.status,TrackingStatus::kTracking); }
        if (cmd.status==TrackingStatus::kCollisionRisk) blocked=true;
        if (cmd.linear_x != 0 || cmd.angular_z != 0) {
            EXPECT_TRUE(cmd.status==TrackingStatus::kTracking || cmd.status==TrackingStatus::kAvoidingObstacle);
        }
    }
    // 当 map 配准的额外预算为 3 cm 时，旧现场不能继续宣称可通行。
    CollisionGeometry map_budget{raw, points, .29, .03};
    EXPECT_FALSE(map_budget.IsClear(robot,robot));
    EXPECT_TRUE(AStarPlanner().Plan(planning,map_budget,robot,start,{43,14},.5).empty());
    PathTracker conservative;
    conservative.SetPath({robot,{pose.x+1,pose.y}},pose.yaw);
    EXPECT_EQ(conservative.Step(pose,pose,{},map_budget,map_budget,0).status,TrackingStatus::kCollisionRisk);
    CollisionGeometry excessive_error{raw, points, .26, .04};
    EXPECT_FALSE(excessive_error.IsClear(robot,robot));
    EXPECT_TRUE(AStarPlanner().Plan(planning,excessive_error,robot,start,{43,14},.5).empty());
}

TEST(CollisionGeometry, RejectsUnknownBoundaryInvalidAndSweptPoint) {
    Costmap2D map(80,80,.05,-2,-2,0);
    std::vector<PathPoint> points{{.5,.1}};
    CollisionGeometry geometry{map,points,.26,.03};
    EXPECT_FALSE(geometry.IsClear({0,0},{1,0}));
    EXPECT_FALSE(geometry.IsClear({1.9,0},{1.9,0}));
    map.SetCost(40,40,255);
    EXPECT_FALSE(geometry.IsClear({0,0},{0,0}));
    points[0].x=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(geometry.IsClear({-1,-1},{-1,-1}));
}

TEST(CollisionGeometry, SafeCandidateAndRecoveryHaveFiniteBounds) {
    Costmap2D map(100,100,.05,-2.5,-2.5,0);
    std::vector<PathPoint> points{{.32,.06}};
    CollisionGeometry geometry{map,points,.26,.03};
    PathTracker tracker;
    tracker.SetPath({{0,0},{1,.2}},0);
    TrackingDiagnostics diagnostics;
    auto cmd=tracker.Step({}, {}, {}, geometry, geometry,0, &diagnostics);
    ASSERT_GT(diagnostics.candidates.size(),1u);
    EXPECT_FALSE(diagnostics.candidates.front().safe);
    EXPECT_STREQ(diagnostics.candidates.front().conflict.kind,"dynamic_point");
    EXPECT_EQ(cmd.status,TrackingStatus::kAvoidingObstacle);
    EXPECT_GT(cmd.linear_x,0);
    EXPECT_LT(cmd.linear_x,.1);
    PathPoint previous{};
    for (int i=1;i<=20;++i) {
        const double t=i*.05;
        const PathPoint next=std::abs(cmd.angular_z)<1e-9 ? PathPoint{cmd.linear_x*t,0} :
            PathPoint{cmd.linear_x/cmd.angular_z*std::sin(cmd.angular_z*t),
                      cmd.linear_x/cmd.angular_z*(1-std::cos(cmd.angular_z*t))};
        EXPECT_TRUE(geometry.IsClear(previous,next,std::abs(cmd.linear_x*cmd.angular_z)*.05*.05/8));
        previous=next;
    }
    for (int i=1;i<=45;++i) cmd=tracker.Step({}, {}, {}, geometry, geometry,i*.1);
    EXPECT_EQ(cmd.status,TrackingStatus::kCollisionRisk);
    EXPECT_DOUBLE_EQ(cmd.linear_x,0);
    EXPECT_DOUBLE_EQ(cmd.angular_z,0);
    points.clear();
    EXPECT_EQ(tracker.Step({}, {}, {}, geometry, geometry,4.6).status,TrackingStatus::kTracking);
}

TEST(CollisionGeometry, StaticRouteTerminalUsesAllCurrentObservations) {
    Costmap2D map(40, 40, .1, -2, -2, 0);
    const std::vector<PathPoint> empty, points{{.45, .05}};
    CollisionGeometry route{map, empty, .29};
    CollisionGeometry current{map, points, .29, .03};
    const PathPoint robot{-.55, .05};
    auto path = AStarPlanner().Plan(map, route, robot, {14, 20}, {24, 20}, .5, &current);
    ASSERT_FALSE(path.empty());
    PathPoint end{};
    map.MapToWorld(path.back().x, path.back().y, end.x, end.y);
    EXPECT_TRUE(current.IsClear(end, end));
    EXPECT_TRUE(AStarPlanner().Plan(map, route, robot, {14, 20}, {24, 20}, 0., &current).empty());
    CollisionGeometry no_terminal{map, points, .29, 10.};
    EXPECT_TRUE(AStarPlanner().Plan(map, route, robot, {14, 20}, {24, 20}, .5, &no_terminal).empty());
}

TEST(CollisionGeometry, ContinuousSafeStartCanConnectToAdjacentSafeCenters) {
    Costmap2D map(80, 80, .05, -2., -2., 0);
    map.SetCost(40, 40, 254);
    const std::vector<PathPoint> empty;
    const CollisionGeometry geometry{map, empty, .29};
    const PathPoint actual{.025, .342};
    ASSERT_TRUE(geometry.IsClear(actual, actual));
    ASSERT_FALSE(geometry.IsClear(actual, {.025, .325}));
    const auto path = AStarPlanner().Plan(map, geometry, actual, {40, 46}, {40, 60});
    ASSERT_FALSE(path.empty());
    PathPoint first{};
    map.MapToWorld(path.front().x, path.front().y, first.x, first.y);
    EXPECT_TRUE(geometry.IsClear(actual, first));
    EXPECT_EQ(path.back().x, 40u);
    EXPECT_EQ(path.back().y, 60u);
    for (std::size_t i = 1; i < path.size(); ++i) {
        PathPoint a{}, b{};
        map.MapToWorld(path[i-1].x, path[i-1].y, a.x, a.y);
        map.MapToWorld(path[i].x, path[i].y, b.x, b.y);
        EXPECT_TRUE(geometry.IsClear(a, b));
    }
    EXPECT_TRUE(AStarPlanner().Plan(map, geometry, {.025, .335}, {40,46}, {40,60}).empty());
}

TEST(CollisionGeometry, FrozenSafeStartConnectsWithoutShrinkingBodyOrObservationBudget) {
    std::ifstream meta(std::string(COLLISION_FIXTURE_DIR) + "/safe_start_anchor_points.txt");
    unsigned int w, h; double res, ox, oy, radius, uncertainty; PathPoint actual{}; std::size_t count;
    ASSERT_TRUE(static_cast<bool>(meta >> w >> h >> res >> ox >> oy >> radius >> uncertainty >>
        actual.x >> actual.y >> count));
    Costmap2D map(w, h, res, ox, oy, 255);
    std::ifstream cells(std::string(COLLISION_FIXTURE_DIR) + "/safe_start_anchor_static.bin", std::ios::binary);
    for (std::size_t i = 0; i < map.GetCellCount(); ++i) {
        char value; ASSERT_TRUE(static_cast<bool>(cells.get(value)));
        MapLocation cell{}; map.IndexToMap(i, cell); map.SetCost(cell.x, cell.y, static_cast<unsigned char>(value));
    }
    std::vector<PathPoint> points(count), empty;
    for (auto & point : points) ASSERT_TRUE(static_cast<bool>(meta >> point.x >> point.y));
    CollisionGeometry current{map, points, radius, uncertainty}, route{map, empty, radius};
    CollisionGeometry terminal{map, points, radius + .12, uncertainty};
    ASSERT_TRUE(current.IsClear(actual, actual));
    MapLocation start{}, goal{}; map.WorldToMap(actual.x, actual.y, start.x, start.y);
    map.WorldToMap(-.00849533, 1.01803, goal.x, goal.y);
    PathPoint center{}; map.MapToWorld(start.x, start.y, center.x, center.y);
    ASSERT_FALSE(route.IsClear(actual, center));
    const auto planning = InflateCostmap(map, InflationParameters{});
    for (const auto * geometry : {&route, &current}) {
        const auto path = AStarPlanner().Plan(planning, *geometry, actual, start, goal, .5, &terminal, &current);
        ASSERT_FALSE(path.empty());
        PathPoint first{}, end{};
        map.MapToWorld(path.front().x, path.front().y, first.x, first.y);
        map.MapToWorld(path.back().x, path.back().y, end.x, end.y);
        EXPECT_TRUE(current.IsClear(actual, first));
        EXPECT_TRUE(terminal.IsClear(end, end));
        const auto smooth = SimplifyAndSmoothPath(map, planning, path, radius, 2., true, geometry);
        ASSERT_FALSE(smooth.empty());
        EXPECT_TRUE(current.IsClear(actual, smooth.front()));
        for (std::size_t i = 1; i < smooth.size(); ++i)
            EXPECT_TRUE(geometry->IsClear(smooth[i-1], smooth[i]));
    }
}

TEST(CollisionGeometry, AdjacentAnchorsRespectBoundaryUnknownAndCurrentPoints) {
    Costmap2D map(80, 80, .05, -2., -2., 0);
    std::vector<PathPoint> empty, points{{.35, .375}};
    map.SetCost(40, 40, 255);
    CollisionGeometry route{map, empty, .29}, current{map, points, .29, .03};
    const PathPoint actual{.025, .342};
    ASSERT_TRUE(current.IsClear(actual, actual));
    auto path = AStarPlanner().Plan(map, route, actual, {40,46}, {60,55}, 0., nullptr, &current);
    ASSERT_FALSE(path.empty());
    PathPoint first{}; map.MapToWorld(path.front().x, path.front().y, first.x, first.y);
    EXPECT_TRUE(current.IsClear(actual, first));
    EXPECT_TRUE(AStarPlanner().Plan(map, route, {.025,.335}, {40,46}, {60,55}).empty());
    const PathPoint boundary{-1.704, 1.};
    ASSERT_TRUE(route.IsClear(boundary, boundary));
    path = AStarPlanner().Plan(map, route, boundary, {5,60}, {20,60});
    ASSERT_FALSE(path.empty());
    map.MapToWorld(path.front().x, path.front().y, first.x, first.y);
    EXPECT_TRUE(route.IsClear(boundary, first));
    EXPECT_TRUE(AStarPlanner().Plan(map, route, {-1.9,1.}, {2,60}, {20,60}).empty());
}

TEST(CollisionGeometry, FrozenRelativeObservationRetainsStaticAndSensorBudgets) {
    std::ifstream meta(std::string(COLLISION_FIXTURE_DIR) + "/relative_scan_geometry.txt");
    unsigned w, h; double res, ox, oy; localization::Pose2D pose{}, from_map{}; std::size_t count;
    ASSERT_TRUE(static_cast<bool>(meta >> w >> h >> res >> ox >> oy >> pose.x >> pose.y >> pose.yaw >>
        from_map.x >> from_map.y >> from_map.yaw >> count));
    Costmap2D map(w, h, res, ox, oy, 255);
    std::ifstream binary(std::string(COLLISION_FIXTURE_DIR) + "/relative_scan_static.bin", std::ios::binary);
    for (std::size_t i = 0; i < map.GetCellCount(); ++i) {
        char value; ASSERT_TRUE(static_cast<bool>(binary.get(value)));
        MapLocation cell{}; map.IndexToMap(i, cell); map.SetCost(cell.x, cell.y, static_cast<unsigned char>(value));
    }
    std::vector<PathPoint> observed(count), empty;
    for (auto & point : observed) ASSERT_TRUE(static_cast<bool>(meta >> point.x >> point.y));
    CollisionGeometry geometry{map, observed, .29, .03};
    geometry.point_radius = .26;
    geometry.points_from_grid_translation = {from_map.x, from_map.y};
    geometry.points_from_grid_yaw = from_map.yaw;
    const PathPoint robot{pose.x, pose.y};
    ASSERT_TRUE(geometry.IsClear(robot, robot));
    CollisionGeometry legacy = geometry; legacy.point_radius = 0.;
    ASSERT_FALSE(legacy.IsClear(robot, robot));
    auto terminal = geometry; terminal.radius += .12; terminal.point_radius += .12;
    MapLocation start{}, goal{}; map.WorldToMap(pose.x, pose.y, start.x, start.y);
    map.WorldToMap(.273739, -.447776, goal.x, goal.y);
    auto path = AStarPlanner().Plan(map, geometry, robot, start, goal, .5, &terminal, &geometry);
    ASSERT_FALSE(path.empty());
    PathPoint first{}; map.MapToWorld(path.front().x, path.front().y, first.x, first.y);
    EXPECT_TRUE(geometry.IsClear(robot, first));
    for (std::size_t i = 1; i < path.size(); ++i) {
        PathPoint a{}, b{};map.MapToWorld(path[i-1].x,path[i-1].y,a.x,a.y);
        map.MapToWorld(path[i].x,path[i].y,b.x,b.y); EXPECT_TRUE(geometry.IsClear(a,b));
    }
    const double c = std::cos(from_map.yaw), s = std::sin(from_map.yaw);
    const localization::Pose2D odom_pose{from_map.x+c*pose.x-s*pose.y,
        from_map.y+s*pose.x+c*pose.y,pose.yaw+from_map.yaw};
    Costmap2D coverage(200,200,.05,-5.,-5.,0);
    CollisionGeometry local{coverage,observed,.26,.03};
    PathTracker tracker;tracker.SetPath({robot,{pose.x-.3,pose.y+.3}},pose.yaw);
    const auto command=tracker.Step(pose,odom_pose,from_map,geometry,local,0.);
    EXPECT_NE(command.status,TrackingStatus::kCollisionRisk);
    EXPECT_NE(command.status,TrackingStatus::kProgressTimeout);
    MapLocation body{};map.WorldToMap(pose.x,pose.y,body.x,body.y);map.SetCost(body.x,body.y,254);
    EXPECT_FALSE(geometry.IsClear(robot,robot));
    EXPECT_TRUE(AStarPlanner().Plan(map,geometry,robot,start,goal,.5).empty());
}

TEST(CollisionGeometry, RelativePointSweepIsInvariantToMapCorrectionAndRejectsRealOverlap) {
    Costmap2D map(200,200,.1,-10.,-10.,0);
    const std::vector<PathPoint> observed{{.311,0.}};
    for (const double yaw : {0., .7, -1.2}) {
        const PathPoint translation{1.3,-.8};
        const double c=std::cos(yaw),s=std::sin(yaw);
        CollisionGeometry geometry{map,observed,.29,.03};geometry.point_radius=.26;
        geometry.points_from_grid_translation={-c*translation.x-s*translation.y,
            s*translation.x-c*translation.y};geometry.points_from_grid_yaw=-yaw;
        EXPECT_TRUE(geometry.IsClear(translation,translation));
        const PathPoint toward{translation.x+.022*c,translation.y+.022*s};
        EXPECT_FALSE(geometry.IsClear(toward,toward));
        EXPECT_FALSE(geometry.IsClear(translation,toward));
        EXPECT_FALSE(geometry.IsClear(translation,translation,.022));
        geometry.points_from_grid_yaw=std::numeric_limits<double>::infinity();
        EXPECT_FALSE(geometry.IsClear(translation,translation));
    }
}

TEST(CollisionGeometry, SafePolylineRequiresAVisibleLookaheadPrefix) {
    Costmap2D map(112,103,.05000000074505806,-.961,-2.072,0);
    map.SetCost(60,48,254);
    const std::vector<PathPoint> empty;
    CollisionGeometry geometry{map,empty,.29};
    localization::Pose2D pose{2.3062501673642237,.1328280512182143,3.5755223684517494};
    const std::vector<PathPoint> path{{2.3062478337932832, 0.13280035063198739},
        {2.3140000488013031, 0.10300003241002553},
        {2.1523604241131093, 0.016839752805824837},
        {1.9813675236884452, 0.0060720811795003105},
        {1.5140000368803741, 0.10300003241002553}};
    PathTrackerParameters parameters;
    parameters.max_linear_speed=.15;parameters.max_angular_speed=.55;
    parameters.max_linear_acceleration=.3;parameters.max_linear_deceleration=.5;
    parameters.max_angular_acceleration=1.;parameters.command_reaction_time=1.;
    parameters.goal_position_tolerance=.12;parameters.goal_position_hysteresis=.03;
    parameters.rotate_in_place_angle=.35;parameters.lookahead_distance=.35;
    PathTracker tracker(parameters);tracker.SetPath(path,-3.119995057698527);
    bool arrived=false, translated=false;
    for(int i=0;i<800;++i) {
        const auto command=tracker.Step(pose,pose,{},geometry,geometry,i*.1);
        ASSERT_NE(command.status,TrackingStatus::kCollisionRisk);
        ASSERT_NE(command.status,TrackingStatus::kProgressTimeout);
        if(command.status==TrackingStatus::kGoalReached){arrived=true;break;}
        EXPECT_LE(command.linear_x,.15);EXPECT_LE(std::abs(command.angular_z),.55);
        // 独立按 0.01 秒积分真实圆弧，避免整周期弦的保守半径误报。
        for (int sample = 0; sample < 10; ++sample) {
            const double dt = .01;
            const double half_turn = command.angular_z * dt / 2.;
            const double advance = command.linear_x * dt *
                (half_turn == 0. ? 1. : std::sin(half_turn) / half_turn);
            const PathPoint next{pose.x + advance * std::cos(pose.yaw + half_turn),
                pose.y + advance * std::sin(pose.yaw + half_turn)};
            ASSERT_TRUE(geometry.IsClear({pose.x, pose.y}, next,
                std::abs(command.linear_x * command.angular_z) * dt * dt / 8.))
                << "step=" << i << " sample=" << sample;
            translated = translated || advance > 0.;
            pose.x = next.x; pose.y = next.y; pose.yaw += command.angular_z * dt;
        }
    }
    EXPECT_TRUE(translated);EXPECT_TRUE(arrived);
}
