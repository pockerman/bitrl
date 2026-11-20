// Following a line and going around boxes with Monte Carlo tree search
//
// The differential drive robot of bitsim drives around the track of bitsim's
// line_following_infrared_sonar example: a dark line on light paper, with two
// boxes on it. There it follows the line with a hand written controller. Here
// it plans: every DECISION_PERIOD seconds, Monte Carlo tree search tries
// thousands of sequences of commands in a fast model of the track, and the
// robot drives with the first command of the best one.
//
// The model is a kinematic robot, a point that moves with the commanded speed
// and turn rate, on the same track with the same boxes. It rewards progress
// around the track, costs a little for every metre away from the line, and a
// lot for touching a box. The robot itself is the Chrono robot, with its motors
// and wheels, so it does not do exactly what the model says. MCTS plans again
// from where Chrono says the robot is, at every decision.
//
// Usage: mcts_line_following [--render] [--record DIR] [--time-limit SECONDS]

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/mc/mc_tree_search_solver.h"

#include "bitsim/bitsim_config.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/control/infrared_line_follower.h"
#include "bitsim/rigid_bodies/chrono_robots/contact_counter.h"
#include "bitsim/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitsim/sensors/downward_camera.h"
#include "bitsim/trajectory/rounded_rectangle.h"
#include "bitsim/utils/render/irrlicht_scene.h"
#include "bitsim/utils/render/irrlicht_utils.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace mcts_line_following
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rl::algos::mc::MCTSConfig;
using bitrl::rl::algos::mc::MCTSSolver;
using bitsim::rb::bitsim_chrono::CHRONO_DiffDriveRobot;
using bitsim::rb::bitsim_chrono::ContactCounter;
using bitsim::sensors::FloorImage;
using bitsim::trajectory::RoundedRectangle;
using bitsim::utils::render::FrameRecorder;
using bitsim::utils::render::Trail;
using Command = bitsim::control::DiffDriveCommand;
using Point = std::array<real_t, 2>;

const std::string WINDOW_TITLE = "Line following with Monte Carlo tree search";
const real_t PI = bitsim::consts::maths::PI;

// The simulation
const real_t STEP_SIZE = 0.01;      // [s] of Chrono
const real_t DECISION_PERIOD = 0.2; // [s] between two plans
const real_t TIME_LIMIT = 150.0;    // [s]
const real_t PRINT_PERIOD = 2.0;    // [s]
const real_t RENDER_RATE = 30.0;    // [frames/s] of simulation in the window
const real_t RECORD_RATE = 10.0;    // [frames/s] of simulation written by --record

// The track and the boxes, as in bitsim's line_following_infrared_sonar
const real_t TRACK_HALF_LENGTH = 2.2;   // [m] along x, to the centre of the line
const real_t TRACK_HALF_WIDTH = 1.4;    // [m] along y
const real_t TRACK_CORNER_RADIUS = 0.7; // [m]
const real_t LINE_WIDTH = 0.05;         // [m]
const real_t FLOOR_WIDTH = 5.6;         // [m] the paper, along x
const real_t FLOOR_HEIGHT = 4.6;        // [m] along y
const real_t METRES_PER_PIXEL = 0.005;  // of the picture of the paper
const std::vector<Point> OBSTACLES{{0.0, -TRACK_HALF_WIDTH}, {-0.2, TRACK_HALF_WIDTH}};
const real_t OBSTACLE_SIZE = 0.3;   // [m] each side of its base
const real_t OBSTACLE_HEIGHT = 0.4; // [m]
const Point START{-1.3, -TRACK_HALF_WIDTH};
const RoundedRectangle TRACK(TRACK_HALF_LENGTH, TRACK_HALF_WIDTH, TRACK_CORNER_RADIUS);

// The robot in the model: a circle around the centre of the chassis, which is
// CHASSIS_AHEAD in front of the axle. It covers the chassis and the wheels
const real_t CHASSIS_AHEAD = 0.1; // [m]
const real_t ROBOT_RADIUS = 0.3;  // [m]
const real_t CLEARANCE = 0.05;    // [m] the model keeps from a box, for the error of the model
const uint_t MODEL_SUBSTEPS = 4;  // per decision

// What the robot can do: drive straight, turn gently or turn hard, both ways
const std::array<Command, 5> COMMANDS{
    {{0.3, 0.0}, {0.3, 0.8}, {0.3, -0.8}, {0.2, 1.6}, {0.2, -1.6}}};

// The rewards of the model
const real_t LINE_COST = 0.5; // per square metre from the line and second
const real_t HIT_COST = 10.0; // for touching a box, which ends the episode

// The rollouts steer towards the line, and take a random command now and then
const real_t ROLLOUT_GAIN = 2.0;     // [rad/s per rad] of heading error
const real_t ROLLOUT_APPROACH = 2.0; // [1/m] how sharply to head back to the line
const real_t ROLLOUT_RANDOM = 0.2;   // the share of random commands
const real_t DETOUR_NEAR = 1.0;      // [m] closer than this to a box, go around it
const real_t DETOUR_OFFSET = 0.6;    // [m] outside the line, while going around

// The search
const MCTSConfig SEARCH{/*n_simulations*/ 400, /*max_depth*/ 25, /*temperature*/ 0.5,
                        /*gamma*/ 0.98, /*seed*/ 42};

// The track goes around its centre, so the angle of the robot around it
// measures how far along the track it is; one lap is 2 pi. This turns it into
// metres along the line
const real_t METRES_PER_RADIAN = TRACK.perimeter() / (2.0 * PI);

real_t angle_around_track(real_t x, real_t y) { return std::atan2(y, x); }

// The distance from a point to the base of a box
real_t distance_to_box(const Point &box, real_t x, real_t y)
{
    const real_t h = 0.5 * OBSTACLE_SIZE;
    const real_t dx = std::max(std::abs(x - box[0]) - h, 0.0);
    const real_t dy = std::max(std::abs(y - box[1]) - h, 0.0);
    return std::hypot(dx, dy);
}

// ---------------------------------------------------------------------------
// The model MCTS plans in
// ---------------------------------------------------------------------------

struct Pose
{
    real_t x{0.0};
    real_t y{0.0};
    real_t heading{0.0};
};

///
/// A kinematic robot on the track. It is cheap to copy and to step, which is
/// what MCTS needs: it copies the model for every simulation
///
class TrackModel
{
  public:
    typedef uint_t action_type;
    typedef Pose state_type;

    struct TimeStep
    {
        real_t reward_{0.0};
        bool done_{false};
        real_t reward() const noexcept { return reward_; }
        bool done() const noexcept { return done_; }
    };

    explicit TrackModel(const Pose &pose) : pose_(pose) {}

    uint_t n_actions() const noexcept { return COMMANDS.size(); }
    const Pose &pose() const noexcept { return pose_; }

    // Drive with COMMANDS[action] for DECISION_PERIOD
    TimeStep step(action_type action)
    {
        const auto &command = COMMANDS[action];
        const real_t dt = DECISION_PERIOD / MODEL_SUBSTEPS;
        TimeStep time_step;
        for (uint_t i = 0; i < MODEL_SUBSTEPS; ++i)
        {
            const real_t before = angle_around_track(pose_.x, pose_.y);
            pose_.x += command.v * std::cos(pose_.heading) * dt;
            pose_.y += command.v * std::sin(pose_.heading) * dt;
            pose_.heading += command.omega * dt;
            const real_t after = angle_around_track(pose_.x, pose_.y);

            // progress along the track, in metres, and the cost of being away
            // from the line
            const real_t off_line = TRACK.distance(pose_.x, pose_.y);
            time_step.reward_ += METRES_PER_RADIAN * std::remainder(after - before, 2.0 * PI);
            time_step.reward_ -= LINE_COST * off_line * off_line * dt;

            if (touches_a_box())
            {
                time_step.reward_ -= HIT_COST;
                time_step.done_ = true;
                break;
            }
        }
        return time_step;
    }

    bool touches_a_box() const
    {
        const real_t cx = pose_.x + CHASSIS_AHEAD * std::cos(pose_.heading);
        const real_t cy = pose_.y + CHASSIS_AHEAD * std::sin(pose_.heading);
        return std::any_of(OBSTACLES.begin(), OBSTACLES.end(),
                           [&](const Point &box)
                           { return distance_to_box(box, cx, cy) < ROBOT_RADIUS + CLEARANCE; });
    }

  private:
    Pose pose_;
};

// The direction of travel along the track at (x, y): the outward normal, the
// gradient of the signed distance, turned a quarter to the left
real_t track_direction(real_t x, real_t y)
{
    const real_t h = 1e-4;
    const real_t nx = (TRACK.signed_distance(x + h, y) - TRACK.signed_distance(x - h, y)) / (2 * h);
    const real_t ny = (TRACK.signed_distance(x, y + h) - TRACK.signed_distance(x, y - h)) / (2 * h);
    return std::atan2(nx, -ny);
}

///
/// The rollout policy: head along the track and back towards the line, the
/// further from it the more sharply, with the command whose turn rate is
/// closest. Near a box it heads for a path DETOUR_OFFSET outside the line
/// instead, to go around the box. Every ROLLOUT_RANDOM of the time it takes a
/// random command, so that the rollouts also try other ways
///
struct TowardsTheLine
{
    uint_t operator()(const TrackModel &model, std::mt19937 &generator) const
    {
        std::uniform_int_distribution<uint_t> random_command(0, COMMANDS.size() - 1);
        if (std::uniform_real_distribution<real_t>(0.0, 1.0)(generator) < ROLLOUT_RANDOM)
        {
            return random_command(generator);
        }

        const auto &pose = model.pose();
        const real_t cx = pose.x + CHASSIS_AHEAD * std::cos(pose.heading);
        const real_t cy = pose.y + CHASSIS_AHEAD * std::sin(pose.heading);
        const bool near_a_box = std::any_of(OBSTACLES.begin(), OBSTACLES.end(),
                                            [&](const Point &box)
                                            { return distance_to_box(box, cx, cy) < DETOUR_NEAR; });
        const real_t target = near_a_box ? DETOUR_OFFSET : 0.0;

        // signed_distance is positive outside the track: further out than the
        // target, turn left, towards the inside
        const real_t offset = TRACK.signed_distance(pose.x, pose.y) - target;
        const real_t heading =
            track_direction(pose.x, pose.y) + std::atan(ROLLOUT_APPROACH * offset);
        const real_t omega = ROLLOUT_GAIN * std::remainder(heading - pose.heading, 2.0 * PI);

        uint_t best = 0;
        for (uint_t c = 1; c < COMMANDS.size(); ++c)
        {
            if (std::abs(COMMANDS[c].omega - omega) < std::abs(COMMANDS[best].omega - omega))
            {
                best = c;
            }
        }
        return best;
    }
};

// The plan MCTS settled on: from the root, the most visited child each time
template <typename Solver> std::vector<uint_t> best_actions(const Solver &solver)
{
    std::vector<uint_t> actions;
    auto node = solver.root();
    while (node && node->has_children())
    {
        std::shared_ptr<const typename Solver::node_type> best;
        for (uint_t c = 0; c < node->n_children(); ++c)
        {
            auto child = node->get_child(c);
            if (!best || child->total_visits() > best->total_visits())
            {
                best = child;
            }
        }
        if (!best || best->total_visits() == 0)
        {
            break;
        }
        actions.push_back(best->get_action());
        node = best;
    }
    return actions;
}

// ---------------------------------------------------------------------------
// The world the robot drives in
// ---------------------------------------------------------------------------

FloorImage draw_floor()
{
    return FloorImage::draw(FLOOR_WIDTH, FLOOR_HEIGHT, METRES_PER_PIXEL,
                            [](real_t x, real_t y)
                            { return TRACK.distance(x, y) < 0.5 * LINE_WIDTH; });
}

class World
{
  public:
    explicit World(const FloorImage &floor)
    {
        system_.SetGravitationalAcceleration(
            chrono::ChVector3d(0.0, 0.0, -bitsim::consts::maths::G));
        system_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
        system_.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);
        system_.GetSolver()->AsIterative()->SetMaxIterations(40);

        auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        material->SetFriction(0.8f);
        material->SetRestitution(0.1f);
        ground_ = chrono_types::make_shared<chrono::ChBodyEasyBox>(20.0, 20.0, 0.1, 1000.0, true,
                                                                   true, material);
        ground_->SetPos(chrono::ChVector3d(0.0, 0.0, -0.05));
        ground_->SetFixed(true);
        ground_->GetVisualShape(0)->SetColor(chrono::ChColor(0.45f, 0.45f, 0.42f));
        system_.Add(ground_);

        for (const auto &p : OBSTACLES)
        {
            auto box = chrono_types::make_shared<chrono::ChBodyEasyBox>(
                OBSTACLE_SIZE, OBSTACLE_SIZE, OBSTACLE_HEIGHT, 500.0, true, true, material);
            box->SetPos(chrono::ChVector3d(p[0], p[1], 0.5 * OBSTACLE_HEIGHT));
            box->SetFixed(true);
            box->GetVisualShape(0)->SetColor(chrono::ChColor(0.85f, 0.45f, 0.1f));
            system_.Add(box);
            obstacles_.push_back(box);
        }
        contacts_ = std::make_shared<ContactCounter>(obstacles_);

        robot_.load(std::filesystem::path(
            bitsim::consts::build_robot_file_path("bitsim_diff_drive_robot.json")));
        robot_.set_floor([&floor](real_t x, real_t y) { return floor.reflectance(x, y); });
        robot_.set_world_pose(START[0], START[1], 0.0);
        robot_.add_to_sys(system_);
    }

    void step(const Command &command)
    {
        robot_.set_velocity(command.v, command.omega);
        robot_.update(STEP_SIZE);
        system_.DoStepDynamics(STEP_SIZE);
        touching_ = contacts_->count(system_) > 0;
    }

    void show_floor(const std::string &texture)
    {
        bitsim::utils::render::add_floor_texture(*ground_, FLOOR_WIDTH, FLOOR_HEIGHT, 0.051,
                                                 texture);
    }

    real_t time() const { return system_.GetChTime(); }
    Pose pose() const
    {
        const auto axle = robot_.axle_center();
        return {axle.x(), axle.y(), robot_.heading()};
    }
    bool is_touching_a_box() const { return touching_; }
    chrono::ChSystemNSC &system() { return system_; }

  private:
    chrono::ChSystemNSC system_;
    std::shared_ptr<chrono::ChBodyEasyBox> ground_;
    std::vector<std::shared_ptr<chrono::ChBody>> obstacles_;
    std::shared_ptr<ContactCounter> contacts_;
    CHRONO_DiffDriveRobot robot_;
    bool touching_{false};
};

// ---------------------------------------------------------------------------
// The window
// ---------------------------------------------------------------------------

class Display
{
  public:
    Display(World &world, const FloorImage &floor, const std::string &record)
        : world_(world), recorder_(record, RECORD_RATE)
    {
        // The paper on the ground goes on before the visual system starts
        const auto texture =
            (std::filesystem::temp_directory_path() / "bitrl_mcts_line_following_floor.png")
                .string();
        floor.write_texture(texture);
        world_.show_floor(texture);

        vis_.AttachSystem(&world_.system());
        vis_.SetWindowSize(1280, 720);
        vis_.SetWindowTitle(WINDOW_TITLE);
        vis_.SetCameraVertical(chrono::CameraVerticalDir::Z);
        vis_.Initialize();
        vis_.AddCamera({0.0, -3.4, 3.0}, {0.0, -0.1, 0.0});
        // A soft light: the typical lights wash the white paper out
        vis_.AddLightDirectional(60.0, 60.0, chrono::ChColor(0.45f, 0.45f, 0.45f),
                                 chrono::ChColor(0.05f, 0.05f, 0.05f),
                                 chrono::ChColor(0.55f, 0.55f, 0.55f));
    }

    // Returns false once the window is closed
    bool show(const std::vector<Pose> &plan, const Command &command, real_t lap)
    {
        if (!vis_.Run())
        {
            return false;
        }
        const auto pose = world_.pose();
        trail_.add(pose.x, pose.y);

        vis_.BeginScene();
        vis_.Render();
        trail_.draw(vis_, chrono::ChColor(0.1f, 0.4f, 0.9f));
        // The plan: where the model expects the robot to go
        for (std::size_t i = 1; i < plan.size(); ++i)
        {
            bitsim::utils::render::irr_draw_line(
                vis_, chrono::ChVector3d(plan[i - 1].x, plan[i - 1].y, 0.06),
                chrono::ChVector3d(plan[i].x, plan[i].y, 0.06), chrono::ChColor(0.1f, 0.8f, 0.2f));
        }
        std::ostringstream text;
        text << std::fixed << std::setprecision(1) << "t " << world_.time() << " s    lap "
             << 100.0 * lap << " %    v " << std::setprecision(2) << command.v << " m/s    w "
             << command.omega << " rad/s    plan " << plan.size() - 1 << " steps ahead";
        bitsim::utils::render::draw_status_bar(vis_, text.str());
        vis_.EndScene();

        recorder_.record(vis_, world_.time());
        return true;
    }

  private:
    World &world_;
    chrono::irrlicht::ChVisualSystemIrrlicht vis_;
    FrameRecorder recorder_;
    Trail trail_;
};

// ---------------------------------------------------------------------------
// The run
// ---------------------------------------------------------------------------

struct Options
{
    bool render{false};
    std::string record;
    real_t time_limit{TIME_LIMIT};
};

Options parse(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--render")
        {
            options.render = true;
        }
        else if (arg == "--record" && i + 1 < argc)
        {
            options.record = argv[++i];
            options.render = true;
        }
        else if (arg == "--time-limit" && i + 1 < argc)
        {
            options.time_limit = std::stod(argv[++i]);
        }
        else
        {
            std::cout << "Usage: mcts_line_following [--render] [--record DIR] [--time-limit S]\n"
                      << "  --render         show the simulation in a window, in real time\n"
                      << "  --record DIR     also write the window to DIR, " << RECORD_RATE
                      << " frames a second\n"
                      << "  --time-limit S   stop after S seconds of simulation, " << TIME_LIMIT
                      << " by default" << std::endl;
            std::exit(arg == "--help" || arg == "-h" ? 0 : 1);
        }
    }
    return options;
}

int run(const Options &options)
{
    const FloorImage floor = draw_floor();
    World world(floor);
    std::optional<Display> display;
    if (options.render)
    {
        display.emplace(world, floor, options.record);
    }

    MCTSSolver<TrackModel, TowardsTheLine> solver(SEARCH);
    std::cout << std::fixed << std::setprecision(2) << "Track " << TRACK.perimeter()
              << " m around, " << OBSTACLES.size() << " boxes on it. MCTS: " << SEARCH.n_simulations
              << " simulations of up to " << SEARCH.max_depth << " decisions, "
              << SEARCH.max_depth * DECISION_PERIOD << " s, every " << DECISION_PERIOD << " s\n\n"
              << "    t [s]   x [m]   y [m]  head [deg]  line [m]  v [m/s]  w [rad/s]\n";

    Command command;
    std::vector<Pose> plan;
    real_t next_decision = 0.0;
    real_t next_print = 0.0;
    real_t next_frame = 0.0;
    real_t last_angle = angle_around_track(START[0], START[1]);
    real_t travelled = 0.0;
    real_t max_off_line = 0.0;
    real_t planning_time = 0.0;
    uint_t n_decisions = 0;
    uint_t n_hits = 0;
    bool was_touching = false;
    bool completed = false;
    const auto wall_start = std::chrono::steady_clock::now();

    while (world.time() < options.time_limit)
    {
        if (world.time() + 1e-9 >= next_decision)
        {
            next_decision += DECISION_PERIOD;
            const TrackModel model(world.pose());
            const auto start = std::chrono::steady_clock::now();
            const auto action = solver.plan(model);
            planning_time +=
                std::chrono::duration<real_t>(std::chrono::steady_clock::now() - start).count();
            n_decisions += 1;
            command = COMMANDS[action];

            // where the model expects the plan to take the robot
            plan = {model.pose()};
            TrackModel ahead(model);
            for (const auto a : best_actions(solver))
            {
                if (ahead.step(a).done())
                {
                    break;
                }
                plan.push_back(ahead.pose());
            }
        }

        world.step(command);

        const auto pose = world.pose();
        const real_t angle = angle_around_track(pose.x, pose.y);
        travelled += std::remainder(angle - last_angle, 2.0 * PI);
        last_angle = angle;
        max_off_line = std::max(max_off_line, TRACK.distance(pose.x, pose.y));
        if (world.is_touching_a_box() && !was_touching)
        {
            n_hits += 1;
        }
        was_touching = world.is_touching_a_box();

        if (world.time() + 1e-9 >= next_print)
        {
            next_print += PRINT_PERIOD;
            std::cout << std::setw(9) << world.time() << std::setw(8) << pose.x << std::setw(8)
                      << pose.y << std::setw(12) << pose.heading * 180.0 / PI << std::setw(10)
                      << TRACK.distance(pose.x, pose.y) << std::setw(9) << command.v
                      << std::setw(11) << command.omega << "\n";
        }

        if (display && world.time() + 1e-9 >= next_frame)
        {
            next_frame += 1.0 / RENDER_RATE;
            if (!display->show(plan, command, travelled / (2.0 * PI)))
            {
                break;
            }
            // Keep to real time, so that the window shows the robot at its speed
            const auto wall = std::chrono::steady_clock::now() - wall_start;
            const auto ahead = std::chrono::duration<real_t>(world.time()) - wall;
            if (ahead.count() > 0.0)
            {
                std::this_thread::sleep_for(ahead);
            }
        }

        if (travelled >= 2.0 * PI)
        {
            completed = true;
            break;
        }
    }

    const real_t wall =
        std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start).count();
    std::cout << "\n";
    if (completed)
    {
        std::cout << "The robot went once around the track in " << world.time()
                  << " s of simulation, " << wall << " s of wall time\n";
    }
    else
    {
        std::cout << "The robot went " << 100.0 * travelled / (2.0 * PI)
                  << " % around the track in " << world.time() << " s of simulation\n";
    }
    std::cout << "Times it touched a box: " << n_hits
              << ", furthest from the line: " << max_off_line << " m\n"
              << "Decisions: " << n_decisions << ", " << 1000.0 * planning_time / n_decisions
              << " ms of planning each" << std::endl;
    return n_hits == 0 ? 0 : 1;
}

} // namespace mcts_line_following

int main(int argc, char **argv)
{
    try
    {
        return mcts_line_following::run(mcts_line_following::parse(argc, argv));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
