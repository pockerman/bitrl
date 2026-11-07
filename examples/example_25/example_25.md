\page bitrl_example_25 BitRL Example 25: Frames

This example accompanies the first lesson of the robotics course. It shows how to create and use
coordinate frames with _bitrl_. You need _bitrl_ configured with Chrono support to run it.

_bitrl_ does not wrap frames. It uses `chrono::ChFrame<>` directly, and the robots in
`src/bitrl/rigid_bodies/chrono_robots` are built from it, so what you learn here applies to the rest of
the library.

## Why frames

A robot has to relate many things to each other: where it is in the world, where its sensors are mounted
on it, and where an obstacle is relative to a sensor. Each of these is simple in its own frame of
reference. A frame is a position and an orientation with respect to a parent frame. In other words it
tells us where the origin of a coordinate system is, and where its axes point.

We use a robot at \f$(2, 1, 0)\f$ turned \f$90^\circ\f$ around \f$z\f$, and a range sensor on its left
side that looks to the left. An obstacle is at \f$(0, 1, 0.1)\f$ in the world. The frames are

- \f$W\f$ the world frame. Chrono uses the right-hand rule
- \f$R\f$ the robot frame
- \f$S\f$ the sensor frame

## Notation

The code follows the naming used in the rest of _bitrl_:

- `X_AB` is the pose of frame \f$B\f$ expressed in frame \f$A\f$
- `p_A` is a point with coordinates expressed in frame \f$A\f$

The benefit is that the inner letters cancel, so mistakes are easy to spot: `X_WR * X_RS` gives `X_WS`
and `X_WS * p_S` gives `p_W`, whereas `X_RS * X_WR` does not cancel and has no meaning here.

## Creating frames

The default constructor gives the identity frame, and a position alone gives a frame with the same
orientation as its parent. There are several ways to specify an orientation and they are equivalent.

@code{.cpp}
// position and quaternion
ChFrame<> from_quaternion(ROBOT_POSITION, chrono::QuatFromAngleZ(YAW_90));

// position, angle and a unit axis
ChFrame<> from_angle_axis(ROBOT_POSITION, YAW_90, z_axis);

// position and rotation matrix. The arguments are the columns of the matrix:
// where the x, y and z axes of the new frame point in the parent
const chrono::ChMatrix33d R(ChVector3d(0.0, 1.0, 0.0), ChVector3d(-1.0, 0.0, 0.0), z_axis);
ChFrame<> from_matrix(ROBOT_POSITION, R);

// a chrono::ChCoordsysd which is a position and a quaternion
ChFrame<> from_coordsys(chrono::ChCoordsysd(ROBOT_POSITION, chrono::QuatFromAngleZ(YAW_90)));

// start from the identity and set the pose
ChFrame<> from_setters;
from_setters.SetPos(ROBOT_POSITION);
from_setters.SetRot(chrono::QuatFromAngleZ(YAW_90));
@endcode

The rotation matrix version is worth a closer look. Its columns are the axes of the new frame seen from
the parent. The robot is turned \f$90^\circ\f$ so its \f$x\f$ axis points along the world \f$y\f$ axis.
The output below prints every frame that way.

@code
1. Creating frames
  world          origin=(0.000, 0.000, 0.000)  x=(1.000, 0.000, 0.000)  y=(0.000, 1.000, 0.000)  z=(0.000, 0.000, 1.000)
  translated     origin=(2.000, 1.000, 0.000)  x=(1.000, 0.000, 0.000)  y=(0.000, 1.000, 0.000)  z=(0.000, 0.000, 1.000)
  robot          origin=(2.000, 1.000, 0.000)  x=(0.000, 1.000, 0.000)  y=(-1.000, 0.000, 0.000)  z=(0.000, 0.000, 1.000)
  all five are the same pose: true
@endcode

## Points and directions

A frame maps coordinates from itself to its parent. For a rotation \f$R_{WR}\f$ and a translation
\f$t_{WR}\f$

\f[
p_W = R_{WR} \, p_R + t_{WR}
\f]

A direction has no location, so only the rotation applies. This distinction matters: a sensor's mounting
point is a point but the force on a wheel or the direction a sensor looks along are directions.

@code{.cpp}
const ChVector3d point_in_world = X_WR * in_robot;     // TransformPointLocalToParent
const ChVector3d point_in_robot = X_WR / point_in_world; // TransformPointParentToLocal
const ChVector3d direction = X_WR.TransformDirectionLocalToParent(in_robot);
@endcode

@code
2. Points and directions
  point in R     (1.000, 0.000, 0.000)
  point in W     (2.000, 2.000, 0.000)
  back in R      (1.000, 0.000, 0.000)
  direction in W (0.000, 1.000, 0.000)
@endcode

## Composing frames

We know how the sensor is mounted on the robot and where the robot is in the world. The pose of the
sensor in the world follows by composing the two

@code{.cpp}
const ChFrame<> X_WS = X_WR * X_RS;
@endcode

Composition does not commute. `X_RS * X_WR` below is a different frame, and it does not mean anything
physically. Chrono also has `operator>>` which composes in the reverse order. We stick to `operator*`
since it reads like the maths and like the rest of _bitrl_.

@code
3. Composing frames
  X_RS           origin=(0.000, 0.200, 0.100)  x=(0.000, 1.000, 0.000)  y=(-1.000, 0.000, 0.000)  z=(0.000, 0.000, 1.000)
  X_WS           origin=(1.800, 1.000, 0.100)  x=(-1.000, 0.000, 0.000)  y=(0.000, -1.000, 0.000)  z=(0.000, 0.000, 1.000)
  X_RS * X_WR    origin=(-1.000, 2.200, 0.100)  x=(-1.000, 0.000, 0.000)  y=(0.000, -1.000, 0.000)  z=(0.000, 0.000, 1.000)
@endcode

## Inverting frames

The inverse gives the opposite relationship: `X_SW` is the pose of the world in the sensor. It answers
the question a robot actually asks, which is what something looks like from where the sensor is.

@code{.cpp}
const ChFrame<> X_SW = X_WS.GetInverse();
const ChVector3d obstacle_in_sensor = X_SW * OBSTACLE_IN_WORLD;
@endcode

The obstacle comes out on the sensor's \f$x\f$ axis, so it is straight ahead, at a range of 1.8 m. Note
that the origin of `X_SW` is not simply minus the origin of `X_WS`, since the inverse also has to undo
the rotation

\f[
R_{SW} = R_{WS}^T \quad t_{SW} = -R_{WS}^T \, t_{WS}
\f]

Composing a frame with its inverse gives the identity.

@code
4. Inverting frames
  X_SW           origin=(1.800, 1.000, -0.100)  x=(-1.000, 0.000, 0.000)  y=(0.000, -1.000, 0.000)  z=(0.000, 0.000, 1.000)
  X_WS * X_SW    origin=(0.000, 0.000, 0.000)  x=(1.000, 0.000, 0.000)  y=(0.000, 1.000, 0.000)  z=(0.000, 0.000, 1.000)
  obstacle in W  (0.000, 1.000, 0.100)
  obstacle in S  (1.800, 0.000, 0.000)
  range = 1.8 m
@endcode

## Frames attached to a moving body

A `chrono::ChBody` is a frame that the simulation moves. `GetFrameRefToAbs()` returns its pose in the
world. `X_RS` stays constant, because the sensor is rigidly mounted, so we only recompute `X_WS`.

_bitrl_ provides `CHRONO_RobotPose` to monitor a body and convert between its local coordinates and the
world.

@code{.cpp}
bitrl::rb::bitrl_chrono::CHRONO_RobotPose pose(robot);
const ChVector3d front_in_world = pose.to_world_coords(front_of_robot);

const ChFrame<> &X_WR = robot->GetFrameRefToAbs();
const ChFrame<> X_WS = X_WR * X_RS;
const ChVector3d obstacle_in_sensor = X_WS / OBSTACLE_IN_WORLD;
@endcode

We spin the robot by \f$90^\circ\f$ over one second. The robot's origin stays where it is, the sensor
swings around it, and the obstacle, which has not moved, is now somewhere else in the sensor frame.

@code
5. Frames attached to a moving body
  t = 0.0 s
  X_WR           origin=(2.000, 1.000, 0.000)  x=(0.000, 1.000, 0.000)  y=(-1.000, 0.000, 0.000)  z=(0.000, 0.000, 1.000)
  X_WS           origin=(1.800, 1.000, 0.100)  x=(-1.000, 0.000, 0.000)  y=(0.000, -1.000, 0.000)  z=(0.000, 0.000, 1.000)
  front in W     (2.000, 1.300, 0.000)
  obstacle in S  (1.800, 0.000, 0.000)
  t = 1.0 s
  X_WR           origin=(2.000, 1.000, 0.000)  x=(-1.000, 0.000, 0.000)  y=(0.000, -1.000, 0.000)  z=(0.000, 0.000, 1.000)
  X_WS           origin=(2.000, 0.800, 0.100)  x=(0.000, -1.000, 0.000)  y=(1.000, 0.000, 0.000)  z=(0.000, 0.000, 1.000)
  front in W     (1.700, 1.000, 0.000)
  obstacle in S  (-0.200, -2.000, 0.000)
@endcode

## Running the example

@code
./example_25
./example_25 --render
@endcode

With `--render` the robot keeps spinning in an Irrlicht window, slower than in section 5 so that you can
follow the numbers. The frames are drawn with their names, and \f$x\f$ is red, \f$y\f$ is green and
\f$z\f$ is blue. The yellow line goes from the sensor to the obstacle.

Watch the panel at the top right. `obstacle in W` never changes since the obstacle does not move, whereas
`obstacle in S` changes all the time. It is the same point seen from a frame that moves. The range
changes too, because the sensor is not at the centre of the rotation.

## Exercises

1. Move the sensor to the front of the robot, looking forward. What is `X_RS`? Where is it in the world?
2. Mount the sensor tilted \f$30^\circ\f$ downwards. Hint: compose rotations with quaternion `operator*`.
3. Compute `obstacle in S` at \f$t = 1\f$ by hand and compare it with the output.
4. In section 5, transform `front_of_robot` as a direction instead of a point. Which values change, and why?
