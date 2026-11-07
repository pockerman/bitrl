\page bitrl_example_26 BitRL Example 26: Rotation matrices

This example accompanies the second lesson of the robotics course and builds on
\ref bitrl_example_25 "Example 25". There we used frames and let Chrono handle the orientation. Now we
open up that part. You need _bitrl_ configured with Chrono support to run it.

## What a rotation matrix is

A frame has a position and an orientation. A rotation matrix is one way to write the orientation. It is
a \f$3 \times 3\f$ matrix, and it has a very direct meaning: **its columns are the axes of the frame,
seen from the parent**.

We keep the robot and the sensor of the previous lesson, and the same notation. `R_AB` is the
orientation of frame \f$B\f$ expressed in frame \f$A\f$, so the inner letters still cancel:
`R_WR * R_RS` gives `R_WS` and `R_WR * v_R` gives `v_W`.

## Elementary rotations

Any rotation can be built from rotations around the three coordinate axes:

\f[
R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & c & -s \\ 0 & s & c \end{bmatrix}
\quad
R_y(\theta) = \begin{bmatrix} c & 0 & s \\ 0 & 1 & 0 \\ -s & 0 & c \end{bmatrix}
\quad
R_z(\theta) = \begin{bmatrix} c & -s & 0 \\ s & c & 0 \\ 0 & 0 & 1 \end{bmatrix}
\f]

with \f$c = \cos\theta\f$ and \f$s = \sin\theta\f$. A positive angle turns counterclockwise when the axis
points at you. Notice that each matrix leaves its own axis unchanged, and that the signs in \f$R_y\f$
are the other way around compared to the other two. The example writes these out by hand and checks
them against Chrono, which builds the same matrices from an angle and an axis:

@code{.cpp}
const ChMatrix33d R(30.0 * DEG, ChVector3d(0.0, 0.0, 1.0));
@endcode

@code
1. Elementary rotations
  Rx(30)
      1.000   0.000   0.000 
      0.000   0.866  -0.500 
      0.000   0.500   0.866 
  Ry(30)
      0.866   0.000   0.500 
      0.000   1.000   0.000 
     -0.500   0.000   0.866 
  Rz(30)
      0.866  -0.500   0.000 
      0.500   0.866   0.000 
      0.000   0.000   1.000 
  same as Chrono             true
@endcode

## What makes a matrix a rotation

Not every matrix is a rotation. A rotation keeps lengths and angles, and does not mirror things. That
gives two conditions:

\f[
R^T R = I \quad \text{and} \quad \det(R) = +1
\f]

The first says that the columns are unit vectors at right angles to each other. The second says that
they form a right-handed frame. The first condition also gives us the most useful property of all:

\f[
R^{-1} = R^T
\f]

Inverting a rotation is just a transpose, and `R_RW` is `R_WR.transpose()`.

@code{.cpp}
bool is_rotation(const ChMatrix33d &R, real_t tolerance = 1.0e-9)
{
    const bool orthogonal = (R.transpose() * R - ChMatrix33d(1.0)).norm() < tolerance;
    return orthogonal && std::abs(R.determinant() - 1.0) < tolerance;
}
@endcode

The two counter-examples below fail for different reasons. Flipping an axis passes the first condition
but gives \f$\det(R) = -1\f$, a mirror image. Scaling fails the first condition since it changes lengths.

@code
2. What makes a matrix a rotation
  R_WR
      0.000  -1.000   0.000 
      1.000   0.000   0.000 
      0.000   0.000   1.000 
  x axis of R                (0.000, 1.000, 0.000)
  y axis of R                (-1.000, 0.000, 0.000)
  z axis of R                (0.000, 0.000, 1.000)
  R_WR is a rotation         true
  R_RW * R_WR = I            true
  flipped z is a rotation    false
  scaled is a rotation       false
@endcode

## Rotating vectors

A rotation matrix re-expresses a vector in another frame. Take a robot that drives forward: in its own
frame the velocity is always along \f$x\f$, but in the world it depends on where the robot is facing.

\f[
v_W = R_{WR} \, v_R
\f]

It is the same physical vector written in different coordinates, so its length does not change. Unlike
the points of the previous lesson there is no translation here: velocities are directions, and only the
rotation applies.

@code
3. Rotating vectors
  v in R                     (0.500, 0.000, 0.000)
  v in W                     (0.000, 0.500, 0.000)
  length in R                0.500
  length in W                0.500
  v back in R                (0.500, 0.000, 0.000)
@endcode

## Composing rotations

This is the most important section of the lesson. **The order of rotations matters.**

We want the sensor to look to the left of the robot and 30 degrees down, so we need one turn and one
tilt. Reading a product from left to right, each rotation is applied around the axes of the frame left
by the previous one:

- `Rz(90) * Ry(30)` turns left, and then tilts around the \f$y\f$ axis of the turned sensor. The beam
  goes left and down, which is what we want.
- `Ry(30) * Rz(90)` tilts around the \f$y\f$ axis of the robot. Then the sensor is turned to look along
  that same axis, so the tilt only rolls the sensor around its own beam. The beam stays level.

The same two rotations give a different result. Chaining rotations works exactly as for frames:

@code{.cpp}
const ChMatrix33d R_RS = rot_z(90.0 * DEG) * rot_y(30.0 * DEG);
const ChMatrix33d R_WS = R_WR * R_RS;
const ChVector3d beam_in_world = R_WS * ChVector3d(1.0, 0.0, 0.0);
@endcode

The beam is the \f$x\f$ axis of the sensor. Knowing its direction in the world, and the sensor position
from the previous lesson, we can work out where it hits the floor. Try this by hand: the sensor is 0.1 m
high and the beam drops 0.5 m per metre travelled along it, so the range is 0.2 m.

@code
4. Composing rotations
  Rz(90) * Ry(30)
      0.000  -1.000   0.000 
      0.866   0.000   0.500 
     -0.500   0.000   0.866 
  Ry(30) * Rz(90)
      0.000  -0.866   0.500 
      1.000   0.000   0.000 
      0.000   0.500   0.866 
  beam of Rz(90) * Ry(30)    (0.000, 0.866, -0.500)
  beam of Ry(30) * Rz(90)    (0.000, 1.000, 0.000)
  R_WS is a rotation         true
  beam in W                  (-0.866, 0.000, -0.500)
  sensor in W                (1.800, 1.000, 0.100)
  beam hits the floor at     (1.627, 1.000, 0.000)
@endcode

@note Eigen evaluates `A * B` lazily, and Chrono's matrix-vector `operator*` needs an actual matrix, so
`A * B * v` does not compile. Store the product in a `ChMatrix33d` first, as above.

## Yaw, pitch and roll

Nine numbers are a lot for three degrees of freedom. A common compact description is yaw \f$\psi\f$,
pitch \f$\theta\f$ and roll \f$\phi\f$:

\f[
R = R_z(\psi) \, R_y(\theta) \, R_x(\phi)
\f]

Reading from the left: turn around \f$z\f$, then pitch around the new \f$y\f$, then roll around the new
\f$x\f$. The angles can be recovered from the matrix:

\f[
\psi = \text{atan2}(R_{10}, R_{00}) \quad
\theta = -\arcsin(R_{20}) \quad
\phi = \text{atan2}(R_{21}, R_{22})
\f]

These formulas break down at \f$\theta = \pm 90^\circ\f$. This is **gimbal lock**. With the pitch at
\f$90^\circ\f$ the yaw and the roll turn around the same axis and only their difference matters, so the
angles \f$(30, 90, 0)\f$ and \f$(50, 90, 20)\f$ give exactly the same matrix. The rotation itself is fine,
it is the three-angle description that fails. That is one reason why simulators store orientation as a
quaternion, as Chrono does.

@code
5. Yaw, pitch and roll
  Rz(90) * Ry(10) * Rx(5)
      0.000  -0.996   0.087 
      0.985   0.015   0.173 
     -0.174   0.086   0.981 
  recovered from R           yaw=90.0 pitch=10.0 roll=5.0
  same as Chrono quaternion  true
  gimbal lock: same matrix   true
@endcode

@warning Angle conventions differ between libraries, so always verify them. We build the matrix by
composing elementary rotations because the conventions of the Chrono 9.0.1 helpers are easy to get
wrong. `ChQuaternion::SetFromCardanAnglesZYX` matches the formula above but takes its angles in the
order `(pitch, roll, yaw)`, and `ChQuaternion::GetCardanAnglesZYX` returns them in that order too.
`ChMatrix33::SetFromCardanAnglesZYX` and `QuatFromAngleSet` with `CARDAN_ANGLES_ZYX` do not produce
\f$R_z R_y R_x\f$ at all: for an input \f$(a, b, c)\f$ they give \f$R_y(-a) R_x(b) R_z(c)\f$.

## The rotation matrix of a moving body

`CHRONO_RobotPose::rotation_matrix()` gives the current `R_WR` of a body. It changes as the simulation
advances, but `R_RS` does not because the sensor is rigidly mounted. After turning the robot by
\f$90^\circ\f$ the beam points somewhere else, and the matrix is still a proper rotation.

@code{.cpp}
bitrl::rb::bitrl_chrono::CHRONO_RobotPose pose(robot);

const ChMatrix33d &R_WR = pose.rotation_matrix();
const ChMatrix33d R_WS = R_WR * R_RS;
@endcode

@code
6. The rotation matrix of a moving body
  t = 0.0 s
  R_WR
      0.000  -1.000   0.000 
      1.000   0.000   0.000 
      0.000   0.000   1.000 
  R_WR is a rotation         true
  beam in W                  (-0.866, 0.000, -0.500)
  t = 1.0 s
  R_WR
     -1.000   0.000   0.000 
      0.000  -1.000   0.000 
      0.000   0.000   1.000 
  R_WR is a rotation         true
  beam in W                  (0.000, -0.866, -0.500)
@endcode

## Running the example

@code
./example_26
./example_26 --render
./example_26 --gimbal
@endcode

With `--render` an Irrlicht window shows the two orders of section 4 side by side, and repeats the
motion. Both start from the same pose and both end up with the same two rotations applied, but only one
of them reaches the floor. The panels show the matrix \f$R_{RS}\f$ as it is built, so you can check that
its first column is the beam.

- The red, green and blue axes are the sensor frame, and the yellow line is the beam, which is its
  \f$x\f$ axis. A circle marks where the beam hits the floor.
- The purple line is the axis of the current rotation. See how the first rotation moves the axis of the
  second one: this is what reading the product from left to right means.

In this scene the sensor sits on a 0.6 m mast, since at 0.1 m the beam hits the floor too close to the
robot to see. The robots face along the world \f$x\f$ axis, so what you see is \f$R_{RS}\f$ alone.

With `--gimbal` the window shows gimbal lock on a real gimbal: three nested rings, each spinning around
the axis of its colour. The blue ring is the yaw around \f$z\f$, the green ring is the pitch around
\f$y\f$ and the red ring is the roll around \f$x\f$. The body in the middle is the small frame.

Both gimbals start from a yaw of \f$30^\circ\f$. The left one pitches to \f$30^\circ\f$ and the right
one to \f$90^\circ\f$. Then both add \f$20^\circ\f$ of yaw, and finally \f$20^\circ\f$ of roll, which
takes the right one from \f$(30, 90, 0)\f$ to \f$(50, 90, 20)\f$ as in section 5.

- The white line is the yaw axis and the purple line is the roll axis. On the right they are the same
  line, and the blue and green rings lie in the same plane.
- On the left the roll moves the body somewhere new. On the right it simply undoes the yaw, and the
  matrix goes back to what it was. The gimbal has lost a degree of freedom: in this position none of
  the rings can start turning the body around the axis perpendicular to the blue and green rings.

## Exercises

1. Tilt the sensor 30 degrees up instead of down. Does the beam ever hit the floor?
2. Use `to_yaw_pitch_roll` on `R_WS` from section 4. Are the angles what you expected?
3. Try `to_yaw_pitch_roll` with the gimbal lock matrix `A`. What do you get, and why?
4. Multiply a 1 degree rotation by itself a million times and check `is_rotation` with a tight tolerance.
   Rounding errors accumulate. How would you repair the matrix?
