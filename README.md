# CoM-Aware Kinematic Character Controller

*A simplified human motion controller developed for the N8RO simulation environment.*

*This project implements a Center of Mass (CoM)-aware procedural animation controller using kinematic joint-angle control rather than rigid-body physics simulation.*

## Project Overview

The controller was developed as part of a simplified character animation and simulation assignment.

The implementation focuses on:

- procedural joint-angle generation
- CoM-aware posture control
- root-motion locomotion
- motion-state transitions
- procedural walk, run, jump, and pull animation

The system intentionally avoids:

- rigid-body dynamics
- force/torque computation
- inverse dynamics
- full contact physics

in accordance with the simplified assignment scope.

## Features

The controller currently supports:

* Center of Mass (CoM) computation
* CoM-aware posture adjustment
* procedural arm swing animation
* procedural leg swing animation
* torso sway motion
* automatic locomotion
* root-motion translation
* character turning/rotation
* smoothed animation blending
* motion-sequence state machine
* walk phase
* run phase
* jump phase
* pull phase
* automatic stop state
* simplified kinematic control pipeline

The animation system uses sinusoidal procedural motion combined with state-based behavior logic to generate human-like movement.

The controller executes an automated motion sequence consisting of:

* Walk
* Run
* Jump
* Pull
* Stop

Each phase modifies procedural animation parameters and root-motion behavior while remaining within a kinematic joint-angle control framework.

## System Architecture

The controller follows a simplified kinematic animation pipeline:

Input / Motion Sequence State Machine

↓

Center of Mass (CoM) Analysis

↓

Procedural Motion Generation

↓

Joint Angle Overrides

↓

Root Motion Output

Main controller responsibilities include:

* reading keyboard input
* updating the motion-sequence state machine
* computing Center of Mass (CoM)
* generating procedural joint rotations
* applying balance-aware posture adjustments
* generating locomotion and root motion
* producing root translation and rotation outputs

The controller uses a motion-sequence state machine to transition automatically between different movement behaviors.

The sequence progresses through:

* Walk
* Run
* Jump
* Pull
* Stop

Each state generates unique procedural joint motions and root-motion outputs while maintaining a fully kinematic control architecture.

The Center of Mass (CoM) is continuously evaluated and used to perform balance-aware posture adjustments throughout the motion sequence.

The implementation is fully kinematic and does not use force-based simulation, rigid-body dynamics, inverse dynamics, or contact-physics calculations.

## Center of Mass (CoM) Computation

The controller computes an approximate Center of Mass (CoM) for the humanoid character using weighted body-segment positions.

The CoM is estimated using a simplified weighted-average model:

CoM = Σ(mᵢ · pᵢ) / Σ(mᵢ)

where:

- mᵢ represents the estimated mass of a body segment
- pᵢ represents the world-space position of the segment

The implementation uses major body segments including:

- upper arms
- lower arms
- thighs
- calves
- feet

The computed CoM is used to:

- evaluate character balance
- trigger posture adjustments
- support balance-aware locomotion behavior

This approach follows the simplified assignment requirement of CoM-aware kinematic motion control without rigid-body dynamics.

## Procedural Animation

The controller uses procedural sinusoidal animation to generate simplified human-like motion.

Arm, leg, and torso movement are generated using time-based sine-wave functions that simulate rhythmic locomotion behavior.

The system supports:

* procedural arm swing
* procedural leg swing
* torso sway motion
* automatic locomotion
* motion-sequence state transitions
* smooth animation interpolation

The controller includes an automatic motion-sequence state machine consisting of:

* Walk
* Run
* Jump
* Pull
* Stop

Each state modifies procedural motion parameters such as animation frequency, swing amplitude, posture behavior, and root-motion output.

Motion phases are executed automatically over a one-minute sequence:

| Time | State |
| --- | --- |
| 0-15 s | Walk |
| 15-30 s | Run |
| 30-40 s | Jump |
| 40-60 s | Pull |
| 60+ s | Stop |

During the walk phase, moderate swing amplitudes are used to simulate normal locomotion.

During the run phase, animation frequency and swing amplitudes increase to produce faster movement.

During the jump phase, the controller generates elevated motion patterns to simulate a jumping action.

During the pull phase, slower procedural motion is used to simulate a pulling behavior.

After the sequence completes, the controller transitions into a stop state where procedural motion gradually settles.

Animation smoothing is applied to arm swing, leg swing, and torso sway motion to reduce abrupt transitions and create more natural movement behavior.

The procedural animation system remains fully kinematic and does not rely on force-based simulation, rigid-body dynamics, or contact-physics calculations.

## Build Instructions

The project is built using:

- Visual Studio 2022
- CMake
- C++17

### Build Steps

1. Open the project folder in Visual Studio 2022.
2. Allow Visual Studio to configure the CMake project.
3. Select the `x64-Release` build configuration.
4. Build the project using:

Build → Rebuild All

5. The generated plugin DLL will be produced by the CMake build system. Depending on the Visual Studio configuration, the DLL may be located inside the build output directory, for example `build/Release`.

### Project Structure

character-plugin-230201904/

├── src/

├── include/

├── docs/

├── tests/

├── README.md

└── CMakeLists.txt

## Simplified Scope Decisions

This project follows the revised simplified assignment scope provided for the N8RO simulation environment.

The implementation focuses on:

* kinematic joint-angle control
* procedural animation
* Center of Mass (CoM)-aware posture behavior
* simplified locomotion control
* motion-sequence state transitions
* root-motion generation

The following advanced systems were intentionally excluded from the implementation:

* rigid-body dynamics
* force and torque computation
* inverse dynamics
* full inverse kinematics (IK)
* detailed contact physics simulation
* biomechanical force modeling

The controller instead uses:

* direct joint-angle generation
* procedural motion synthesis
* state-based motion sequencing
* CoM-aware balance adjustments
* root-motion control
* animation smoothing and interpolation

Additional procedural motion phases (walk, run, jump, pull, and stop) were implemented as state-based kinematic behaviors while remaining within the assignment's joint-angle control framework.

The controller executes an automated motion sequence consisting of:

* Walk
* Run
* Jump
* Pull
* Stop

This simplified design provides a clean and focused implementation of balance-aware kinematic human motion control while demonstrating procedural animation generation, locomotion control, and Center of Mass analysis.

## Motion Sequence Timeline

The controller executes the following automated motion sequence:

| Time | State |
| --- | --- |
| 0-15 s | Walk |
| 15-30 s | Run |
| 30-40 s | Jump |
| 40-60 s | Pull |
| 60+ s | Stop |

The sequence is implemented using a finite-state machine and progresses automatically during simulation.

Each phase modifies procedural animation parameters, locomotion speed, and posture behavior while maintaining a fully kinematic control framework.

## Final Changes Added

The implementation was updated to fix integration and motion-output issues found during testing:

- valid axis-angle quaternion output was added instead of raw angle values
- all 10 assignment joints are driven through the exposed override indices
- the motion sequence is set to one minute: walk, run, jump, pull, then stop
- hotkey motion A starts/restarts the full sequence in the assignment SDK path
- the DLL output name was changed to `character_plugin_230201904.dll`
- N8RO 2.x sim-plugin exports were added: `create_plugin`, `destroy_plugin`, and `get_plugin_signature`
- the N8RO 2.x adapter registers procedural animations including
Idle Alert, Idle Neutral, Idle Breathing, and Idle Shake
- local N8RO testing used this plugin folder: `C:\N8RO\userPlugins\sim`
- local N8RO test files were backed up at `C:\N8RO\backup-character-plugin-230201904-20260529-214312`

