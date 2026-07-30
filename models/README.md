# Adaptive Lane Keeping and Yaw Stability Control: AFS vs. Torque Vectoring

A MATLAB/Simulink and CarSim project comparing three vehicle-control configurations:

1. **Baseline steering control**
2. **Active Front Steering (AFS)**
3. **Torque Vectoring (TV)**

The controllers are evaluated using the same vehicle, reference trajectories, signals, and performance metrics. Two driving maneuvers are included:

- curved-lane tracking at **60 km/h** on a dry road;
- double-lane change at **70 km/h** on a dry road.

The repository contains the vehicle parameters, controller models, reference generators, bicycle-model simulation harnesses, CarSim co-simulation interfaces, automated result processing, and plotting scripts.

---

## Project objectives

The project investigates how two yaw-stability augmentation methods affect path tracking and vehicle stability:

- **AFS** modifies the commanded front steering angle;
- **TV** applies a corrective external yaw moment;
- **Baseline** uses only the nominal steering controller.

The comparison focuses on:

- vehicle trajectory;
- cross-track error;
- heading error;
- yaw-rate response;
- vehicle sideslip;
- lateral acceleration;
- steering demand;
- AFS steering correction;
- TV yaw-moment demand;
- longitudinal-speed regulation;
- verification of the Simulink-to-CarSim steering interface.

---

## Controller architecture

All controller variants use the same interface:

```text
Inputs:
    delta_base      nominal front-steering command
    Ref             reference trajectory bus
    VehicleState    measured vehicle-state bus

Outputs:
    delta_add       additional steering command
    Mz              corrective yaw moment
```

The total steering command is

```text
delta_f = delta_base + delta_add
```

The three controller modes are:

| Mode | Additional steering | Corrective yaw moment |
|---|---:|---:|
| Baseline | `delta_add = 0` | `Mz = 0` |
| AFS | `delta_add = delta_AFS` | `Mz = 0` |
| TV | `delta_add = 0` | `Mz = Mz_TV` |

### Baseline

The baseline configuration uses the nominal lane/path-following steering command without any additional yaw-stability control.

### Active Front Steering

The AFS controller computes a bounded steering correction and adds it to the nominal steering command. The implementation uses the online AFS controller and solver contained in:

```text
models/controllers/AFS.slx
src/solveAFSMPCOnline.m
```

### Torque Vectoring

The TV controller produces a corrective yaw-moment command using yaw-rate tracking, feedforward/feedback action, command saturation, and anti-windup.

In the CarSim implementation, TV is represented by the equivalent external yaw moment:

```text
IMP_MZ_EXT
```

It is therefore a **direct yaw-moment implementation**, not a wheel-level torque-allocation model.

---

## Vehicle model and common signals

The project uses an E-Class SUV parameter set in the MATLAB/Simulink models and the corresponding E-Class SUV dataset in CarSim.

The common vehicle-state bus contains:

```text
X       global longitudinal position [m]
Y       global lateral position [m]
psi     yaw/heading angle [rad]
Vx      longitudinal velocity [m/s]
Vy      lateral velocity [m/s]
r       yaw rate [rad/s]
beta    vehicle sideslip angle [rad]
ay      lateral acceleration [m/s^2]
```

The reference bus contains:

```text
X_ref
Y_ref
psi_ref
kappa_ref
Vx_ref
mu
```

---

## Simulation environments

### 1. MATLAB/Simulink bicycle model

The linear bicycle model is used for controller development, debugging, and initial comparison.

Relevant models include:

```text
models/plants/bicycle.slx
models/harness/experimentHarness.slx
models/harness/experimentHarnessDLC.slx
```

### 2. CarSim–Simulink co-simulation

CarSim provides the higher-fidelity vehicle response. Simulink generates the nominal steering command and the AFS or TV augmentation.

Relevant interface models include:

```text
models/harness/experimentHarnessCurvedLane_CarSim_Interface.slx
models/harness/experimentHarnessDLC_CarSim_Interface.slx
```

The CarSim database is not necessarily portable with the Git repository. The required CarSim datasets must be created or restored separately on the local computer.

---

## Test scenarios

| Scenario | Speed | Friction | Main geometry | Stop time | Metric interval |
|---|---:|---:|---|---:|---:|
| Curved lane | 60 km/h | 0.90 | 100 m radius; curve begins at 2 s | 10 s | 3–10 s |
| Double lane change | 70 km/h | 0.90 | 3.5 m lane displacement | 12 s | approximately 1.543–8.486 s |

The curved-lane parameters are defined in:

```text
config/scenarios/curvedLaneParams.m
```

The double-lane-change parameters are defined in:

```text
config/scenarios/DLCParams.m
```

---

## Repository structure

```text
AFS_vs_TV-main/
├── config/
│   ├── scenarios/
│   │   ├── curvedLaneParams.m
│   │   └── DLCParams.m
│   ├── controllerParams.m
│   ├── eClassParams.m
│   ├── referenceBus.m
│   └── stateBus.m
│
├── models/
│   ├── controllers/
│   │   ├── AFS.slx
│   │   └── TV.slx
│   │
│   ├── harness/
│   │   ├── experimentHarness.slx
│   │   ├── experimentHarnessDLC.slx
│   │   ├── experimentHarnessCurvedLane_CarSim_Interface.slx
│   │   └── experimentHarnessDLC_CarSim_Interface.slx
│   │
│   └── plants/
│       └── bicycle.slx
│
├── src/
│   ├── solveAFSMPCOnline.m
│   ├── runCurvedLaneComparison.m
│   ├── runDLCComparison.m
│   ├── createAllCurvedLanePlots_dt0005_v2.m
│   └── createAllDLCPlots_dt0005.m
│
├── results/
│   ├── CarSim_CurvedLane60_mu090_dt0005/
│   ├── CarSim_DLC70_mu090_dt0005/
│   └── ...
│
├── setupProject.m
└── README.md
```

Some working copies may contain additional development models, archived scripts, or intermediate results.

---

## Software requirements

The project was developed using:

- **MATLAB R2025b**
- **Simulink**
- **CarSim 2025.0**

Depending on the MATLAB installation and model configuration, additional products may be required.

A valid CarSim installation and license are required for the CarSim co-simulation cases.

---

## Getting started

Clone or download the repository and set the MATLAB current folder to the repository root.

```matlab
cd("path/to/AFS_vs_TV-main")
setupProject
```

`setupProject.m`:

- adds the configuration, model, and source folders to the MATLAB path;
- creates `VehicleStateBus` and `ReferenceBus`;
- loads the E-Class vehicle parameters;
- loads the controller parameters;
- defines the controller-selection constants;
- loads the default curved-lane scenario.

The controller-selection constants are:

```matlab
CTRL_BASELINE
CTRL_AFS
CTRL_TV
```

Select one controller using:

```matlab
controllerMode = CTRL_BASELINE;
% controllerMode = CTRL_AFS;
% controllerMode = CTRL_TV;
```

After changing `controllerMode`, update the model before running it:

```matlab
set_param(modelName, "SimulationCommand", "update");
```

---

## Running the bicycle-model simulations

### Curved lane

```matlab
setupProject

scenario = curvedLaneParams();
controllerMode = CTRL_BASELINE;

open_system("experimentHarness")
simOut = sim("experimentHarness", "StopTime", "10");
```

The automated bicycle-model comparison can be run with:

```matlab
results = runCurvedLaneComparison();
```

### Double lane change

```matlab
setupProject

scenario = DLCParams();
controllerMode = CTRL_BASELINE;

open_system("experimentHarnessDLC")
simOut = sim("experimentHarnessDLC", ...
    "StopTime", num2str(scenario.stopTime));
```

The automated bicycle-model comparison can be run with:

```matlab
resultsDLC = runDLCComparison();
```

---

## CarSim co-simulation configuration

### CarSim imports

The Simulink model sends the following commands to CarSim in this order:

| Index | CarSim variable | Unit | Mode |
|---:|---|---|---|
| 1 | `IMP_STEER_L1` | deg | Replace |
| 2 | `IMP_STEER_R1` | deg | Replace |
| 3 | `IMP_MZ_EXT` | N·m | Add |

### CarSim exports

CarSim returns the following outputs in this order:

| Index | CarSim variable | Unit |
|---:|---|---|
| 1 | `Xcg_TM` | m |
| 2 | `Ycg_TM` | m |
| 3 | `Yaw` | deg |
| 4 | `Vx` | km/h |
| 5 | `Vy` | km/h |
| 6 | `AVz` | deg/s |
| 7 | `Beta` | deg |
| 8 | `Ay` | g |
| 9 | `Steer_L1` | deg |
| 10 | `Steer_R1` | deg |

Do not change this order without updating the Simulink Demux and conversion blocks.

### Time-step configuration

The final CarSim runs use:

```text
CarSim math-model step:   0.0005 s
CarSim output-file step:  0.025 s
Simulink parent step:     0.0005 s
```

The two controller models use different fixed-step solvers:

```text
AFS:
    Solver type = Fixed-step
    Solver      = FixedStepDiscrete
    Step size   = 0.0005 s

TV:
    Solver type = Fixed-step
    Solver      = ode4
    Step size   = 0.0005 s
```

TV requires a fixed-step continuous solver because its model contains continuous states.

### Current-folder requirement

The MATLAB current folder must contain the CarSim-generated file:

```text
simfile.sim
```

A typical workflow is:

1. run the project setup script from the repository;
2. open the CarSim-linked Simulink model;
3. return MATLAB to the local CarSim database folder;
4. confirm `simfile.sim` exists;
5. run the simulation from Simulink or MATLAB.

Example:

```matlab
projectRoot = "path/to/AFS_vs_TV-main";
carSimRoot = "path/to/local/CarSim/database";

addpath(projectRoot, "-begin");
run(fullfile(projectRoot, "setupProject.m"));

cd(carSimRoot)

assert(isfile("simfile.sim"), ...
    "simfile.sim is missing from the active CarSim database.");
```

---

## Running the three CarSim controller cases

Run the controllers one at a time and keep all three runs in Simulation Data Inspector.

Suggested run names are:

### Curved lane

```text
CarSim_CurvedLane60_dt0005_Baseline
CarSim_CurvedLane60_dt0005_AFS
CarSim_CurvedLane60_dt0005_TV
```

### Double lane change

```text
CarSim_DLC70_dt0005_Baseline
CarSim_DLC70_dt0005_AFS
CarSim_DLC70_dt0005_TV
```

Before each run:

```matlab
controllerMode = CTRL_BASELINE;  % or CTRL_AFS / CTRL_TV
set_param(modelName, "SimulationCommand", "update");
```

After the simulation, rename the newest SDI run and keep it for comparison.

---

## Saving Simulation Data Inspector results

### Curved lane

Save the three runs to:

```text
results/CarSim_CurvedLane60_mu090_dt0005/
```

Recommended filenames:

```text
CurvedLane60_mu090_dt0005_Baseline_AFS_TV.mldatx
CurvedLane60_mu090_dt0005_AllRuns.mat
```

### Double lane change

Save the three runs to:

```text
results/CarSim_DLC70_mu090_dt0005/
```

Recommended filenames:

```text
DLC70_mu090_dt0005_Baseline_AFS_TV.mldatx
DLC70_mu090_dt0005_AllRuns.mat
```

Example export:

```matlab
Simulink.sdi.save(fullfile(resultsDir, "session.mldatx"));

runIDs = Simulink.sdi.getAllRunIDs;
Simulink.sdi.exportRun( ...
    runIDs, ...
    To="file", ...
    Filename=fullfile(resultsDir, "all_runs.mat"));
```

---

## Automated plot generation

### Curved-lane plots

Run:

```matlab
createAllCurvedLanePlots_dt0005_v2
```

The script loads the saved SDI session when available and creates:

```text
CL01_Trajectory.png
CL02_CrossTrack_Error.png
CL03_Yaw_Rate.png
CL04_Heading_Error.png
CL05_Sideslip.png
CL06_Lateral_Acceleration.png
CL07_Total_Steering.png
CL08_AFS_Control_Effort.png
CL09_TV_Yaw_Moment.png
CL10_Longitudinal_Speed.png
CL11_Steering_Verification.png
CurvedLane60_dt0005_metrics.csv
```

### Double-lane-change plots

Run:

```matlab
createAllDLCPlots_dt0005
```

The script creates:

```text
DLC01_Trajectory.png
DLC02_Lateral_Position.png
DLC03_CrossTrack_Error.png
DLC04_Heading_Response.png
DLC05_Yaw_Rate.png
DLC06_Sideslip.png
DLC07_Lateral_Acceleration.png
DLC08_Total_Steering.png
DLC09_AFS_Control_Effort.png
DLC10_TV_Yaw_Moment.png
DLC11_CarSim_Steering_Verification.png
DLC12_Longitudinal_Speed.png
DLC70_dt0005_metrics.csv
```

The scripts also save editable MATLAB `.fig` files.

---

## Performance metrics

The main tracking and stability errors are calculated as follows.

### Cross-track error

\[
e_y =
-\sin(\psi_{\mathrm{ref}})(X-X_{\mathrm{ref}})
+\cos(\psi_{\mathrm{ref}})(Y-Y_{\mathrm{ref}})
\]

### Heading error

\[
e_\psi =
\operatorname{atan2}
\left(
\sin(\psi_{\mathrm{ref}}-\psi),
\cos(\psi_{\mathrm{ref}}-\psi)
\right)
\]

### Yaw-rate reference

The path-based yaw-rate reference is based on:

\[
r_{\mathrm{ref}} = V_x\kappa_{\mathrm{ref}}
\]

and is limited by the available friction:

\[
|r_{\mathrm{ref}}|
\leq
\frac{\mu g}{\max(|V_x|,0.1)}
\]

Reported metrics include:

- RMS and peak cross-track error;
- RMS and peak heading error;
- RMS and peak yaw-rate error;
- peak sideslip angle;
- peak lateral acceleration;
- steering effort;
- AFS correction magnitude and rate;
- TV yaw-moment magnitude;
- actuator-saturation fraction where applicable.

---

## Expected controller invariants

The result-processing scripts assume the following conditions:

### Baseline

```text
delta_add = 0
Mz = 0
delta_f = delta_base
```

### AFS

```text
Mz = 0
delta_f = delta_base + delta_add
|delta_add| remains within the configured AFS limit
```

### TV

```text
delta_add = 0
delta_f = delta_base
Mz is nonzero when corrective yaw control is required
|Mz| remains within the configured TV limit
```

---

## Troubleshooting

### Undefined bus, controller, vehicle, or scenario variables

Run:

```matlab
setupProject
```

from the repository root.

### Red bus blocks or missing bus elements

Verify that both buses exist:

```matlab
whos VehicleStateBus ReferenceBus
```

Then update the model:

```matlab
set_param(modelName, "SimulationCommand", "update");
```

### AFS model-reference solver error

Use:

```matlab
set_param("AFS", ...
    "SolverType", "Fixed-step", ...
    "Solver", "FixedStepDiscrete", ...
    "FixedStep", "0.0005");
```

### TV contains continuous states

Do not use `FixedStepDiscrete` for TV. Use:

```matlab
set_param("TV", ...
    "SolverType", "Fixed-step", ...
    "Solver", "ode4", ...
    "FixedStep", "0.0005");
```

### `simfile.sim` is missing

Return MATLAB to the active local CarSim database folder:

```matlab
cd(carSimRoot)
assert(isfile("simfile.sim"))
```

### CarSim steering does not match the Simulink command

Verify:

- import order;
- export order;
- radian-to-degree conversion;
- commanded and actual left/right front-steering signals;
- that no CarSim closed-loop driver is simultaneously commanding steering.

### The plotting script selects the wrong SDI run

Delete or rename short test runs. The full runs should include `Baseline`, `AFS`, or `TV` in their names and should cover the complete maneuver duration.

### Hard-coded project fallback path

The plotting scripts first infer the project root from their location. Store them in:

```text
AFS_vs_TV-main/src/
```

If a fallback network path remains in a script, replace it with the appropriate local project path.

---

## Reproducibility notes

For a valid comparison:

- use the same vehicle dataset for all controllers;
- use the same road, friction, maneuver, speed, stop time, and time step;
- keep the nominal steering controller unchanged;
- change only the selected controller variant;
- retain the same signal names and unit conversions;
- keep all three controller runs in the same SDI session;
- calculate metrics over the same evaluation interval;
- preserve both raw SDI data and exported figures.

---

## Limitations

- TV is applied as an equivalent external yaw moment rather than through individual wheel-torque allocation.
- The CarSim database and proprietary vehicle datasets may not be included in the Git repository.
- Results are specific to the selected E-Class SUV parameters and the two tested maneuvers.
- Controller performance outside the tested speeds and friction conditions has not been established.
- The reference trajectory and the CarSim road geometry must remain aligned.
- The bicycle model is useful for controller development but does not capture all nonlinear tire, suspension, load-transfer, and powertrain effects present in CarSim.

---

## Git and large files

Simulink, SDI, and MATLAB result files are binary and may be large. Consider using Git LFS for:

```text
*.slx
*.mldatx
*.mat
```

Avoid committing temporary CarSim files, generated caches, or machine-specific database files unless they are required for reproducibility and permitted by the CarSim license.

A project-specific `.gitignore` may include:

```gitignore
# MATLAB generated files
slprj/
*.slxc
*.autosave
*.asv

# CarSim generated/runtime files
simfile.sim
*.par
*.echo
*.out

# Operating-system files
.DS_Store
Thumbs.db
```

Do not ignore result files that are intentionally included as final project evidence.

---

## Acknowledgment

This repository was developed as an ME 780 Vehicle System Dynamics project. It combines MATLAB/Simulink controller development with CarSim co-simulation to compare Active Front Steering and Torque Vectoring under common test conditions.

---

## License

No software license is specified in this repository. Add a `LICENSE` file before distributing or reusing the project outside its intended academic context.
