# N8RO Procedural Human Kinematic Locomotion Controller

A high-fidelity C++ procedural simulation plugin for the Arkheon/N8RO environment. This project implements a closed-library character locomotion controller that determines exact joint angles for 10 major degrees of freedom (DOF) without requiring computationally expensive rigid-body dynamics, force calculations, or contact physics.

---

## 🚀 Key Biomechanical Features

### 1. Symmetrical Locomotion (Symmetric Sinusoidal Gait)
Locomotion is modeled using a continuous $180^\circ$ phase-shifted sinusoidal gait loop, ensuring both legs execute the **exact same physical travel** ($\pm 30^\circ$) and pass smoothly through the straight vertical standing posture ($0^\circ$):
* **Swing Phase (`phi < 0.5`)**: The leg is in the air, folding up to $-65^\circ$ at the knee for ground clearance, and returning straight.
* **Stance Phase (`phi >= 0.5`)**: The leg is on the ground, pushing backward to $-30^\circ$ at the hip, utilizing a minor $-8^\circ$ knee bend for impact absorption.
* **Ankle Timing**: Integrated heel-striking dorsiflexion ($+10^\circ$) transitioning to toe-off plantarflexion push ($-10^\circ$).

### 2. Counter-Pendulum Arm Logic
Arm swing is procedurally coupled to the gait phase to mirror human kinetics and preserve momentum:
* **Shoulder Swing (Z-Axis Pitch)**: Swings in exact opposition to the same-side hip.
* **Shoulder Abduction (X-Axis Roll)**: Held at a slight outward angle ($\pm 10^\circ$) to prevent clipping the civilian torso mesh.
* **Dynamic Elbows**: Modeled as dynamic levers (~$78^\circ$ to $100^\circ$) that automatically flex as the arm drives forward and extend as it swings back, mirroring true biomechanical efficiency.

---

## 🛠️ SDK & Coordinate Architecture

In accordance with the N8RO Rig specifications:
* **Radian Inputs**: The plugin converts all internal degrees into radians before outputting them to N8RO's SDK `AnimationJointOverride` structure (`xRad, yRad, zRad`).
* **Axis Convention**:
  * **Z-Axis**: Forward/backward pitch (hips, knees, ankles, shoulder swing).
  * **X-Axis**: Outward roll/abduction (shoulder abduction, elbow hinge).
* **Left-Side Non-Mirroring**: Left hips/joints use identical signs to the right side (rather than being negated), maintaining strict out-of-phase synchronization.

---

## 💻 Build & Deploment Instructions

### Prerequisites
* **Visual Studio 2022 Community** (with Desktop development with C++ enabled).
* **CMake 3.20+** (or the Visual Studio embedded CMake toolchain).

### Step 1: Configure CMake
Open developer command prompt/PowerShell and navigate to the project directory:
```bash
cmake -S . -B build -A x64
```

### Step 2: Compile the Plugin
Build the Release DLL configuration using MSBuild:
```bash
cmake --build build --config Release
```
This generates `kinematic_run_plugin.dll` in the `build/Release/` directory.

### Step 3: Deplay to N8RO
Deploy the DLL directly to N8RO's simulation plugin directory:
```powershell
Copy-Item -Path ".\build\Release\kinematic_run_plugin.dll" -Destination "C:\N8RO\userPlugins\sim\kinematic_run_plugin.dll" -Force
```

---

## 🧪 Mathematical Validation (Mock Host)
The codebase includes a lightweight math verification harness `mock_host.exe` that simulates the N8RO engine's ticks across a full gait cycle (at $2.5\text{ Hz}$). 

To run the validation test:
```powershell
# Set path to load N8RO runtime DLL dependencies
$env:PATH = "C:\N8RO\bin;" + $env:PATH
.\build\Release\mock_host.exe
```
This outputs precise joint radian values at each normalised phase step (`Phi = 0.00` to `0.90`) for complete regression testing.
