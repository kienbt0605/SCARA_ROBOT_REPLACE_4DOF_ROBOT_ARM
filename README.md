[cite_start]**THIS PROJECT IS STILL UNDER DEVELOPMENT AND IS INCOMPLETE** [cite: 1]

---

## 🤖 4-DOF Articulated Robot Arm

Hey! [cite_start]This project is all about building and controlling a 4-degree-of-freedom (4-DOF) robot arm[cite: 3]. [cite_start]It’s designed to handle 3D movements using a mix of hardware control and simulation[cite: 3, 5].

### 💻 System Overview
* **The Arm:** The physical robot's logic and movement are powered by **C code**.
* [cite_start]**The Simulation:** A **Python**-based environment used to visualize the arm in 3D, test trajectories, and calculate math before the real arm moves[cite: 3, 6].

### ⚙️ Robot Specs
[cite_start]The arm consists of four main joints: a base (J1), shoulder (J2), elbow (J3), and a wrist (J4)[cite: 4].
* [cite_start]**Structure:** It sits on a 75 mm base[cite: 29]. [cite_start]The arm itself has three main links with lengths of 90 mm, 90 mm, and 30 mm[cite: 28, 35].
* [cite_start]**Movement Range:** * **J1 (Base):** -180° to 180° [cite: 31]
    * [cite_start]**J2 (Shoulder):** -90° to 135° [cite: 32]
    * [cite_start]**J3 (Elbow):** -135° to 135° [cite: 33]
    * [cite_start]**J4 (Wrist):** -180° to 180° [cite: 34]

### 🚀 Key Features
* [cite_start]**Smart Targeting:** Uses Inverse Kinematics (IK) to find the exact joint angles needed to reach a specific (X, Y, Z) coordinate[cite: 12, 79].
* [cite_start]**Path Following:** The arm doesn't just jump to a spot; it can follow smooth straight lines or circular paths[cite: 13, 140, 152].
* [cite_start]**Live Feedback:** The system calculates the position of the end-effector in real-time using DH matrices[cite: 39, 71].

### 🛠️ Current Status & To-Do
* **Orientation:** Right now, we're focused on the position (X, Y, Z). [cite_start]Controlling the specific angle (pose) of the wrist is still in the works[cite: 179, 180].
* [cite_start]**Optimization:** We are looking into replacing numerical Jacobian calculations with analytical ones to make movements even faster and smoother[cite: 181, 186].
* [cite_start]**Real-time Sync:** Still perfecting the bridge between the Python simulation and the real-time servo control[cite: 190].
