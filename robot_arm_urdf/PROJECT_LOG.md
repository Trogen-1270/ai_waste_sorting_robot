# AI Waste Sorting Robot - Project Log

My personal log of commands, steps, and learnings for my capstone project.

---

## Phase 1: Initial Project & Git Setup

**Date:** 2025-07-03

**Goal:** Create the ROS workspace, set up the GitHub repository, and synchronize the initial tutorial code between my local PC and The Construct.

**Key Commands Used:**

* **On Local PC (PowerShell):**
    ```powershell
    # Cloned the repository from GitHub to my local machine
    git clone [https://github.com/Trogen-1270/ai_waste_sorting_robot.git](https://github.com/Trogen-1270/ai_waste_sorting_robot.git)

    # Navigated into the new directory and added the robot package
    cd D:\Users\TROGEN\Downloads\ai_waste_sorting_robot

    # Staged, committed, and pushed the `robot_arm_urdf` folder
    git add .
    git commit -m "feat: Initial commit of tutorial robot package"
    git push origin main
    ```

* **On The Construct (Terminal):**
    ```bash
    # Pulled the new files from GitHub into the Rosject
    cd ~/catkin_ws/src/ai_waste_sorting_robot
    git pull

    # Built the workspace
    cd ~/catkin_ws
    catkin_make

    # Sourced the environment to make the package available
    source devel/setup.bash
    ```

**Outcome:** Phase 1 was successful. The project is now synchronized and ready for development.

---

## Phase 2: Validate Simulation Pipeline

**Date:**
**Goal:** To get the tutorial robot model running and controllable in Gazebo and MoveIt!, following the provided PDF guide.

**Key Commands Used:**
*(You will fill this section in as we complete the steps)*

```bash
# Example: Creating the new branch
# git checkout -b feature/validate-tutorial-robot