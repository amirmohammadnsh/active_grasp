# Closed-Loop Next-Best-View Planning for Target-Driven Grasping

This repository contains the implementation of our IROS 2022 submission, _"Closed-Loop Next-Best-View Planning for Target-Driven Grasping"_. [[Paper](http://arxiv.org/abs/2207.10543)][[Video](https://youtu.be/67W_VbSsAMQ)]

> **Ubuntu 22.04 re-implementation guide.** The original code [active_grasp](https://github.com/ethz-asl/active_grasp) was developed and
> tested on Ubuntu 20.04 with a native ROS Noetic install. This guide covers
> setting up and running the full experiment suite (simulation + real-world
> hardware) on Ubuntu 22.04 using ROS Noetic via the
> [RoboStack](https://github.com/robostack/ros_noetic) conda distribution, with
> all necessary compatibility patches.

---

## Prerequisites

- Ubuntu 22.04
- [Miniconda](https://docs.anaconda.com/miniconda/) or Anaconda

---

## Setup

### 1. Create Conda Environment

```bash
conda create -n ag_env -c conda-forge -c robostack-noetic ros-noetic-desktop
conda activate ag_env
conda config --env --add channels robostack-noetic
conda config --env --remove channels defaults
```

### 2. Install Build Tools & ROS Packages

```bash
conda install -c conda-forge ros-dev-tools compilers cmake pkg-config make \
                ninja catkin_tools nlopt poco eigen

conda install -c robostack-noetic ros-noetic-ros-control \
                ros-noetic-ros-controllers ros-noetic-realtime-tools \
                ros-noetic-control-toolbox ros-noetic-joint-limits-interface

conda install -c conda-forge -c robostack-noetic ros-noetic-moveit \
                ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

### 3. Install Python ML & Simulation Dependencies

```bash
conda install -c conda-forge pytorch scipy numpy open3d scikit-image pandas tqdm
pip install catkin_pkg
```

### 4. Build libfranka

The physical Franka Emika Panda arm requires libfranka. Build it from source
with GCC 14 compatibility patches:

```bash
cd ~
git clone --recursive -b 0.8.0 https://github.com/frankaemika/libfranka.git
cd libfranka

# Patch C++ standard to C++14 for GCC 14 compatibility
find . -type f -name "CMakeLists.txt" -exec sed -i 's/c++11/c++14/g' {} \;
find . -type f -name "CMakeLists.txt" -exec sed -i 's/CXX_STANDARD 11/CXX_STANDARD 14/g' {} \;
find . -type f -name "CMakeLists.txt" -exec sed -i 's/Eigen3::Eigen3/Eigen3::Eigen/g' {} \;

# Add missing system headers
sed -i '1i #include <stdexcept>' src/control_types.cpp
sed -i '1i #include <string>' include/franka/control_tools.h

# Remove broken FindEigen3 cmake and build
rm cmake/FindEigen3.cmake
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DBUILD_TESTS=OFF \
      -DBUILD_EXAMPLES=OFF -DEigen3_DIR=$CONDA_PREFIX/share/eigen3/cmake ..
make -j$(nproc)
make install
```

### 5. Create Workspace & Clone Repositories

```bash
mkdir -p ~/ag_ws/src
cd ~/ag_ws/src
git clone https://github.com/ethz-asl/active_grasp.git
git clone https://github.com/ethz-asl/vgn.git
git clone https://github.com/mbreyer/robot_helpers.git
git clone -b master https://bitbucket.org/traclabs/trac_ik.git
git clone -b noetic-devel https://github.com/ros-planning/panda_moveit_config.git
git clone -b noetic-devel https://github.com/frankaemika/franka_ros.git
```

### 6. Install Python Dependencies

```bash
cd ~/ag_ws/src/active_grasp
pip install -r requirements.txt

cd ~/ag_ws/src/vgn
pip install -e . --no-build-isolation
```

### 7. Create boost_sml Package

The `franka_gazebo` node requires `boost/sml.hpp` which is not available in
modern distributions. Create a standalone ROS package to provide it:

```bash
cd ~/ag_ws/src
mkdir -p boost_sml/include/boost
cd boost_sml
wget https://raw.githubusercontent.com/boost-ext/sml/master/include/boost/sml.hpp \
     -O include/boost/sml.hpp
cp include/boost/sml.hpp include/boost_sml/sml.hpp

cat > package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>boost_sml</name>
  <version>1.1.0</version>
  <description>Boost.SML ROS Wrapper</description>
  <maintainer email="robotics@example.com">Robotics</maintainer>
  <license>BSL-1.0</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
EOF

cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(boost_sml)
find_package(catkin REQUIRED)
catkin_package(INCLUDE_DIRS include)
EOF
```

### 8. Apply Compatibility Patches

#### vgn

```bash
# Fix NumPy 2.x indexing (fancy indexing returns 2D array in newer NumPy)
sed -i 's/\[:, \[0\]\]/[:, 0]/g' ~/ag_ws/src/vgn/src/vgn/perception.py

# Handle both batched (1,40,40,40) and unbatched (40,40,40) TSDF input
python3 -c "
import re
with open('$HOME/ag_ws/src/vgn/src/vgn/detection.py') as f:
    c = f.read()
c = c.replace(
    'assert tsdf.shape == (40, 40, 40)',
    'if tsdf.shape == (1, 40, 40, 40):\n        tsdf = tsdf.squeeze(0)\n    assert tsdf.shape == (40, 40, 40)'
)
with open('$HOME/ag_ws/src/vgn/src/vgn/detection.py', 'w') as f:
    f.write(c)
"

# Python 3.12+ compatibility: tostring() removed in favor of tobytes()
sed -i 's/\.tostring()/.tobytes()/g' ~/ag_ws/src/vgn/src/vgn/utils.py
```

#### robot_helpers

```bash
# Duck-type Transform check (handles vgn's Transform vs robot_helpers' Transform)
RHM=~/ag_ws/src/robot_helpers/robot_helpers/ros/moveit.py
sed -i 's/isinstance(target, Transform)/hasattr(target, "rotation") and hasattr(target, "translation")/g' "$RHM"

# compute_cartesian_path third arg changed from float (0.0) to bool (True) in newer MoveIt
sed -i 's/compute_cartesian_path(waypoints, 0.01, 0.0)/compute_cartesian_path(waypoints, 0.01, True)/g' "$RHM"
```

#### active_grasp — nbv.py (Numba JIT Compatibility)

Newer versions of numba do not support returning `None` from a JIT-compiled
function or returning a dynamically-grown list. Replace the JIT functions and
update the class accordingly:

```bash
cat > ~/ag_ws/src/active_grasp/src/active_grasp/nbv.py << 'NBVEOF'
import itertools
import numpy as np
from numba import jit
import rospy

from .policy import MultiViewPolicy
from .timer import Timer


@jit(nopython=True)
def mark_visited(tsdf_grid, ori, pos, fx, fy, cx, cy,
                 u_min, u_max, v_min, v_max, t_min, t_max, t_step,
                 voxel_size, visited):
    for u in range(u_min, u_max):
        for v in range(v_min, v_max):
            direction = np.asarray([(u - cx) / fx, (v - cy) / fy, 1.0])
            direction = ori @ (direction / np.linalg.norm(direction))
            t = t_min
            tsdf_prev = -1.0
            while t < t_max:
                p = pos + t * direction
                t += t_step
                index = (p / voxel_size).astype(np.int64)
                if (index >= 0).all() and (index < 40).all():
                    i, j, k = index[0], index[1], index[2]
                    tsdf = tsdf_grid[i, j, k]
                    if tsdf * tsdf_prev < 0 and tsdf_prev > -1:
                        break
                    visited[i, j, k] = 1
                    tsdf_prev = tsdf


class NextBestView(MultiViewPolicy):
    def __init__(self):
        super().__init__()
        self.min_z_dist = rospy.get_param("~camera/min_z_dist")
        self.max_views = rospy.get_param("nbv_grasp/max_views")
        self.min_gain = rospy.get_param("nbv_grasp/min_gain")
        self.downsample = rospy.get_param("nbv_grasp/downsample")
        self._visited = np.zeros((40, 40, 40), dtype=np.int32)
        self._compile()

    def _compile(self):
        mark_visited(
            np.zeros((40, 40, 40), dtype=np.float32),
            np.eye(3), np.zeros(3), 1.0, 1.0, 1.0, 1.0,
            0, 1, 0, 1, 0.0, 1.0, 0.1, 1.0, self._visited,
        )

    def activate(self, bbox, view_sphere):
        super().activate(bbox, view_sphere)

    def update(self, img, x, q):
        if len(self.views) > self.max_views or self.best_grasp_prediction_is_stable():
            self.done = True
        else:
            with Timer("state_update"):
                self.integrate(img, x, q)
            with Timer("view_generation"):
                views = self.generate_views(q)
            with Timer("ig_computation"):
                gains = [self.ig_fn(v, self.downsample) for v in views]
            with Timer("cost_computation"):
                costs = [self.cost_fn(v) for v in views]
            utilities = gains / np.sum(gains) - costs / np.sum(costs)
            self.vis.ig_views(self.base_frame, self.intrinsic, views, utilities)
            i = np.argmax(utilities)
            nbv, gain = views[i], gains[i]
            if gain < self.min_gain and len(self.views) > self.T:
                self.done = True
            self.x_d = nbv

    def best_grasp_prediction_is_stable(self):
        if self.best_grasp:
            t = (self.T_task_base * self.best_grasp.pose).translation
            i, j, k = (t / self.tsdf.voxel_size).astype(int)
            qs = self.qual_hist[:, i, j, k]
            if np.count_nonzero(qs) == self.T and np.mean(qs) > 0.9:
                return True
        return False

    def generate_views(self, q):
        thetas = np.deg2rad([15, 30])
        phis = np.arange(8) * np.deg2rad(45)
        view_candidates = []
        for theta, phi in itertools.product(thetas, phis):
            view = self.view_sphere.get_view(theta, phi)
            if self.solve_cam_ik(q, view):
                view_candidates.append(view)
        return view_candidates

    def ig_fn(self, view, downsample):
        tsdf_grid, voxel_size = self.tsdf.get_grid(), self.tsdf.voxel_size
        tsdf_grid = -1.0 + 2.0 * tsdf_grid
        fx = self.intrinsic.fx / downsample
        fy = self.intrinsic.fy / downsample
        cx = self.intrinsic.cx / downsample
        cy = self.intrinsic.cy / downsample
        T_cam_base = view.inv()
        corners = np.array([T_cam_base.apply(p) for p in self.bbox.corners]).T
        u = (fx * corners[0] / corners[2] + cx).round().astype(int)
        v = (fy * corners[1] / corners[2] + cy).round().astype(int)
        u_min, u_max = u.min(), u.max()
        v_min, v_max = v.min(), v.max()
        t_min = 0.0
        t_max = corners[2].max()
        t_step = np.sqrt(3) * voxel_size
        view = self.T_task_base * view
        ori, pos = view.rotation.as_matrix(), view.translation
        self._visited.fill(0)
        mark_visited(
            tsdf_grid, ori, pos, fx, fy, cx, cy,
            u_min, u_max, v_min, v_max, t_min, t_max, t_step,
            voxel_size, self._visited,
        )
        bbox_min = self.T_task_base.apply(self.bbox.min) / voxel_size
        bbox_max = self.T_task_base.apply(self.bbox.max) / voxel_size
        indices = np.argwhere(self._visited == 1)
        mask = ((indices >= bbox_min) & (indices < bbox_max)).all(axis=1)
        i, j, k = indices[mask].T
        if len(i) == 0:
            return 0
        tsdfs = tsdf_grid[i, j, k]
        ig = np.logical_and(tsdfs > -1.0, tsdfs < 0.0).sum()
        return ig

    def cost_fn(self, view):
        return 1.0
NBVEOF
```

#### System: cv_bridge

```bash
# Patch cv_bridge for Python 3.12+ (tostring() was removed)
python3 -c "
import cv_bridge.core as m
p = m.__file__
with open(p) as f:
    c = f.read()
c = c.replace('.tostring()', '.tobytes()')
with open(p, 'w') as f:
    f.write(c)
print('Patched:', p)
"
```

### 9. Build Workspace

```bash
cd ~/ag_ws

# Standardize C++14 across all packages
find src -type f -name "CMakeLists.txt" -exec sed -i 's/c++11/c++14/g' {} \;
find src -type f -name "CMakeLists.txt" -exec sed -i 's/CXX_STANDARD 11/CXX_STANDARD 14/g' {} \;
find src -type f -name "CMakeLists.txt" -exec sed -i 's/c++0x/c++14/g' {} \;

# Fix missing cstdint include in franka_hw
sed -i '1i #include <cstdint>' src/franka_ros/franka_hw/include/franka_hw/resource_helpers.h

# Build
catkin build --cmake-args -DCMAKE_CXX_STANDARD=14 \
             -DCMAKE_CXX_FLAGS="-std=c++14 -I$CONDA_PREFIX/include/eigen3"

# Source the workspace
source devel/setup.bash
```

### 10. Download Pre-trained Models

```bash
cd ~/ag_ws/src/active_grasp
wget https://github.com/ethz-asl/active_grasp/releases/download/v1.0/assets.tar.gz
tar -xzf assets.tar.gz
rm assets.tar.gz
```

---

## Running Experiments

### Simulation

Open three terminals.

**Terminal 1 — ROS Core:**
```bash
conda activate ag_env
source ~/ag_ws/devel/setup.bash
roscore
```

**Terminal 2 — Environment:**
```bash
conda activate ag_env
cd ~/ag_ws
source devel/setup.bash
roslaunch active_grasp env.launch sim:=true
```

**Terminal 3 — Experiment Runner:**
```bash
conda activate ag_env
cd ~/ag_ws
source devel/setup.bash
python3 src/active_grasp/scripts/run.py nbv
```

### Real-World Hardware 


```bash
# Terminal 1
conda activate ag_env && source ~/ag_ws/devel/setup.bash && roscore

# Terminal 2 — Hardware drivers
conda activate ag_env && cd ~/ag_ws && source devel/setup.bash
roslaunch active_grasp hw.launch

# Terminal 3 — Environment node
conda activate ag_env && cd ~/ag_ws && source devel/setup.bash
roslaunch active_grasp env.launch sim:=false

# Terminal 4 — Experiment (--wait-for-input pauses between runs for safety)
conda activate ag_env && cd ~/ag_ws && source devel/setup.bash
python3 src/active_grasp/scripts/run.py nbv --wait-for-input
```
---

## Citation

```bibtex
@inproceedings{breyer2022closed,
  title={Closed-Loop Next-Best-View Planning for Target-Driven Grasping},
  author={Breyer, Michel and Chung, Jen Jen and Ott, Lionel and Siegwart, Roland and Nieto, Juan},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2022}
}
```

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file
for details. External dependencies (vgn, robot_helpers, trac_ik, etc.) have their
own licenses.