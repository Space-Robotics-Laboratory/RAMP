# RAMP: Reaction-Aware Motion Planning

![Build](https://github.com/MasazumiImai/RAMP/actions/workflows/build.yml/badge.svg)

This is a pure C++ library for locomotion of multi-limbed articulated robots, providing reaction-aware trajectory optimization and momentum distribution algorithms.

## Features

- **`ramp::lrst` (Low-Reaction Swing Trajectory)**: Generates optimal swing trajectories using Bézier curves and NLopt to minimize base reaction forces and moments.
- **`ramp::md` (Momentum Distribution)**: Analytically computes the optimal base velocity and joint velocities to distribute momentum across support limbs.

## Requirments

- [ROS 2](https://docs.ros.org/en/humble/index.html) (Humble)
- [NLopt](https://nlopt.readthedocs.io/en/latest/)

## Installation

```bash
# Clone repository
mkdir -p ~/ramp_ws/src
cd ~/ramp_ws/src
git clone https://github.com/MasazumiImai/RAMP.git

# Build
cd ~/ramp_ws
colcon build --packages-select ramp --symlink-install
source install/setup.bash
```

## Usage

RAMP uses a Dependency Injection (Callback) architecture. You need to provide the necessary calculations (IK, mass matrices) via `std::function`.

Example: TBA

## Citation

To cite RAMP in your academic publication, please use the following BibTeX entry:

```bibtex
@article{ribeiro2023ramp,
  title={RAMP: Reaction-aware motion planning of multi-legged robots for locomotion in microgravity},
  author={Ribeiro, Warley FR and Uno, Kentaro and Imai, Masazumi and Murase, Koki and Yoshida, Kazuya},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  pages={11845--11851},
  year={2023},
  organization={IEEE}
}
```

### Citing LRST

To cite LRST (Low-Reaction Swing Trajectory), please use the following BibTeX entry:

```bibtex
@inproceedings{ribeiro2022low,
  title={Low-reaction trajectory generation for a legged robot in microgravity},
  author={Ribeiro, Warley FR and Uno, Kentaro and Yoshida, Kazuya},
  booktitle={IEEE/SICE International Symposium on System Integration (SII)},
  pages={505--510},
  year={2022},
  organization={IEEE}
}
```

## Authors and Maintainers

- [Masazumi Imai](https://masazumiimai.github.io/)
- [Warley F. R. Ribeiro](https://www.ribeirowarley.com/)
- [Kentaro Uno](https://kentarouno.github.io/)
