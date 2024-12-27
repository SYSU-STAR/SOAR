<div align="center">
    <h1> 
      <img src="assets/imgs/logo.svg" width="35" height="35" /> SOAR
    </h1>
    </h1>
    <h2>Simultaneous Exploration and Photographing with Heterogeneous UAVs for Fast Autonomous Reconstruction</h2>
    <strong>IROS 2024 Oral</strong>
    <br>
        <a href="http://sysu-star.com/people" target="_blank">Mingjie Zhang</a><sup>1,3,*</sup>,
        <a href="https://chen-albert-feng.github.io/AlbertFeng.github.io" target="_blank">Chen
            Feng</a><sup>2,*</sup>,
        <a href="http://sysu-star.com/people" target="_blank">Zengzhi Li</a><sup>1,4</sup>,
        <a href="http://sysu-star.com/people" target="_blank">Guiyong Zheng</a><sup>1</sup>,
        <a href="http://sysu-star.com/people" target="_blank">Yiming Luo</a><sup>1</sup>,
        <br>
        <a href="https://zdh.ncepu.edu.cn/szdw/fjs/2e5352e61fb648aa890d5aaaf1f1447f.htm" target="_blank">Zhu
            Wang</a><sup>4</sup>,
        <a href="https://facultyprofiles.hkust-gz.edu.cn/faculty-personal-page/ZHOU-Jinni/eejinni"
            target="_blank">Jinni Zhou</a><sup>5</sup>,
        <a href="https://uav.hkust.edu.hk/group" target="_blank">Shaojie Shen</a><sup>2</sup>,
        <a href="http://sysu-star.com/people" target="_blank">Boyu Zhou</a><sup>1,†</sup>
        <p>
        <h45>
            <sup>1</sup> Sun Yat-Sen University. &nbsp;&nbsp;
            <sup>2</sup> The Hong Kong University of Science and Technology. &nbsp;&nbsp;
            <br>
            <sup>3</sup> Northwestern Polytechnical University. &nbsp;&nbsp;
            <sup>4</sup> North China Electric Power University. &nbsp;&nbsp;
            <br>
            <sup>5</sup> The Hong Kong University of Science and Technology(Guang Zhou). &nbsp;&nbsp;
            <br>
        </h45>
        <sup>*</sup>Equal Contribution &nbsp;&nbsp;
        <sup>†</sup>Corresponding Authors
    </p>
    <a href="https://arxiv.org/abs/2409.02738"><img alt="Paper" src="https://img.shields.io/badge/Paper-arXiv-red"/></a>
    <a href='https://sysu-star.github.io/SOAR'><img src='https://img.shields.io/badge/Project_Page-SOAR-green' alt='Project Page'></a>
    <a href="https://www.bilibili.com/video/BV1G1421Q79m"><img alt="Bilibili" src="https://img.shields.io/badge/Video-Bilibili-blue"/></a>
    <a href="https://www.bilibili.com/video/BV1wEyHYjEAq"><img alt="Bilibili" src="https://img.shields.io/badge/Talk-Bilibili-purple"/></a>
</div>


## 📢 News

- **[09/10/2024]**: The code of SOAR is released. 
- **[30/06/2024]**: SOAR is accepted to IROS 2024 and selected as **oral presentation** (acceptance rate: 10%). 

## 📜 Introduction

**[IROS'24]** This repository maintains the implementation of "SOAR: Simultaneous Exploration and Photographing with Heterogeneous UAVs for Fast Autonomous Reconstruction".

The key modules of SOAR are detailed in this overview.
<p align="center">
  <img src="assets/videos/overview.gif" width = 90% height = 90%/>
</p>

And we also provide a special demo for IROS2024.
<p align="center">
  <img src="assets/videos/iros_demo.gif" width = 90% height = 90%/>
</p>

If you find this work useful in your research, please consider citing:

```bibtex
@INPROCEEDINGS{zhang2024soar,
  author={Zhang, Mingjie and Feng, Chen and Li, Zengzhi and Zheng, Guiyong and Luo, Yiming and Wang, Zhu and Zhou, Jinni and Shen, Shaojie and Zhou, Boyu},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={SOAR: Simultaneous Exploration and Photographing with Heterogeneous UAVs for Fast Autonomous Reconstruction}, 
  year={2024},
  pages={10975-10982}
}
```
Please kindly star ⭐️ this project if it helps you. We take great efforts to develop and maintain it 😁.

## 🛠️ Installation

### Prerequisite

* ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
* PCL
* Eigen

__For Marsim__:
```shell
sudo apt update
sudo apt install libglfw3-dev libglew-dev
```

__For GCOPTER__:
```shell
sudo apt update
sudo apt install libompl-dev
```

### Compilation

__Project__:

```shell
git clone https://github.com/SYSU-STAR/SOAR
cd SOAR
catkin_make
```
If you have installed ***Anaconda***, please use ```catkin_make  -DPYTHON_EXECUTABLE=/usr/bin/python3```.

__LKH-3.0.6__:
```shell
cd src/planner/utils/lkh_mtsp_solver/LKH
make
```

## 🚀 Quick Start

__Pisa Cathedral__
```shell
source devel/setup.bash && roslaunch heterogeneous_manager rviz.launch
source devel/setup.bash && roslaunch heterogeneous_manager pisa.launch
```

<p align="center">
  <img src="assets/videos/pisa.gif" width = 60% height = 60%/>
</p>

__Sydney Opera House__
```shell
source devel/setup.bash && roslaunch heterogeneous_manager rviz.launch
source devel/setup.bash && roslaunch heterogeneous_manager sydney.launch
```

<p align="center">
  <img src="assets/videos/sydney.gif" width = 60% height = 60%/>
</p>

__NOTE__: Trigger the quadrotors to start planning with the `2D Nav Goal` when the terminal displays `wait for trigger`. All scenes are provided in `src/heterogeneous_manager/launch/XXX.launch`

If you want to use the GPU version of MARSIM, you can change the parameter "use_gpu" to `true` in `src/heterogeneous_manager/launch/XXX.launch`

```xml
<arg name="use_gpu" value="true" />
```

## 🤓 Acknowledgments

- [FC-Planner](https://github.com/HKUST-Aerial-Robotics/FC-Planner)
- [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)
- [RACER](https://github.com/SYSU-STAR/RACER)
- [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)
- [MARSIM](https://github.com/hku-mars/MARSIM)

## 🤗 FC-family Works

#### 1. What is FC-family?

We aim to develop intelligent perception-centric flight to realize ***F***ast ***C***overage / re***C***onstruction / inspe***C***tion etc.

#### 2. Projects list

* [PredRecon](https://github.com/HKUST-Aerial-Robotics/PredRecon) (ICRA2023): Prediction-boosted Planner for Aerial Reconstruction.
* [FC-Planner](https://github.com/HKUST-Aerial-Robotics/FC-Planner) (ICRA2024): Highly Efficient Global Planner for Aerial Coverage.
* [SOAR](https://github.com/SYSU-STAR/SOAR) (IROS2024): Heterogenous Multi-UAV Planner for Aerial Reconstruction.
  
