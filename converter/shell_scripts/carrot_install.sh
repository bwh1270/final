#! /bin/bash

## Install the required Packages
sudo apt-get update
sudo apt-get install -y cmake doxygen libqt4-dev libqt4-opengl-dev libqglviewer-dev-qt4
sudo apt-get install -y ros-noetic-octomap

## Git clone
cd /root/uav_ws/src
git clone https://github.com/OctoMap/octomap_mapping.git
git clone -b noetic --single-branch https://github.com/ros-perception/image_pipeline.git

## Move carrot package
cd /root/uav_ws/src
mv -f final/converter .
mv -f final/have2move/octomap_mapping.launch octomap_mapping/octomap_server/launch/
mv -f final/have2move/convert_metric.launch image_pipeline/depth_image_proc/launch/
mv -f final/have2move/point_cloud_xyz.launch image_pipeline/depth_image_proc/launch/
mv -f final/have2move/image_proc.launch image_pipeline/image_proc/launch/
mv -f final/have2move/point_cloud_xyzrgb.launch image_pipeline/depth_image_proc/launch/
mv -f final/have2move/register.launch image_pipeline/depth_image_proc/launch/

## move start files into convert packages
mv -f final/carrot_start.sh final/xyz_session.yml final/xyzrgb_session.yml converter/

## Build
catkin build -j$(nproc) converter depth_image_proc image_proc octomap_server
