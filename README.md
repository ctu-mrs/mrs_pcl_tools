# MRS PointCloudLibrary tools


Package grouping smaller nodes for processing, filtering, and general online/offline work with pointclouds.

> :warning: **Attention please: Work in progress.**


> :warning: **Attention please: This README is outdated.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.


## Dependencies

* [ouster-ros](https://github.com/ctu-mrs/ouster-ros) for Ouster point type.


## ToDo
- [ ] custom srv
- [ ] Launch files
- [ ] cfg params
- [ ] config params
- [ ] libraries
    - [ ] MrsPclTools_PCLSupportLib
    - [ ] MrsPclTools_PCLFiltration
    - [ ] MrsPclTools_PCL2MapRegistration
    - [ ] MrsPclTools_GroundplaneDetector
    - [ ] MrsPclTools_RemoveBelowGroundFilter
    - [ ] MrsPclTools_PCLPublishCloudFileToNetwork
- [ ] executables
    - [ ] estimate_cloud_to_cloud_drift
    - [ ] estimate_lidar_slam_drift
    - [ ] pcd_estimate_normals
    - [ ] concatenate_lidar_scans