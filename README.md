# LOCUS

LOCUS (Lidar Odometry for Consistent operation in Uncertain Settings) is a Multi-Sensor Lidar-Centric Solution for High-Precision Odometry and 3D Mapping in Real-Time.

![alt text](readme.png)

# TODOS

- [ ] Rename all lo_frontend instances to locus
- [ ] Cleanup comments, the less the better, just keep the very relevant ones
- [ ] Cleanup unused parameters, class variables, publishers and subscribers 
- [ ] Bring in MDC in launch
- [ ] Remove filter_.Filter call in LidarCallback as nothing is being filtered internally anymore and remove point_cloud_filter package accordingly
- [ ] Simplify our robot-specific files for a generic robot (e.g. no husky/spot anymore) and define how you want to let the user load sensor extrinsic, either hardcoded or by TF
- [ ] Suggest parameters for open/closed space in yaml where relevant (e.g. PointCloudOdometry, PointCloudLocalization)
- [ ] Update README.md with instructions on how to setup/run the package



# Cite
```
@article{reinke2022iros,
  title={LOCUS 2.0: Robust and Computationally Efficient LiDAR Odometry for Real-Time Underground 3D Mapping},
  author={Andrzej Reinke, Matteo Palieri, Benjamin Morrell, Yun Chang, Kamak Ebadi, Luca Carlone, Ali-akbar Agha-mohammadi},
  journal={TODO},
  year={2022},
  publisher={TODO}
}
```
