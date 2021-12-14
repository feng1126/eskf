#pragma once

#include <vector>
#include <iostream>
#include <Eigen/Geometry>
#include "data.h"

class eskf
{

public:
	bool Predict(Data imuData);
	bool updateGPS(const Data gnss);
	void updateq(const Data& q);
	void updateRPY(const Data& RPY);
	bool updatelane(const Data gnss);
	bool updateVehicle(const Data vehicle);
	bool InitPose(State state, Data data);
	void getPose(State& state);
	bool ComputeG_R_IFromImuData(Eigen::Matrix3d& G_R_I, std::vector<IMUData> imu_buffer_);
	eskf();
private:

	State state_;
	ErrorState error_state;
	Eigen::Matrix3d ComputeEarthTranform(const double& dt, const Eigen::Vector3d& velocity);
	void predictNewState(const double& dt,const Eigen::Vector3d& gyro,const Eigen::Vector3d& acc);
	void predictNewRkState(const double& dt, const  Eigen::Vector3d& gyro, const  Eigen::Vector3d& acc);
	Data lastData;
	bool has_imu;
	bool IsCovStable(int INDEX_OFSET);
	Eigen::Vector3d LastP;




};