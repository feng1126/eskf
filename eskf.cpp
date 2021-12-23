#include "eskf.h"

constexpr double kDegreeToRadian = PI / 180.;

eskf::eskf()
{
    error_state.x.setZero();
    error_state.p.setZero();
    error_state.p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-4;
    error_state.p.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-4;
    error_state.p.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-4;
    error_state.p.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-4;
    error_state.p.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-4;
    has_imu = false;
}

void eskf::getPose(State& state)
{
    state = state_;
}

bool eskf::InitPose(State state, Data data)
{
    state_.time = state.time;
    state_.p = state.p;
    state_.q = YPR2Quaterniond(data.ypr);
    state_.v = state.v;
    state_.bg = state.bg;
    state_.ba = state.ba;

    LastP = state.p;

    lastData = data;
    return true;
}

void eskf::updateXYZ(const Data XYZ, const double cov)
{

    double dp_noise = cov;
    double geo_x, geo_y, geo_z;
    Eigen::MatrixXd Y = Eigen::Matrix<double, 3, 1>::Zero();
    Y.block<3, 1>(0, 0) = state_.p - XYZ.xyz;
    //Y.block<3, 1>(3, 0) =  state_.v - current_gnss.v;
    Eigen::MatrixXd Gt = Eigen::Matrix<double, 3, 15>::Zero();
    Gt.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    //Gt.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> Ct = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::MatrixXd R = Eigen::Matrix<double, 3, 3>::Identity();
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dp_noise;
    //R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dp_noise * dp_noise;
    Eigen::MatrixXd K = error_state.p * Gt.transpose() * (Gt * error_state.p * Gt.transpose() + Ct * R * Ct.transpose()).inverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
    error_state.p = (I - K * Gt) * error_state.p;// *(I - K * Gt).transpose() + K * R * K.transpose();
    error_state.x = error_state.x + K * (Y - Gt * error_state.x);
    state_.p = state_.p - error_state.x.block<3, 1>(0, 0);
    state_.v = state_.v - error_state.x.block<3, 1>(3, 0);
    Eigen::Vector3d dphi_dir = error_state.x.block<3, 1>(6, 0);
    double dphi_norm = dphi_dir.norm();
    if (dphi_norm != 0)
    {
        dphi_dir = dphi_dir / dphi_norm;
        dphi_dir = dphi_dir * std::sin(dphi_norm / 2);
    }
    Eigen::Quaterniond temp2(std::cos(dphi_norm / 2), dphi_dir[0], dphi_dir[1], dphi_dir[2]);
    state_.q = temp2 * state_.q;
    state_.q.normalize();

    if (IsCovStable(9))
    {
        state_.bg = state_.bg + error_state.x.block<3, 1>(9, 0);
    }

    if (IsCovStable(12))
    {
        state_.ba = state_.ba + error_state.x.block<3, 1>(12, 0);
    }

    error_state.x.setZero();
    lastData.lla = XYZ.lla;
    lastData.ypr = XYZ.ypr;
    lastData.xyz = XYZ.xyz;
    state_.time = XYZ.timeStamp;
    LastP = state_.p;

}


bool eskf::updateGPS(const Data gnss)
{
    double dp_noise =  1e-11;
    double geo_x, geo_y, geo_z;
    Eigen::MatrixXd Y = Eigen::Matrix<double, 3, 1>::Zero();
    Y.block<3, 1>(0, 0) = state_.p - gnss.xyz;
    //Y.block<3, 1>(3, 0) =  state_.v - current_gnss.v;
    Eigen::MatrixXd Gt = Eigen::Matrix<double, 3, 15>::Zero();
    Gt.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    //Gt.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> Ct = Eigen::Matrix<double,3, 3>::Identity();

    Eigen::MatrixXd R = Eigen::Matrix<double, 3, 3>::Identity();
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity()  * dp_noise;
    //R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dp_noise * dp_noise;
    Eigen::MatrixXd K = error_state.p * Gt.transpose() * (Gt * error_state.p * Gt.transpose() + Ct * R * Ct.transpose()).inverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
    error_state.p = (I - K * Gt) * error_state.p;// *(I - K * Gt).transpose() + K * R * K.transpose();
    error_state.x = error_state.x + K * (Y - Gt * error_state.x);
    state_.p = state_.p - error_state.x.block<3, 1>(0, 0);
    state_.v = state_.v - error_state.x.block<3, 1>(3, 0);
    Eigen::Vector3d dphi_dir = error_state.x.block<3, 1>(6, 0);
    double dphi_norm = dphi_dir.norm();
    if (dphi_norm != 0)
    {
        dphi_dir = dphi_dir / dphi_norm;
        dphi_dir = dphi_dir * std::sin(dphi_norm / 2);
    }
    Eigen::Quaterniond temp2(std::cos(dphi_norm / 2), dphi_dir[0], dphi_dir[1], dphi_dir[2]);
    state_.q = temp2 * state_.q;
    state_.q.normalize();

    if (IsCovStable(9))
    {
        state_.bg = state_.bg + error_state.x.block<3, 1>(9, 0);
    }

    if (IsCovStable(12))
    {
        state_.ba = state_.ba + error_state.x.block<3, 1>(12, 0);
    }

    error_state.x.setZero();
    lastData.lla  = gnss.lla;
    lastData.ypr = gnss.ypr;
    lastData.xyz = gnss.xyz;
    state_.time = gnss.timeStamp;
    LastP = state_.p;
    return true;
}



bool eskf::updatelane(const Data gnss, const double cov)
{

    Eigen::Vector3d deta = gnss.xyz - lastData.xyz;
    double dist = deta.squaredNorm();
    if (dist >2.5) return false;

    //std::cout << "lane : " << gnss.xyz << std::endl;
    //std::cout << "lastData : " << lastData.xyz << std::endl;

    //std::cout << "dist : " << dist << std::endl;

    double dp_noise = cov;
    double geo_x, geo_y, geo_z;
    Eigen::MatrixXd Y = Eigen::Matrix<double, 3, 1>::Zero();
    Y.block<3, 1>(0, 0) = state_.p - gnss.xyz;
    //Y.block<3, 1>(3, 0) =  state_.v - current_gnss.v;
    Eigen::MatrixXd Gt = Eigen::Matrix<double, 3, 15>::Zero();
    Gt.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    //Gt.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> Ct = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::MatrixXd R = Eigen::Matrix<double, 3, 3>::Identity();
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dp_noise;
    //R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dp_noise * dp_noise;
    Eigen::MatrixXd K = error_state.p * Gt.transpose() * (Gt * error_state.p * Gt.transpose() + Ct * R * Ct.transpose()).inverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
    error_state.p = (I - K * Gt) * error_state.p * (I - K * Gt).transpose();// +K * R * K.transpose();
    error_state.x = error_state.x + K * (Y - Gt * error_state.x);
    state_.p = state_.p - error_state.x.block<3, 1>(0, 0);
    state_.v = state_.v - error_state.x.block<3, 1>(3, 0);
    Eigen::Vector3d dphi_dir = error_state.x.block<3, 1>(6, 0);
    double dphi_norm = dphi_dir.norm();
    if (dphi_norm != 0)
    {
        dphi_dir = dphi_dir / dphi_norm;
        dphi_dir = dphi_dir * std::sin(dphi_norm / 2);
    }
    Eigen::Quaterniond temp2(std::cos(dphi_norm / 2), dphi_dir[0], dphi_dir[1], dphi_dir[2]);
    state_.q = temp2 * state_.q;
    state_.q.normalize();

    if (IsCovStable(9))
    {
        state_.bg = state_.bg + error_state.x.block<3, 1>(9, 0);
    }

    if (IsCovStable(12))
    {
        state_.ba = state_.ba + error_state.x.block<3, 1>(12, 0);
    }

    error_state.x.setZero();
    lastData.ypr = gnss.ypr;
    state_.time = gnss.timeStamp;
    LastP = state_.p;
    return true;
}

bool eskf::updateVehicle(const Data vehicle, const double cov)
{
    Eigen::Vector3d vehicleEnu = vehicle.v;


    //std::cout << "ypr " << lastData.ypr.transpose() << std::endl;

    //std::cout << "vehicle " <<  vehicleEnu.transpose() << std::endl;
    // 
    vehicleEnu = (state_.q.toRotationMatrix()) * vehicleEnu;

    //std::cout << "vehicleEnu " << vehicleEnu.transpose() << std::endl;
    //double heading = lastData.ypr[0];
    //double vel = vehicle.v[0];
    //vehicleEnu = Eigen::Vector3d(vel * cos(heading), vel * sin(heading), 0);
    double dp_noise = cov;
    double geo_x, geo_y, geo_z;
    Eigen::MatrixXd Y = Eigen::Matrix<double, 3, 1>::Zero();
    Y.block<3, 1>(0, 0) = state_.v - vehicleEnu;
    //Y.block<3, 1>(3, 0) =  state_.v - current_gnss.v;
    Eigen::MatrixXd Gt = Eigen::Matrix<double, 3, 15>::Zero();
    //Gt.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Gt.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> Ct = Eigen::Matrix<double, 3, 3>::Identity();

    //std::cout << "Gt " << std::endl << Gt << std::endl;

    Eigen::MatrixXd R = Eigen::Matrix<double, 3, 3>::Identity();
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dp_noise;
    Eigen::MatrixXd K = error_state.p * Gt.transpose() * (Gt * error_state.p * Gt.transpose() + Ct * R * Ct.transpose()).inverse();

   // std::cout << "K " << std::endl << K << std::endl;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);

   // std::cout << "error_state.p 0 " << std::endl << error_state.p << std::endl;
    error_state.p = (I - K * Gt) * error_state.p;// *(I - K * Gt).transpose() + K * R * K.transpose();

   // std::cout << "error_state.p 1" << std::endl << error_state.p << std::endl;

    error_state.x = error_state.x + K * (Y - Gt * error_state.x);
    state_.p = state_.p - error_state.x.block<3, 1>(0, 0);
    state_.v = state_.v - error_state.x.block<3, 1>(3, 0);
    Eigen::Vector3d dphi_dir = error_state.x.block<3, 1>(6, 0);
    double dphi_norm = dphi_dir.norm();
    if (dphi_norm != 0)
    {
        dphi_dir = dphi_dir / dphi_norm;
        dphi_dir = dphi_dir * std::sin(dphi_norm / 2);
    }
    Eigen::Quaterniond temp2(std::cos(dphi_norm / 2), dphi_dir[0], dphi_dir[1], dphi_dir[2]);
    state_.q = temp2 * state_.q;
    state_.q.normalize();

    if (IsCovStable(9))
    {
        state_.bg = state_.bg + error_state.x.block<3, 1>(9, 0);
    }

    if (IsCovStable(12))
    {
        state_.ba = state_.ba + error_state.x.block<3, 1>(12, 0);
    }

    error_state.x.setZero();
    state_.time = vehicle.timeStamp;

    return true;
} 

bool eskf::ComputeG_R_IFromImuData(Eigen::Matrix3d& G_R_I, std::vector<IMUData> imu_buffer_)
{
    // Compute mean and std of the imu buffer.
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buffer_)
    {
        sum_acc += imu_data.acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buffer_)
    {
        sum_err2 += (imu_data.acc - mean_acc).cwiseAbs2();
    }
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > 100)
    {
        std::cout << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
        return false;
    }

    // Compute rotation.
    // Please refer to
    // https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp

    // Three axises of the ENU frame in the IMU frame.
    // z-axis.
    const Eigen::Vector3d& z_axis = mean_acc.normalized();

    // x-axis.
    Eigen::Vector3d x_axis =
        Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis.
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;

    G_R_I = I_R_G.transpose();

    return true;
}

Eigen::Matrix3d eskf::ComputeEarthTranform(const double& dt, const Eigen::Vector3d& velocity)
{
    double w = 7.27220521664304e-05;
    Eigen::Vector3d gravity_(0, 0, -9.79484197226504);
    Eigen::Vector3d w_ie_n(0, w * std::cos(lastData.lla[0] * EIGEN_PI / 180), w * std::sin(lastData.lla[0] * EIGEN_PI / 180));
    double rm = 6353346.18315;
    double rn = 6384140.52699;
    Eigen::Vector3d w_en_n(
        -velocity[1] / (rm + lastData.lla[2]),
        velocity[0] / (rn + lastData.lla[2]),
        velocity[0] / (rn + lastData.lla[2]) * std::tan(lastData.lla[0] * EIGEN_PI / 180));
    Eigen::Vector3d wtemp = (w_ie_n + w_en_n) * dt;
    Eigen::AngleAxisd angle_axisd(wtemp.norm(), wtemp.normalized());
    return angle_axisd.toRotationMatrix().transpose();
}

void eskf::predictNewState(const double& dt,const  Eigen::Vector3d& gyro, const  Eigen::Vector3d& acc)
{
    Eigen::Vector3d gn(0, 0, -9.79484197226504);
    Eigen::Vector3d pp = state_.p;
    Eigen::Vector3d vv = state_.v;
    Eigen::Quaterniond qq = state_.q;
    Eigen::Matrix3d R_nm_nm = ComputeEarthTranform(dt, vv);

    Eigen::Vector3d w_hat = 0.5 * (gyro + lastData.gyro) - state_.bg;
    Eigen::Vector3d a_hat = 0.5 * (acc + lastData.acc) - state_.ba;

    if (w_hat.norm() > 1e-5)
    {
        state_.q = qq * R_nm_nm * deltaQ(w_hat * dt);
    }
    else
    {
        state_.q = qq * R_nm_nm ;
    }

    state_.v = vv + dt * (state_.q * a_hat + gn);
    state_.p = pp + 0.5 * dt * (vv + state_.v);
}

void eskf::predictNewRkState(const double& dt, const  Eigen::Vector3d& gyro, const  Eigen::Vector3d& acc)
{

    //// TODO: Will performing the forward integration using
    ////    the inverse of the quaternion give better accuracy?
    //double gyro_norm = gyro.norm();
    //Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
    //Eigen::Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
    //Eigen::Omega.block<3, 1>(0, 3) = gyro;
    //Eigen::Omega.block<1, 3>(3, 0) = -gyro;

    //Eigen::Vector4d& q = state_server.imu_state.orientation;
    //Eigen::Vector3d& v = state_server.imu_state.velocity;
    //Eigen::Vector3d& p = state_server.imu_state.position;

    //// Some pre-calculation
    //Vector4d dq_dt, dq_dt2;
    //if (gyro_norm > 1e-5) {
    //    dq_dt = (cos(gyro_norm * dt * 0.5) * Matrix4d::Identity() +
    //        1 / gyro_norm * sin(gyro_norm * dt * 0.5) * Omega) * q;
    //    dq_dt2 = (cos(gyro_norm * dt * 0.25) * Matrix4d::Identity() +
    //        1 / gyro_norm * sin(gyro_norm * dt * 0.25) * Omega) * q;
    //}
    //else {
    //    dq_dt = (Matrix4d::Identity() + 0.5 * dt * Omega) *
    //        cos(gyro_norm * dt * 0.5) * q;
    //    dq_dt2 = (Matrix4d::Identity() + 0.25 * dt * Omega) *
    //        cos(gyro_norm * dt * 0.25) * q;
    //}
    //Matrix3d dR_dt_transpose = quaternionToRotation(dq_dt).transpose();
    //Matrix3d dR_dt2_transpose = quaternionToRotation(dq_dt2).transpose();

    //// k1 = f(tn, yn)
    //Vector3d k1_v_dot = quaternionToRotation(q).transpose() * acc +
    //    IMUState::gravity;
    //Vector3d k1_p_dot = v;

    //// k2 = f(tn+dt/2, yn+k1*dt/2)
    //Vector3d k1_v = v + k1_v_dot * dt / 2;
    //Vector3d k2_v_dot = dR_dt2_transpose * acc +
    //    IMUState::gravity;
    //Vector3d k2_p_dot = k1_v;

    //// k3 = f(tn+dt/2, yn+k2*dt/2)
    //Vector3d k2_v = v + k2_v_dot * dt / 2;
    //Vector3d k3_v_dot = dR_dt2_transpose * acc + IMUState::gravity;
    //Vector3d k3_p_dot = k2_v;

    //// k4 = f(tn+dt, yn+k3*dt)
    //Vector3d k3_v = v + k3_v_dot * dt;
    //Vector3d k4_v_dot = dR_dt_transpose * acc +
    //    IMUState::gravity;
    //Vector3d k4_p_dot = k3_v;

    //// yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)
    //q = dq_dt;
    //quaternionNormalize(q);
    //v = v + dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
    //p = p + dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

    //state_.p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
    //state_.v = v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;
    //state_.q = R_nm_nm *  quat_2_Rot(new_q);
}

bool eskf::Predict(Data imuData)
{
    if (!has_imu)
    {
        lastData.acc = imuData.acc;
        lastData.gyro = imuData.gyro;
        has_imu = true;
        return false;
    }
    double gyro_noise = 1e-6;
    double acc_noise = 1e-5;

    double dt = imuData.timeStamp - state_.time;

    Eigen::Vector3d gyro =  0.5 * (imuData.gyro + lastData.gyro) - state_.bg;
    Eigen::Vector3d acc = 0.5 * (imuData.acc + lastData.acc) - state_.ba;

   predictNewState(dt, imuData.gyro, imuData.acc);

    //predictNewRkState(dt, imuData.gyro, imuData.acc);

    Eigen::Matrix<double, 15, 15> Ft = Eigen::Matrix<double, 15, 15>::Zero();
    Ft = Eigen::Matrix<double, 15, 15>::Zero();
    Ft.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    Ft.block<3, 3>(3, 6) = state_.q * skewSymmetric(acc); //F23
    Ft.block<3, 3>(3, 12) =  state_.q.toRotationMatrix(); //Cbn

    double wsl = sin(lastData.lla[0] * PI / 180); 
    double wcl = cos(lastData.lla[0] * PI / 180);
    Eigen::Vector3d wl = Eigen::Vector3d(0, -wcl, -wsl);
    Ft.block<3, 3>(6, 6) = skewSymmetric(wl); //F33
    Ft.block<3, 3>(6, 9) = -state_.q.toRotationMatrix();


    Eigen::Matrix<double, 15, 6> Bt = Eigen::Matrix<double, 15, 6>::Zero();
    Bt.block<3, 3>(3, 3) = state_.q.toRotationMatrix();//Cbn
    Bt.block<3, 3>(6, 0) = -state_.q.toRotationMatrix();//-Cbn
    Ft = Eigen::Matrix<double, 15, 15>::Identity() + Ft * dt;

    Bt = Bt * dt;
    Eigen::Matrix<double, 6, 1> W = Eigen::Matrix<double, 6, 1>::Zero();
    W.head(3) = Eigen::Vector3d(gyro_noise, gyro_noise, gyro_noise);
    W.tail(3) = Eigen::Vector3d(acc_noise, acc_noise, acc_noise);
    error_state.x = Ft * error_state.x + Bt * W;
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity();
    Q.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity() * gyro_noise * gyro_noise;
    Q.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * acc_noise * acc_noise;
    error_state.p = Ft * error_state.p * Ft.transpose() + Bt * Q * Bt.transpose();
    state_.time = imuData.timeStamp;
    lastData.acc = imuData.acc;
    lastData.gyro = imuData.gyro;


    std::cout << "imu 0 " << (imuData.acc).transpose() << std::endl;
    std::cout << "imu 1 " << (imuData.acc - state_.ba).transpose() << std::endl;
    return true;
}


void eskf::updateRPY(const Data& RPY,const double cov)
{

    state_.time = RPY.timeStamp;
    double dp_noise = cov;
    Eigen::VectorXd Y = Eigen::Matrix<double, 3, 1>::Zero();

    Eigen::Quaterniond  q_bNominal_bMeas;
    q_bNominal_bMeas = state_.q.toRotationMatrix().transpose() * YPR2Quaterniond(RPY.ypr) ;

    Y = ToEulerAngles(q_bNominal_bMeas);
    Eigen::MatrixXd Gt = Eigen::Matrix<double, 3, 15>::Zero();
    Gt.block(0, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::MatrixXd R = Eigen::Matrix<double, 3, 3>::Identity();
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dp_noise;

    Eigen::MatrixXd Ct = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::MatrixXd K = error_state.p * Gt.transpose() * (Gt * error_state.p * Gt.transpose() + Ct * R * Ct.transpose()).inverse();

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
    error_state.p = (I - K * Gt) * error_state.p * (I - K * Gt).transpose();// +K * R * K.transpose();
    error_state.x = error_state.x + K * (Y - Gt * error_state.x);

    state_.p = state_.p  -  error_state.x.block<3, 1>(0, 0);
    state_.v = state_.v - error_state.x.block<3, 1>(3, 0);

    Eigen::Vector3d dphi_dir = error_state.x.block<3, 1>(6, 0);

    Eigen::Matrix3d temp = YPR2Quaterniond(dphi_dir[0], dphi_dir[1], dphi_dir[2]).toRotationMatrix();
    state_.q = state_.q.toRotationMatrix() * temp  ;
    state_.q.normalize();


    if (IsCovStable(9))
    {
        state_.bg = state_.bg + error_state.x.block<3, 1>(9, 0);
    }

    if (IsCovStable(12))
    {
        state_.ba = state_.ba + error_state.x.block<3, 1>(12, 0);
    }
    error_state.x.setZero();
}

void eskf::updateq(const Data& q)
{
    state_.time = q.timeStamp;
    double dp_noise = 1e-12;
    Eigen::VectorXd Y = Eigen::Matrix<double, 3, 1>::Zero();

    Eigen::Quaterniond  q_bNominal_bMeas;   
    q_bNominal_bMeas = YPR2Quaterniond(q.ypr) * state_.q.toRotationMatrix().inverse();

    Y = ToEulerAngles(q_bNominal_bMeas);

    Eigen::MatrixXd Gt = Eigen::Matrix<double, 3, 15>::Zero();
    Gt.block(0, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::MatrixXd R = Eigen::Matrix<double, 3, 3>::Identity();
    R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity()  * dp_noise;

    Eigen::MatrixXd Ct = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::MatrixXd K = error_state.p * Gt.transpose() * (Gt * error_state.p * Gt.transpose() + Ct * R * Ct.transpose()).inverse();

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
    error_state.p = (I - K * Gt) * error_state.p * (I - K * Gt).transpose();// + K * R * K.transpose();// (I - KH)P(I - KH)' + KRK'
    error_state.x = error_state.x + K * (Y - Gt * error_state.x);

    state_.p = state_.p - error_state.x.block<3, 1>(0, 0);
    state_.v = state_.v - error_state.x.block<3, 1>(3, 0);

    Eigen::Vector3d dphi_dir = error_state.x.block<3, 1>(6, 0);

    Eigen::Matrix3d temp = YPR2Quaterniond(dphi_dir[0], dphi_dir[1], dphi_dir[2]).toRotationMatrix();
    state_.q = temp * state_.q;
    state_.q.normalize();


     if (IsCovStable(9))
    {
        state_.bg = state_.bg + error_state.x.block<3, 1>(9, 0);
    }

     if (IsCovStable(12))
    {
        state_.ba = state_.ba + error_state.x.block<3, 1>(12, 0);
    }
    error_state.x.setZero();

}

bool eskf::IsCovStable(int INDEX_OFSET)
{
    for (int i = 0; i < 3; ++i)
    {
        if (error_state.p(INDEX_OFSET + i, INDEX_OFSET + i) > 1.0e-11)
        {
            return false;
        }
    }
    return true;
}

