// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;

public class Sub_Gyroscope extends SubsystemBase {
  /** Creates a new Sub_Gyroscope. */

  public double gyroAngle = 0;
  public double gyroRate = 0;

  // Uncomment simulation gyro and comment real gyro code below when
  // using simulated robot, and vice versa when using the real robot

  // The real gyro sensor
  public static final ADIS16470_IMU imuGyro = new ADIS16470_IMU(Constants.gyroAxis, Constants.imuPort, Constants.CalTime);

  // The simulation gyro sensor
  //public ADXRS450_Gyro imuGyro = new ADXRS450_Gyro();
  //public ADXRS450_GyroSim gyroSimHardW = new ADXRS450_GyroSim(imuGyro);

  public Sub_Gyroscope() {
  }

  public void getGyroValues() {
    gyroAngle = imuGyro.getAngle();
    gyroRate = imuGyro.getRate();
  }

  public void resetHeading() {
    imuGyro.reset();
  }

  public double fetchDirec() {
    return Math.IEEEremainder(imuGyro.getRotation2d().getDegrees(), 360);
  }

  public double fetchHeading() {
    return -imuGyro.getRotation2d().getDegrees();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getGyroValues();
    SmartDashboard.putNumber("gyroAngle@ ", (long) fetchDirec());
    SmartDashboard.putNumber("gyroRate@ ", (long) gyroRate);
  }
}
