/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Sub_DriveTrain extends SubsystemBase {

  /**
   * Creates a new Sub_DriveTrain.
   */

  Spark mc_leftFront = null;
  Spark mc_leftBack = null;
  Spark mc_rightFront = null;
  Spark mc_rightBack = null;

  SpeedControllerGroup mc_leftGroup = null;
  SpeedControllerGroup mc_rightGroup = null;

  DifferentialDrive diffDrive = null;

  public double[][] speeds = { { 1.0, 0.9 }, { 0.8, 0.6 }, { 0.5, 0.3} };
  public int speedMode = 0; // 0 corresponds to fast, 1 corresponds to slow
  public boolean driveMode = true; // arcade or curvatureDrive
  private double lJoyInput = 0;
  private double rJoyInput = 0;
  private double avalVolts;
  private double largerVolt;

  public Sub_DriveTrain() {

    mc_leftFront = new Spark(Constants.m_leftFront);
    mc_leftBack = new Spark(Constants.m_leftBack);
    mc_rightFront = new Spark(Constants.m_rightFront);
    mc_rightBack = new Spark(Constants.m_rightBack);

    mc_leftGroup = new SpeedControllerGroup(mc_leftFront, mc_leftBack);
    mc_rightGroup = new SpeedControllerGroup(mc_rightFront, mc_rightBack); //might have to invert this 

    diffDrive = new DifferentialDrive(mc_leftGroup, mc_rightGroup);
  }

  public void drive(double speed, double turn, boolean mode) {
    if(mode) {
      diffDrive.arcadeDrive(speed, turn, false);
    } else {
      diffDrive.curvatureDrive(speed, turn, false);
    }
    lJoyInput = speed;
    rJoyInput = turn;
  }

  public void autoVoltDR(double leftVolts, double rightVolts) {
    if(RobotBase.isReal())  rightVolts *= -1;
    avalVolts = (long) RobotController.getBatteryVoltage();
    if (Math.max(leftVolts, rightVolts) >= avalVolts) {
      largerVolt = Math.max(Math.abs(leftVolts), Math.abs(rightVolts));

      mc_leftGroup.setVoltage(avalVolts * (leftVolts / largerVolt));
      mc_rightGroup.setVoltage(avalVolts * (rightVolts / largerVolt));
    } else { 
      mc_leftGroup.setVoltage(leftVolts);
      mc_rightGroup.setVoltage(rightVolts);
    }
    diffDrive.feed();
  }

  public void changeSpeed() {
    if(speedMode == 2) {
      speedMode = 0;
    } else {
      speedMode++;
    }
  }
  
  public void changeDrive() {
    if(driveMode) {
      driveMode = false;
    } else {
      driveMode = true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lGroupPWM@", 0.5 * (mc_rightFront.getSpeed() + mc_rightBack.getSpeed()));
    SmartDashboard.putNumber("rGroupPWM@", 0.5 * (mc_leftFront.getSpeed() + mc_leftBack.getSpeed()));
    SmartDashboard.putNumber("lJoystick Inputs:", lJoyInput);
    SmartDashboard.putNumber("rJoystick Inputs:", rJoyInput);
    SmartDashboard.putNumber("TeleOp speed is", speedMode);
  }
}
