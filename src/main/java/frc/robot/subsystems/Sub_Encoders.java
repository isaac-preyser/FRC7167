/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sub_Encoders extends SubsystemBase {
  /**
   * Creates a new Sub_Encoders.
   */
  public Encoder leftEnc;
  public Encoder rightEnc;

  // public DifferentialDriveWheelSpeeds wheelSpeeds;
  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  public EncoderSim m_leftEncoderSim;
  public EncoderSim m_rightEncoderSim;

  //Recyclops Wheels are 6-inch wheels & AMT103 encoders are set to 2048 PPR (pulses per revolution or 1/4 EPR)
  public double distancePerPulse = 0.47876/2048; // WheelCircInM/PPR
  public double degreesPerPulse = 360/2048; //DegInACircle/PPR

  public Sub_Encoders() {

    leftEnc = new Encoder(Constants.encoder1A, Constants.encoder1B, true, Encoder.EncodingType.k2X);
    rightEnc = new Encoder(Constants.encoder2A, Constants.encoder2B, false, Encoder.EncodingType.k2X);

    if (RobotBase.isSimulation()) {
      m_leftEncoderSim = new EncoderSim(leftEnc);
      m_rightEncoderSim = new EncoderSim(rightEnc);
    }
  }

  public void reset() {
    leftEnc.reset();
    rightEnc.reset();
  }

  public void setDistanceScale(double scale) {
    leftEnc.setDistancePerPulse(scale);
    rightEnc.setDistancePerPulse(scale);
  }

  public double getDistanceAvg() {
    return (leftEnc.getDistance() + rightEnc.getDistance()) / 2;
  }

  public DifferentialDriveWheelSpeeds fetchWheelSpds() {
    return new DifferentialDriveWheelSpeeds(leftEnc.getRate(), rightEnc.getRate());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftEnc Distance:", (float) leftEnc.getDistance());
    SmartDashboard.putNumber("rightEnc Distance:", (float) rightEnc.getDistance());
    SmartDashboard.putNumber("leftEnc Rate", (float) leftEnc.getRate());
    SmartDashboard.putNumber("rightEnc Rate:", (float) rightEnc.getRate());
  }
}
