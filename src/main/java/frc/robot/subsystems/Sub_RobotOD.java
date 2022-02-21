// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class Sub_RobotOD extends SubsystemBase {

  // Classes for Drivetrain Simulation
  DifferentialDrivetrainSim m_driveSim;
  DifferentialDriveOdometry m_odometry;

  public Trajectory autoTrajectory[] = {null, null, null, null, null, null, null, null};
  private Path traJLocation = null;
  private Field2d m_field = null;
  private int count = 0;
  public int i = 0;
  public boolean traJState = false;

  /** Creates a new Sub_RobotOD. */
  public Sub_RobotOD() {

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(RobotContainer.gyroscope.fetchHeading()));

    // Create the simulation model of our drivetrain.
    if (RobotBase.isSimulation() == true) {
      m_driveSim = new DifferentialDrivetrainSim(
        Constants.kDrivetrainPlant, Constants.motorConfig,
        Constants.gearingConfig, Constants.trackWidth, 
        Constants.wheelRadius, VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
    }
    
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (count < 1 && RobotContainer.encoders != null) {
      RobotContainer.encoders.reset();
      count++;
    }

    m_odometry.update(Rotation2d.fromDegrees(RobotContainer.gyroscope.fetchHeading()),
        RobotContainer.encoders.leftEnc.getDistance(), RobotContainer.encoders.rightEnc.getDistance());

    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("TrajectoryProgress@", i / 8);
  }

  @Override
  public void simulationPeriodic() {

    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(RobotContainer.driveTrain.mc_leftGroup.get() * RobotController.getInputVoltage(),
        RobotContainer.driveTrain.mc_rightGroup.get() * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.

    m_driveSim.update(0.02);

    // This will get the simulated sensor readings while in simulation,
    // but will use real values on the robot itself.

    RobotContainer.encoders.m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    RobotContainer.encoders.m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    RobotContainer.encoders.m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    RobotContainer.encoders.m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    //RobotContainer.gyroscope.gyroSimHardW.setAngle(m_driveSim.getHeading().getDegrees());
  }

  public void resetOdometry(Pose2d pose) {
    
    if(RobotBase.isSimulation()) {
      m_driveSim.setPose(pose);
    }

    RobotContainer.encoders.reset();
    m_odometry.resetPosition(pose, (Rotation2d.fromDegrees(RobotContainer.gyroscope.fetchHeading())));
  }

  public void iniTrajectories(String[] traJFileNames, int selFileExt) {
    traJState = false;
    for(i = 0; i < traJFileNames.length; i++) {
      autoTrajectory[i] = new Trajectory();

      try {
        traJLocation = Filesystem.getDeployDirectory().toPath().resolve(Constants.traJAutoSpd[selFileExt] + traJFileNames[i]);
        autoTrajectory[i] = TrajectoryUtil.fromPathweaverJson(traJLocation);
        traJState = true;
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + Constants.traJAutoSpd[selFileExt] + traJFileNames[i], ex.getStackTrace());
      }
    }
  }

  public void resetTraj(String[] traJCount) {
    for(int j = 0; j < traJCount.length; j++) {
      autoTrajectory[j] = null;
    }
    i = 0;
    traJState = false;
  }  

  public Pose2d getPosex() {
    return m_odometry.getPoseMeters();
  }

}
