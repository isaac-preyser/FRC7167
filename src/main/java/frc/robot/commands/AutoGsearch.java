// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoGsearch extends CommandBase {
  /** Creates a new AutoGsearch. */
  public AutoGsearch() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  RamseteCommand ramseteCommand;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Trajectory autoGTraj = new Trajectory();
    RobotContainer.isTeleop = true;
    
    // Get our selected trajectory
    autoGTraj = RobotContainer.odometry.autoTrajectory[Constants.autoRoute];
 
    // Reset odometry to the starting pose of the trajectory.
    RobotContainer.odometry.resetOdometry(autoGTraj.getInitialPose());

    // Set up autonomousChooser and set these with appropriate paths
    ramseteCommand = new RamseteCommand(
        autoGTraj, 
        RobotContainer.odometry::getPosex,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kVelocityLinear, Constants.kAccelerationLinear),
        Constants.kDriveKinematics, 
        RobotContainer.encoders::fetchWheelSpds,
        new PIDController(Constants.kPDriveVel, 0, 0), 
        new PIDController(Constants.kPDriveVel, 0, 0),
        RobotContainer.driveTrain::autoVoltDR, 
        RobotContainer.driveTrain);

    // Run path following command, then stop at the end.
    ramseteCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ramseteCommand != null && ramseteCommand.isFinished()) {
      RobotContainer.setAutoEnd(true);
    } else {
      RobotContainer.setAutoEnd(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.autoVoltDR(0, 0);
    System.out.println("Finished AutoRoute");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.autoEnd != true) {
      return false;
    } else {
      return true;
    }
  }
}
