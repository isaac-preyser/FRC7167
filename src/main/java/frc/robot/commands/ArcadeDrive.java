/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArcadeDrive extends CommandBase {
  /**
   * Creates a new ArcadeDrive.
   */

  public ArcadeDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.stick.getRawAxis(Constants.joy_left_y);
    double turn = RobotContainer.stick.getRawAxis(Constants.joy_right_x);
    
    speed = -speed*RobotContainer.driveTrain.speeds[RobotContainer.driveTrain.speedMode][0];
    turn = turn*RobotContainer.driveTrain.speeds[RobotContainer.driveTrain.speedMode][1];
    
    RobotContainer.driveTrain.drive(speed, turn, RobotContainer.driveTrain.driveMode);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.drive(0, 0, true);
    if(interrupted) System.out.println("AUTO CONTROL");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
