// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_DriveTrain;

public class Auto extends CommandBase {
  /** Creates a new Auto. */
  Sub_DriveTrain driveTrain; 

  public Auto(Sub_DriveTrain driveTrain) {
    driveTrain = new Sub_DriveTrain();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double turn = 0;  
    double speed = -0.5;
    driveTrain.drive(speed, turn, true);
    Timer.delay(5);
    driveTrain.drive(0, 0, true);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
