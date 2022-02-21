/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DegreesRotate extends CommandBase {
  /**
   * Creates a new DegreesRotate.
   */
  private double tarDeg;
  private double turnSpd;
  private double curDeg;
  private double errSpd;
  private double minSpd = 1.5;  // Volts

  public DegreesRotate(double speed, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);

    if(degrees > 180) {
      tarDeg = degrees - 360;
    } else {
      tarDeg = degrees;
    }
    turnSpd = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.isTeleop = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curDeg = -RobotContainer.gyroscope.fetchDirec();
    
    RobotContainer.driveTrain.autoVoltDR(-getSpinSpeed(), getSpinSpeed());

    System.out.println("ROTATING degrees: " + (int)curDeg + "/" + (int)tarDeg + ", " + getSpinSpeed());
    
  }

  public double getSpinSpeed() {
    if(Math.signum(tarDeg) != Math.signum(curDeg)) {
      errSpd = (tarDeg + curDeg)/36;  // Output Voltage
    } else {
      errSpd = Math.signum(tarDeg - curDeg) * (Math.abs(tarDeg) - Math.abs(curDeg))/36;
    }
    errSpd *= turnSpd;

    return (double) Math.max(Math.abs(errSpd), minSpd) * Math.signum(errSpd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.drive(0, 0, true);
    Robot.enableManualControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((int)curDeg/3 == tarDeg/3){
      return true;
    } else if(tarDeg == 180 && (int) Math.abs(curDeg) == 179) {
      return true;
    } else {
      return false;
    }
  }
}