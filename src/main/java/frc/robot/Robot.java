/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//RECYCLOPS

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Sub_DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public double autoTimer;
  double turn = 0;
  double speed = -0.5; 
  static Sub_DriveTrain driveTrain = RobotContainer.driveTrain;

  public static ArcadeDrive defaultDrive = new ArcadeDrive();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will instantiate all subsytems,
    // and perform all our button bindings
    m_robotContainer = new RobotContainer();

    RobotContainer.powerDist.clearStickyFaults();

    RobotContainer.ultrasonics.setAuto();
    RobotContainer.encoders.setDistanceScale(RobotContainer.encoders.distancePerPulse);
    RobotContainer.colourSensor.setColorValues(Constants.RD, Constants.GN, Constants.BE, Constants.YW);
  }

  @Override
  public void simulationPeriodic() {

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    RobotContainer.powerDist.clearStickyFaults();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    Constants.autoSpeed = RobotContainer.choose_autoRouteSpd.getSelected();

    if (Constants.autoSpeed != Constants.teleOperation && !RobotContainer.odometry.traJState) {
      RobotContainer.odometry.iniTrajectories(Constants.traJAutoPaths, Constants.autoSpeed);
    } else if (Constants.autoSpeed == Constants.teleOperation && RobotContainer.odometry.traJState) {
      RobotContainer.odometry.resetTraj(Constants.traJAutoPaths);
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autoTimer = Timer.getFPGATimestamp(); // get the current time in ms to see if autonomous needs to be stopped.
    /*
     * m_autonomousCommand = m_robotContainer.getAutonomousCommand();
     * 
     * 
     * Constants.autoRoute = RobotContainer.choose_autoRouteDR.getSelected();
     * RobotContainer.encoders.setDistanceScale(RobotContainer.encoders.
     * distancePerPulse);
     * 
     * // schedule the autonomous command (example) if (m_autonomousCommand != null
     * && RobotContainer.odometry.traJState) { m_autonomousCommand.schedule(); }
     * else { DriverStation.
     * reportWarning("Please Select an Autonomous Trajectory and Speed in SmartDashBoard"
     * , false); }
     */
    // above is from old code ^^
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (Timer.getFPGATimestamp() < autoTimer + 5) {

      driveTrain.drive(speed, turn, true);

    } else {
      driveTrain.drive(0, 0, true);
    }

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.encoders.setDistanceScale(RobotContainer.encoders.distancePerPulse);
    RobotContainer.odometry.resetOdometry(new Pose2d(0.76, 2.29, new Rotation2d(0)));
    RobotContainer.autoEnd = false;
    RobotContainer.isTeleop = true;
    enableManualControl();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static void enableManualControl() {
    // Schedules the commands used in teleop
    if (RobotContainer.isTeleop) {
      RobotContainer.isTeleop = false;

      defaultDrive.schedule();

      System.out.println("MANUAL CONTROL");
    }
  }




}