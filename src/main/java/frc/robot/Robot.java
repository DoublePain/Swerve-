//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.GD;
import frc.robot.lib.RobotMode;
import frc.robot.lib.k;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public Robot(){
    super(k.ROBOT.PERIOD);
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();


  }
  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    GD.G_RobotMode = RobotMode.DISABLE_INIT;
    RobotContainer.m_LEDs.setAllianceColor();
  }

  @Override
  public void disabledPeriodic() {
    GD.G_RobotMode = RobotMode.DISABLE_PERIODIC;
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        GD.G_Alliance = Alliance.Red;
        GD.G_AllianceSign = -1.0;
      }else {
        GD.G_Alliance = Alliance.Blue;
        GD.G_AllianceSign = 1.0;
      }
    }
    //RobotContainer.m_LEDs.setAllianceColor();
    RobotContainer.m_LEDs.meteor();
    //RobotContainer.m_LEDs.fade();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        GD.G_Alliance = Alliance.Red;
        GD.G_AllianceSign = -1.0;
      }
    }
    RobotContainer.m_drivetrainSubsystem.resetYaw();
    GD.G_RobotMode = RobotMode.AUTONOMOUS_INIT;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //RobotContainer.m_LEDs.setRGBColor(50, 50, 0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {GD.G_RobotMode = RobotMode.AUTONOMOUS_PERIODIC;}

  @Override
  public void teleopInit() {
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        GD.G_Alliance = Alliance.Red;
        GD.G_AllianceSign = -1.0;
      }
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    GD.G_RobotMode = RobotMode.TELEOP_INIT;
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    GD.G_RobotMode = RobotMode.TELEOP_PERIODIC;
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    GD.G_RobotMode = RobotMode.TEST_INIT;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {GD.G_RobotMode = RobotMode.TEST_PERIODIC;}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() { GD.G_RobotMode = RobotMode.SIMULATION_INIT;}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {GD.G_RobotMode = RobotMode.SIMULATION_PERIODOC;}
}
