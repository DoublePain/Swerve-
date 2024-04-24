//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveTimeVelToAprilArea extends Command {
  DrivetrainSubsystem m_drivetrain;
  Timer m_timer = new Timer();
  double m_rampTime = 1.0; // Seconds
  double m_timeOut_sec;
  double m_driveAngle;
  double m_robotAngle;
  double m_speed;
  double m_rampUpTime_sec;
  double m_rampDownTime_sec;
  double m_currentSpeed = 0;
  double m_yaw = 0;
  double m_area = 0;
  double m_areaError = 0;
  double m_yawError = 0;
  double m_driveAngleAdjusted = m_driveAngle;
  /**
   * * AutoDriveTimeVel
   * <p>
   * Drive at a set velocity, robot angle and time. A velocity ramp time of 1
   * second will be used if rampEnable is true.
   * 
   * @param _drive       An instance of the drive subsystem
   * @param _velocity    The velocity you want the robot to drive at in meters/sec
   * @param _driveAngle  The Drive angle the chassis should drive at in degrees
   * @param _robotAngle  The Angle the robot should face while driving in degrees
   * @param _timeOut_sec The time to stop driving in seconds.
   * @param _rampEnable  Enable the ramp of velocity at the start.
   */
  public AutoDriveTimeVelToAprilArea(DrivetrainSubsystem _drive, double _speed_mps, double _driveAngle, double _robotAngle) {
    m_drivetrain = _drive;
    m_driveAngle = _driveAngle;
    m_robotAngle = _robotAngle;
    m_speed = _speed_mps;
    m_rampUpTime_sec = 0.1;
    m_rampDownTime_sec = 0.1;
    m_currentSpeed = m_speed;
    m_timeOut_sec = 2;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // double currentTime_sec = m_timer.get();
    // if (currentTime_sec < m_timeOut_sec && currentTime_sec > m_timeOut_sec - m_rampDownTime_sec) { // In the ramp down time
    //   m_currentSpeed = m_speed * (m_timeOut_sec - currentTime_sec) / m_rampDownTime_sec;
    // } else if (currentTime_sec < m_rampUpTime_sec) {// In the ramp up time
    //   m_currentSpeed = m_speed * currentTime_sec / m_rampUpTime_sec;
    // } else { // past the ramp up time and not in ramp down time
    //   m_currentSpeed = m_speed;
    // }
    
    
    // if(m_drivetrain.getAprilArea() < 5){
    //   m_areaError = (k.DRIVE.APRIL_AREA_SHOT - m_drivetrain.getAprilArea()) * 3;
    // }
    // m_drivetrain.drivePolarFieldCentric(m_driveAngle, m_robotAngle, m_areaError,true,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_timeOut_sec)) {
      m_drivetrain.driveStopMotion();
      return true;
    }
    return false;
  }

}
