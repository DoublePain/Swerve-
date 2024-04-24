// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveDelayCommand extends Command {
  DrivetrainSubsystem m_drive;
  Timer m_timer;
  double m_time;
  boolean m_isFinished = false;
  /** Creates a new AutoDelayCommand. */
  public AutoDriveDelayCommand(DrivetrainSubsystem _drive, double _time) {
    m_drive = _drive;
    m_time = _time;
    addRequirements(_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveStopMotion();
    if(m_timer.hasElapsed(m_time)){
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
