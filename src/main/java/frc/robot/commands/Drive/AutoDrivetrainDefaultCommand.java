//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Drive;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;


public class AutoDrivetrainDefaultCommand extends Command {

  boolean blue1;
  boolean blue2;
  boolean blue3;
  boolean red1;
  boolean red2;
  boolean red3;
  private DrivetrainSubsystem m_drive;

  /** Creates a new DrivetrainDefaultCommand. */
  public AutoDrivetrainDefaultCommand(DrivetrainSubsystem _drivetrain) {
    m_drive = _drivetrain;
    this.setName("AutoDriveDefaultCommand");
    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveStopMotion();
  }

  // /**
  //  * If right stick X,Y Hyp is > 0.8 then set the target angle
  //  * Reset the Shooter angle and speed to default if button not pressed.
  //  * 
  //  * Set the following data if the appropriate buttons are pressed
  //  * Robot Angle to turn to
  //  * Shooter angle to move to
  //  * Shooter speed to run at.
  //  */
  // public void setShotData(Shots shot) {
  //   // set the target angle only if the x,y hyp are > deadband
  //   // Assume blue
  //   double allianceSign = 1.0;
  //   allianceSign = GD.G_Alliance == Alliance.Red ? -allianceSign : allianceSign;

  //   switch (shot) {
  //     case Blue1:
  //       blue1 = true;
  //       break;
  //     case Blue2:
  //       blue2 = true;
  //       break;
  //     case Blue3:
  //       blue3 = true;
  //       break;
  //     case Red1:
  //       red1 = true;
  //       break;
  //     case Red2:
  //       red2 = true;
  //       break;
  //     case Red3:
  //       red3 = true;
  //       break;
  //     default:
  //       break;
  //   }

  //   // if x,y hyp > deadband reset the shooter angle and speed
  //   if (GD.G_RobotTargetAngle.getHyp() > k.DRIVE.TARGET_ANGLE_DEADBAND) {
  //     GD.G_ShooterAngle = k.SHOOTER.ROTATE_OFFSET_ANGLE_DEG;
  //     GD.G_ShooterSpeed = 0.0;
  //   }
  //   // Handle target angle buttons if pressed to set angles and speeds
  //   if (blue1) {
  //     GD.G_RobotTargetAngle.setTargetAngle(50);
  //     GD.G_ShooterAngle = 50;
  //     GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
  //   } else if (blue2) {
  //     GD.G_RobotTargetAngle.setTargetAngle(50);
  //     GD.G_ShooterAngle = 50;
  //     GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
  //   } else if (blue3) {
  //     GD.G_RobotTargetAngle.setTargetAngle(-30);
  //     GD.G_ShooterAngle = 41;
  //     GD.G_ShooterSpeed = 0.65;
  //   } else if (red1) {
  //     GD.G_RobotTargetAngle.setTargetAngle(-50);
  //     GD.G_ShooterAngle = 50;
  //     GD.G_ShooterSpeed = k.SHOOTER.SPIN_SHOT_SPEED_RPS;
  //   } else if (red2) {
  //     GD.G_RobotTargetAngle.setTargetAngle(-27 * allianceSign);
  //     GD.G_ShooterAngle = 51;
  //     GD.G_ShooterSpeed = 0.68;
  //   } else if (red3) {
  //     GD.G_RobotTargetAngle.setTargetAngle(45);
  //     GD.G_ShooterAngle = 65;
  //     GD.G_ShooterSpeed = 0.6;
  //   }
  //   blue1 = false;
  //   blue2 = false;
  //   blue3 = false;
  //   red1 = false;
  //   red2 = false;
  //   red3 = false;
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void updateDashboard() {

  }
}
