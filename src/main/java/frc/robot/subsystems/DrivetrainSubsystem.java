//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.DriveSpeedState;
import frc.robot.lib.EDriveMode;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;
import frc.robot.lib.Swerve.SwerveDrive;

public class DrivetrainSubsystem extends SubsystemBase implements ISubsystem{
  public SwerveDrive m_robotDrive;
  public EDriveMode m_driveMode = EDriveMode.ANGLE_FIELD_CENTRIC;
  
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
     m_robotDrive = new SwerveDrive();
     
    initialize(); 
  }
  public void initialize(){
    RobotContainer.subsystems.add(this);
  }

  public void driveStopMotion(){
    m_robotDrive.driveStopMotion();
  }
  public void driveRobotCentric(ChassisSpeeds _speeds){
    m_robotDrive.driveRobotCentric(_speeds);
  }

  public void driveFieldCentric(ChassisSpeeds _speeds){
    m_robotDrive.driveFieldCentric(_speeds);
  }

  public void driveAngleFieldCentric(double _x, double _y, Rotation2d _targetAngle, boolean _enableSteer, boolean _enableDrive){
    m_robotDrive.driveAngleFieldCentric(_x, _y, _targetAngle, _enableSteer, _enableDrive);
  }

  public void drivePolarFieldCentric(double _driveAngle_deg, double _robotAngle_deg, double _speed, boolean _enableSteer, boolean _enableDrive){
    double y = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    double x = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    driveAngleFieldCentric(x, y, new Rotation2d(Math.toRadians(_robotAngle_deg)), _enableSteer, _enableDrive);
  }
  public void setDriveMode_FieldCentric(){
    m_driveMode = EDriveMode.FIELD_CENTRIC;
  }
  public void setDriveMode_AngleFieldCentric(){
    m_driveMode = EDriveMode.ANGLE_FIELD_CENTRIC;
  }
  public void setDriveMode_RotateFieldCentric(){
    m_driveMode = EDriveMode.ROTATE_FIELD_CENTRIC;
  }
  public void setDriveMode_RobotCentric(){
    m_driveMode = EDriveMode.ROBOT_CENTRIC;
  }
  public void setDriveSpeedHI(){
    GD.G_DriveSpeedState = DriveSpeedState.HIGH;
  }
  public void setDriveSpeedLOW(){
    GD.G_DriveSpeedState = DriveSpeedState.LOW;
  }
  public EDriveMode getDriveMode(){
    return m_driveMode;
  }

  public double getRobotAngle(){
    return m_robotDrive.getRobotYaw();
  }

  public void resetYaw(){
    m_robotDrive.resetYaw();
  }
public double getAllianceSign(){
  return GD.G_AllianceSign;
}
public void setAllianceSign(){
  if(DriverStation.getAlliance().isPresent()){
    if(DriverStation.getAlliance().get() == Alliance.Red){
      GD.G_Alliance = Alliance.Red;
      GD.G_AllianceSign = -1.0;
    }
  }
}
  public void setLastTargetAngle(Rotation2d _targetAngle){
    GD.G_RobotTargetAngle.setTargetAngle(_targetAngle.getDegrees());
  }
  public void resetPose(){
    m_robotDrive.resetPose();
  }
  public void setPose(Pose2d _pose){
    GD.G_RobotPose = _pose;
  }
  // public double getAprilArea(){
  // --    return RobotContainer.m_vision.getAprilArea();
  // }
  // public double getAprilYaw(){
  // --    return RobotContainer.m_vision.getAprilYaw();
  // }
  // public double getNoteYaw(){
  // --    return RobotContainer.m_vision.getNoteYaw();
  // }
  public void updateDashboard(){
    SmartDashboard.putString(k.DRIVE.T_DRIVER_MODE, m_driveMode.toString());
    SmartDashboard.putString("Robot Target Angle", GD.G_RobotTargetAngle.getTargetAngle().toString());
    SmartDashboard.putNumber("Robot Angle", getRobotAngle());
    // -- SmartDashboard.putNumber("NoteYaw", RobotContainer.m_vision.getNoteYaw());
   // --  SmartDashboard.putNumber("AprilYaw", RobotContainer.m_vision.getAprilYaw());
    m_robotDrive.updateDashboard();
  }
  @Override
  public void periodic() {


  }
}
