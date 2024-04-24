//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive.DrivetrainDefaultCommand;
import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.LEDs;
import frc.robot.lib.k;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * In command-based projects, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Subsystems, commands, and trigger mappings should be defined here.
 * 
 */
public class RobotContainer {
  public static ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  public static PowerDistribution m_pd = new PowerDistribution();
  public static Set<ISubsystem> subsystems = new HashSet<>();




  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(m_drivetrainSubsystem);





  private Notifier m_telemetry;
  //public static final OrangePi5Vision m_vision = new OrangePi5Vision();
  
  public static final CommandPS5Controller s_driverController = new CommandPS5Controller(k.OI.DRIVER_CONTROLLER_PORT);
  public static final CommandPS5Controller s_operatorController = new CommandPS5Controller(k.OI.OPERATOR_CONTROLLER_PORT);

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static final LEDs m_LEDs = new LEDs(2);
  private void updateDashboard(){
    SmartDashboard.putString("RobotMode", GD.G_RobotMode.toString());
    Iterator<ISubsystem> it = subsystems.iterator();
    while(it.hasNext()){
      it.next().updateDashboard(); // Comment this line out if you want ALL smartdashboard data to be stopped.
    }
  }
  /** This is the constructor for the class. */
  public RobotContainer() {
   
    m_drivetrainSubsystem.setDefaultCommand(m_drivetrainDefaultCommand);

    // Configure the trigger bindings
    configureBindings();
    

    // Add all autonomous command groups to the list on the Smartdashboard

    SmartDashboard.putData("Autonomous Play",autoChooser);

    // Setup the dashboard notifier that runs at a slower rate than our main robot periodic.
    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);
    configShuffelBoard();
  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   */
  private void configureBindings() {

    k.OI.DRIVER_RESET_YAW.onTrue(new InstantCommand(m_drivetrainSubsystem::resetYaw, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_ANGLEFIELDCENTRIC.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveMode_AngleFieldCentric, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_FIELDCENTRIC.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveMode_FieldCentric, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_ROBOTCENTRIC.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveMode_RobotCentric, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_ROTATEFIELDCENTRIC.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveMode_RotateFieldCentric, m_drivetrainSubsystem));

    k.OI.DRIVER_DRIVE_MODE_SPEED_HI.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveSpeedHI, m_drivetrainSubsystem));
    k.OI.DRIVER_DRIVE_MODE_SPEED_LOW.onTrue(new InstantCommand(m_drivetrainSubsystem::setDriveSpeedLOW, m_drivetrainSubsystem));
    
  }
  /**
   * @return the command to run in autonomous routine
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  private void configShuffelBoard(){
    
    Shuffleboard.selectTab("Match");
    matchTab.add("Autonomous Play",autoChooser).withPosition(5,0).withSize(3, 2);
    matchTab.add("Note State", GD.G_NoteState.toString()).withPosition(8, 0).withSize(2,2);
    matchTab.add("Shooter State", GD.G_ShooterState.toString()).withPosition(10, 0).withSize(2,2);
    matchTab.add("Flipper State", GD.G_ShooterState.toString()).withPosition(12, 0).withSize(2,2);
    matchTab.add("Drive State", m_drivetrainSubsystem.getDriveMode().toString()).withPosition(14, 0).withSize(3,2);
    matchTab.add("Battery Volts", m_pd.getVoltage()).withPosition(5, 2).withSize(12,3).withWidget(BuiltInWidgets.kGraph);
   
   // matchTab.add("Shooter Volts", m_shooterSubsystem.getShooterSpeed()).withPosition(7, 2).withSize(2,2).withWidget(BuiltInWidgets.kDial);
  }
}
