package frc.robot.lib.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.GD;
import frc.robot.lib.DriveSpeedState;
import frc.robot.lib.k;


public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonSRX m_steerMotor;
    private CANcoder m_cancoder;
    private String m_name;
    private double m_driveSetVelocity_mps = 0;
    private double m_steerSetAngle_deg = 0;
    private double m_driveActualVelocity_mps = 0;
    private double m_steerActualAngle_deg = 0;
    private double m_steerVolts = 0.0;
    private double m_driveVolts = 0.0;
    private PIDController m_steerPID = new PIDController(k.STEER.PID_Kp, k.STEER.PID_Ki, 0);
    private PIDController m_drivePID = new PIDController(k.DRIVE.PID_Kp, k.DRIVE.PID_Ki, 0.0);
    private SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(k.DRIVE.PID_Ks, k.DRIVE.PID_Kv);
    private SwerveModulePosition m_internalState = new SwerveModulePosition();


    public SwerveModule(SwerveModuleConstants _constants) {
        m_driveMotor = new TalonFX(_constants.m_driveMotorId , k.ROBORIO_CAN_IDS.NAME); 
        m_steerMotor = new TalonSRX(_constants.m_steerMotorId);
        m_cancoder = new CANcoder(_constants.m_CANcoderId);

        m_name = _constants.m_name;
        // Configure Drive Motor
        m_driveMotor.setInverted(_constants.m_isDriveMotorReversed);
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

 

        // Configure Steer Motor
        m_steerPID.enableContinuousInput(-180.0, 180.0);
        m_steerMotor.setInverted(_constants.m_isSteerMotorReversed ? InvertType.FollowMaster : InvertType.None);
        m_steerMotor.setNeutralMode(NeutralMode.Brake);
        m_steerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        m_steerMotor.setSensorPhase(false);
        m_steerMotor.configNominalOutputForward(0);
        m_steerMotor.configNominalOutputReverse(0);
        m_steerMotor.configPeakOutputForward(.1);
        m_steerMotor.configPeakOutputReverse(-.1);
        m_steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5);
        m_steerMotor.setSelectedSensorPosition((int) (m_cancoder.getPosition().getValue().doubleValue() * k.STEER.GEAR_RATIO));
    }

    public void setDesiredState(SwerveModuleState _state, boolean _enableSteer, boolean _enableDrive) {
        SwerveModuleState optimized = SwerveModuleState.optimize(_state, m_internalState.angle);

        if (_enableSteer) {
            m_steerSetAngle_deg = optimized.angle.getDegrees();
            m_steerActualAngle_deg = m_steerMotor.getSelectedSensorPosition() / k.STEER.GEAR_RATIO;
            m_steerVolts = m_steerPID.calculate(m_steerActualAngle_deg, m_steerSetAngle_deg);
            m_steerMotor.set(ControlMode.PercentOutput, m_steerVolts);
        }
        if (_enableDrive) {
            m_driveSetVelocity_mps = optimized.speedMetersPerSecond;
            m_driveActualVelocity_mps = m_driveMotor.getPosition().getValue() / k.DRIVE.WHEEL_MotRotPerMeter;
            m_driveSetVelocity_mps = GD.G_DriveSpeedState == DriveSpeedState.LOW ? m_driveSetVelocity_mps * 0.5 : m_driveSetVelocity_mps;
            m_driveVolts = m_drivePID.calculate(m_driveActualVelocity_mps, m_driveSetVelocity_mps);
            m_driveVolts += m_driveFF.calculate(m_driveSetVelocity_mps);
            m_driveMotor.set( m_driveVolts);
        }
    }

    public void stopMotors() {
        m_steerMotor.set(ControlMode.PercentOutput, 0);
        m_driveMotor.set(0);
    }

    public SwerveModulePosition getPosition(boolean _refresh) {
        StatusSignal<Double> positionSignal = m_driveMotor.getPosition();
        double drive_rot = positionSignal.getValue();
        double angle_rot = m_steerMotor.getSelectedSensorPosition();
        angle_rot /= k.STEER.GEAR_RATIO;
        m_internalState.distanceMeters = drive_rot / k.DRIVE.WHEEL_MotRotPerMeter;
        m_internalState.angle = Rotation2d.fromDegrees(angle_rot);
        return m_internalState;
    }

    public void updateDashboard() {
        SmartDashboard.putNumber(m_name + "_set_mps", m_driveSetVelocity_mps);
        SmartDashboard.putNumber(m_name + "_act_deg", m_steerActualAngle_deg);
        SmartDashboard.putNumber(m_name + "_act_mps", m_driveActualVelocity_mps);
        SmartDashboard.putNumber(m_name + "_driveVolts", m_driveVolts);
    }
}