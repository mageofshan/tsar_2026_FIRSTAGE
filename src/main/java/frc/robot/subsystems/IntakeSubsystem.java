package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // Pivot (SparkMax)
    private final SparkMax pivotMotor = new SparkMax(54, MotorType.kBrushless);
    private final SparkClosedLoopController pivotPID;
    private final RelativeEncoder pivotEncoder;

    // Roller motors — KrakenX60 (TalonFX)
    private final TalonFX rollerLeader   = new TalonFX(61);
    private final TalonFX rollerFollower = new TalonFX(62);

    // Pivot gear ratio — degrees output per motor rotation
    private final double GEAR_RATIO = 24.0;
    private final double DEGREES_PER_ROTATION = 360.0 / GEAR_RATIO; // 15.0 deg/rot

    // PID gains — tune these on the robot
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;

    // Output clamp — limits max pivot speed during PID control
    private static final double kMaxOutput =  0.5;
    private static final double kMinOutput = -0.5;

    private final VoltageOut m_voltageSetter = new VoltageOut(0);

    // Soft limits in degrees — uncomment and set once range of motion is known
    // private static final float SOFT_LIMIT_FWD_DEG = 90.0f;
    // private static final float SOFT_LIMIT_REV_DEG =  0.0f;

    public IntakeSubsystem() {
        
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        // Encoder conversion: motor rotations → degrees
        pivotConfig.encoder
            .positionConversionFactor(DEGREES_PER_ROTATION) // rotations  → degrees
            .velocityConversionFactor(DEGREES_PER_ROTATION / 60.0); // RPM → deg/s

        // PID configuration
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(kMinOutput, kMaxOutput);

        // Soft limits — uncomment once range of motion is confirmed
        // pivotConfig.softLimit
        //     .forwardSoftLimit(SOFT_LIMIT_FWD_DEG)
        //     .forwardSoftLimitEnabled(true)
        //     .reverseSoftLimit(SOFT_LIMIT_REV_DEG)
        //     .reverseSoftLimitEnabled(true);

        pivotMotor.configure(
            pivotConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        pivotPID = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();

        pivotEncoder.setPosition(0.0);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = 40.0;

        rollerLeader.getConfigurator().apply(rollerConfig);
        rollerFollower.getConfigurator().apply(rollerConfig);

        rollerFollower.setControl(new Follower(rollerLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Commands the pivot to a target angle in degrees.
     * 0° = home position at boot. Tune kP/kI/kD before using in match.
     */
    public void setPivotAngle(double targetDegrees) {
        pivotPID.setReference(targetDegrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    public void runPivotVoltage(double percent) {
        pivotMotor.set(percent);
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    /**
     * Re-zeros the encoder. Call this when the arm is confirmed at its
     * home/reference position (e.g., triggered by a limit switch).
     */
    public void zeroPivotEncoder() {
        pivotEncoder.setPosition(0.0);
    }

    public void runRollers(double percent) {
        rollerLeader.setControl(m_voltageSetter.withOutput(percent * 12.0));
        SmartDashboard.putNumber("rollerRPM",
            rollerLeader.getVelocity().getValueAsDouble() * 30.0);
    }

    public void stopRollers() {
        rollerLeader.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Angle (deg)", getPivotAngle());
        SmartDashboard.putNumber("Pivot Output",      pivotMotor.getAppliedOutput());
    }
}