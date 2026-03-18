package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // 1. Define Motors (CANSparkMax is now SparkMax)
    private final SparkMax rollerMotor = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(7, MotorType.kBrushless);

    // 2. Define Controller and Encoder
    private final SparkClosedLoopController pivotPID;
    private final RelativeEncoder pivotEncoder;

    // Constants for your 36:1 setup
    private final double GEAR_RATIO = 36.0;
    private final double SPIKE_THRESHOLD = 35.0; // Amps

    // State tracking for spike logic
    private double targetRotation = 0;
    private boolean isAtTarget = false;

    public IntakeSubsystem() {
        // Create configuration objects
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        // Set conversion for 36:1 (360 / 36 = 10 degrees per motor rotation)
        pivotConfig.encoder
            .positionConversionFactor(360.0 / GEAR_RATIO) 
            .velocityConversionFactor(360.0 / GEAR_RATIO / 60.0);

        // Add P gain so the motor actually moves to the position
        pivotConfig.closedLoop
            .p(0.1) 
            .outputRange(-0.5, 0.5);

        // Configure Pivot
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        
        // Configure Rollers
        rollerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);

        // Apply configurations
        // kResetSafeParameters = restoreFactoryDefaults
        // kPersistParameters = burnFlash
        pivotMotor.configure(pivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        rollerMotor.configure(rollerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        pivotPID = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();

        // Ensure we start at 0
        pivotEncoder.setPosition(0.0);
        pivotConfig.closedLoop.p(0.1);
    }

    @Override
    public void periodic() {
        double currentPos = pivotEncoder.getPosition();

        // 1. Check if we have reached the target (within 2 degrees)
        if (Math.abs(currentPos - targetRotation) < 2.0) {
            isAtTarget = true;
        }

        // 2. ONLY check for spikes if we have reached the target
        if (isAtTarget && pivotMotor.getOutputCurrent() > SPIKE_THRESHOLD) {
            pivotMotor.stopMotor();
        }
        SmartDashboard.putNumber("Intake/Degrees", pivotEncoder.getPosition());
    }

    public void runPivotVoltage(double percent){
        pivotMotor.set(percent);
    }


    public void runRollers(double speed) {
        rollerMotor.set(speed);
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }

    public void setPivotPosition(double positionDegrees) {
        // The '0' at the end explicitly targets PID Slot 0
        pivotPID.setSetpoint(positionDegrees, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);

    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }
}
