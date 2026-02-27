package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // 1. Define Motors (CANSparkMax is now SparkMax)
    private final SparkMax pivotMotor = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax rollerMotor = new SparkMax(7, MotorType.kBrushless);

    // 2. Define Controller and Encoder
    private final SparkClosedLoopController pivotPID;
    private final RelativeEncoder pivotEncoder;

    public IntakeSubsystem() {
        // Create configuration objects
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        // Configure Pivot
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        
        pivotConfig.closedLoop
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-0.5, 0.5);

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
    }

    public void runRollers(double speed) {
        rollerMotor.set(speed);
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }

    public void setPivotPosition(double position) {
        pivotPID.setSetpoint(position, SparkBase.ControlType.kPosition);
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }
}
