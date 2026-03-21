package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // 1. Define Motors (CANSparkMax is now SparkMax)
    private final SparkMax rollerMotor = new SparkMax(53, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(54, MotorType.kBrushless);
    
    private final double GEAR_RATIO = 36.0;

    public IntakeSubsystem() {
        // Create configuration objects
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        // Set conversion for 36:1 (360 / 36 = 10 degrees per motor rotation)
        //pivotConfig.encoder
        //    .positionConversionFactor(360.0 / GEAR_RATIO) 
        //    .velocityConversionFactor(360.0 / GEAR_RATIO / 60.0);

        // Configure Pivot
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);
        
        // Configure Rollers
        rollerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30);


        pivotMotor.configure(pivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        rollerMotor.configure(rollerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
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

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    //  public void setPivotPosition(double positionDegrees) {
    //    The '0' at the end explicitly targets PID Slot 0
    //    pivotPID.setSetpoint(positionDegrees, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //}
}
