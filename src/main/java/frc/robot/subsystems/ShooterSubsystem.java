package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
public class ShooterSubsystem extends SubsystemBase {
    // Flywheel motors (Master and Follower)
    private final TalonFX flywheelMotor1 = new TalonFX(40);
    private final TalonFX flywheelMotor2 = new TalonFX(41);
    
    // Single feeder motor
    private final TalonFX feederMotor1 = new TalonFX(42);

    private final VelocityVoltage m_velocitySetter = new VelocityVoltage(0);

    private final DutyCycleOut m_feederSetter = new DutyCycleOut(0);


    public ShooterSubsystem() {
        // --- Flywheel Configuration ---
        TalonFXConfiguration flyConfig = new TalonFXConfiguration();
        
        //flywheeel PIDF (to be tuned)
        flyConfig.Slot0.kS = 0.637; 
        flyConfig.Slot0.kV = 0.14002;
        flyConfig.Slot0.kA = 0.0092594;
        flyConfig.Slot0.kP = 0.11;

        // natural spin down
        flyConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flyConfig.CurrentLimits.StatorCurrentLimit = 60; // 60 Amps is a safe limit for flywheels
        flyConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Re-apply the config to the motor
        flywheelMotor1.getConfigurator().apply(flyConfig);
        
        // flywheelMotor2 mirrors flywheelMotor1
        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
        
        // --- Feeder Configuration ---
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        // You can add current limits or neutral modes to feederConfig here if needed
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederMotor1.getConfigurator().apply(feederConfig);
    }

    /**
     * Runs the flywheel at a specific velocity.
     * @param rps Rotations Per Second
     */
    public void runFlywheel(double rps) {
        flywheelMotor1.setControl(m_velocitySetter.withVelocity(rps));
    }

    //Update the method to use percent output
    public void runSushiPercent(double percent) {
    // percent is a value from -1.0 to 1.0
    feederMotor1.setControl(m_feederSetter.withOutput(percent));
    }

    /**
     * Runs the feeder (sushi) motor.
     * @param rps Rotations Per Second
     */
    public void runSushi(double rps) {
        feederMotor1.setControl(m_velocitySetter.withVelocity(rps));
    }

    /**
     * Stops both the flywheel and the feeder motor.
     */
    public void stopAll() {
        flywheelMotor1.stopMotor();
        feederMotor1.stopMotor();
    }

    /**
     * Checks if the flywheel is within 1 RPS of the target.
     */
    public boolean isReady(double targetRPS)
{
// Check if we are at least at 95% of the target speed
return flywheelMotor1.getVelocity().getValueAsDouble() >= (targetRPS * 0.95);
    }
}