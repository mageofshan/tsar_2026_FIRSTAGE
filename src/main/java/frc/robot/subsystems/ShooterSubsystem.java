package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX flywheelMotor1 = new TalonFX(1);
    private final TalonFX flywheelMotor2 = new TalonFX(2);
    private final TalonFX feederMotor1 = new TalonFX(3);
    private final TalonFX feederMotor2 = new TalonFX(4);

    private final VelocityVoltage m_velocitySetter = new VelocityVoltage(0);

    public ShooterSubsystem() {
        TalonFXConfiguration flyConfig = new TalonFXConfiguration();
        
        flyConfig.Slot0.kS = 0.637; 
        flyConfig.Slot0.kV = 0.14002;
        flyConfig.Slot0.kA = 0.0092594;
        flyConfig.Slot0.kP = 0.11;

        flyConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        flywheelMotor1.getConfigurator().apply(flyConfig);
        
        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
        
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederMotor1.getConfigurator().apply(feederConfig);
        feederMotor2.setControl(new Follower(feederMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    // Modern way to set speed: Target is in Rotations Per Second (RPS)
    public void runFlywheel(double rps) {
        flywheelMotor1.setControl(m_velocitySetter.withVelocity(rps));
    }

    public void runSushi(double rps) {
        feederMotor1.setControl(m_velocitySetter.withVelocity(rps));
    }

    public void stopAll() {
        flywheelMotor1.stopMotor();
        feederMotor1.stopMotor();
    }

    public boolean isReady(double targetRPS) {
        double currentRPS = flywheelMotor1.getVelocity().getValueAsDouble();
        return Math.abs(targetRPS - currentRPS) < 1.0;
    }
}