package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(5);
    private final DutyCycleOut m_output = new DutyCycleOut(0);

    public ClimberSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        //hardware limit switch = safer
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        
        //brake = no slip
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        climberMotor.getConfigurator().apply(config);
    }

    public void setPower(double speed) {
        climberMotor.setControl(m_output.withOutput(speed));
    }

    public void stop() {
        climberMotor.stopMotor();
    }
}