// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        //LIMELIGHT MEASUREMENTS
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 19.0; 

        // distance from the target to the floor
        double goalHeightInches = 44.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //Lower KpDistance makes the robots corrections faster
        float KpDistance = -0.1f; 

        private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    }

    public static double calculateDistance(double velocity, double angleDeg, double shooterHeight, double targetHeight) {
        //Velocity is in m/s

        final double g = 9.81; // gravity (m/s^2)

        double angleRad = Math.toRadians(angleDeg);
        double deltaH = targetHeight - shooterHeight;

        double vSin = velocity * Math.sin(angleRad);
        double vCos = velocity * Math.cos(angleRad);

        double discriminant = (vSin * vSin) - (2 * g * deltaH);

        // If discriminant is negative, no real solution (shot impossible)
        if (discriminant < 0) {
            return -1;
        }

        double distance = (vCos / g) * (vSin + Math.sqrt(discriminant));

        return distance;
    }

    //calculate distance from goal
    public static float Estimate_Distance();
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        //Align swerve module if x button is pressed, checking periodically
        if (joystick.getRawButton(kButtonX)) {
            float shooterHeight = 0.559 //Height in meters
            float targetHeight = 1.26 //Height of hub + hopper in meters
            float velocity = m_shooter.getFlywheelRPM()
            float angleDeg = 85
            float desired_distance = calculateDistance(velocity, angleDeg, shooterHeight, targetHeight)
            float current_distance = Estimate_Distance();

            float distance_error = desired_distance - current_distance;
            float driving_adjust = KpDistance * distance_error;
                
            left_command += driving_adjust;
            right_command += driving_adjust;
        }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}