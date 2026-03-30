//Function to calculate desired distance from goal based on shooter angle and velocity (some formula idek)

calculateDistance(velocity, angleDeg, shooterHeight, targetHeight) // Replace with auto shooter parameters

public static double calculateDistance(double velocity, double angleDeg, double shooterHeight, double targetHeight) {
        
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

//calculate distance from goal (probably could replace with robot absolute location)
public static float Estimate_Distance();
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches

//ADJUSTING the robot when a button is pressed

//Lower KpDistance makes the robots corrections faster
float KpDistance = -0.1f; 

//Run this code for every robot update
float desired_distance = calculateDistance(velocity, angleDeg, shooterHeight, targetHeight) // Replace with auto shooter parameters
float current_distance = Estimate_Distance();

float distance_error = desired_distance - current_distance;
driving_adjust = KpDistance * distance_error;
    
left_command += distance_adjust;
right_command += distance_adjust;
