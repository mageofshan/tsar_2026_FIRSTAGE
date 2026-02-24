//Shooter subsytem pseudocode

//1. Define each motor
public WPI_Kraken flywheelMotor1;
public WPI_Kraken flywheelMotor2;
public WPI_Kraken sushiMotor1;
public WPI_Kraken sushiMotor2;

//2. Feed-Forward: Making sure that the flywheel's velocity remains constant

//Put this in a constructor
public boolean shooterReady;

public void feedForward() {
    //Get the average speed of motors in rps, assuming one is inverted
    double shooterSpeed = Math.abs((flywheelMotor1.getVelocity().getValueAsDouble() - flywheelMotor2.getVelocity().getValueAsDouble())/2.0);
    double flywheelRPS = shooterSpeed; //Would need to do some conversions here to calculate flywheel
    double targetRPS = 10.0; //Change to whatever
    //change constants for static, velocity, and acceleration
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.637, 0.14002, 0.0092594);
    double feedVoltage = feedforward.calculate(targetRPS);
    flywheelMotor1.setVoltage(feedVoltage);
    flywheelMotor2.setVoltage(-feedVoltage); //use negative sign on whichever motor is inverted
    if (Math.abs(targetRPS - shooterSpeed) < 1) { //Check if shooter is within +- 1 RPS of target
        shooterReady = true;
    }
    else {
        shooterReady = false;
    }
}

//3. Define shot function

public void shot() {
    //Setting sushi roll motors to 10 rps, adjust if needed
    sushiMotor1.setControl(new VelocityVoltage(10));
    sushiMotor1.setControl(new VelocityVoltage(10));
}

public void stop_shot() {
    sushiMotor1.setControl(new VelocityVoltage(0));
    sushiMotor1.setControl(new VelocityVoltage(0));
}

//4. Configure an Xbox button to the shooter (using Y for now)
private final CommandXboxController joystick = new CommandXboxController(0); //Copied from robotcontainer, idk if we need it again
//while loop might not be the right way of doing this
while (true) {
    if (shooterReady) {
        joystick.y().onTrue(new shot());
    }
    joystick.y().onFalse(new stop_shot());
}