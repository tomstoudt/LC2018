
package org.usfirst.frc.team272.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private String attackCode = "11";
	private String leftRight;
	private String frontRear;
	private boolean calibrateSwerveDrives = false;
//	private boolean calibrateShooterMotor = false;
	private boolean autonMinDisplay;
	private boolean climberMinDisplay;
	private boolean gearGrabberMinDisplay;
	private boolean inputsMinDisplay;
	private boolean robotMinDisplay;
	private boolean sensorsMinDisplay;
//	private boolean shooterMinDisplay;
	private boolean swerveDriveMinDisplay;
	private boolean gyroDrive = false;
	private int drivingMode;
	private int turn180Range;
		
	private int lfTurnForwardPosition;
	private int lrTurnForwardPosition;
	private int rfTurnForwardPosition;
	private int rrTurnForwardPosition;
	
	private double robotSpeedDivisor;
	private double robotAngleDivisor;
	private double robotRotationDivisor;
	
	private int robotSpeedExpo;
	private int robotAngleExpo;
	private int robotRotationExpo;
	
	private int calibrateWheelAngle0Button;
	private int calibrateWheelAngle90Button;
	private int calibrateWheelAngle180Button;
	private int climbActionFastButton;
	private int climbActionSlowButton;
//	private int shooterShootButton;
//	private int sweeperSweepButton;
	private int grabberDownAndOpenButton;
	private int grabGearButton;
	private int deployGearButton;
	private int grabberUpButton;
	
	private int camHorizRes;
	private int camVertRes;
	private int camFPS;
	private ControlVars controlVars;
	private LCTelemetry telem;
	private Auton auton;
	private Inputs inputs;
	private Config config;
	private Sensors sensors;
	private Climber climber;
//	private Shooter shooter;
//	private Sweeper sweeper;
	private SwerveDriveTrain swerveDriveTrain;
	private GearGrabber gearGrabber;
	private RobotMoves robotMoves;
	private GyroNavigate gyroNavigate;
	private UsbCamera usbCamera0;
//	private Relay targetingCameraPower;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		
		this.robotMinDisplay = true;
		controlVars = new ControlVars();
		controlVars.zeroVars();
		this.config = new Config("/c/FRC272Config.cfg");
		loadConfig(this.config);		
    	telem = new LCTelemetry();
    	telem.loadConfig(this.config);
    	
    	try {
			this.usbCamera0 = CameraServer.getInstance().startAutomaticCapture(0);
			this.usbCamera0.setResolution(this.camHorizRes, this.camVertRes);
			this.usbCamera0.setFPS(this.camFPS);
			this.usbCamera0.setExposureAuto();
		} catch (Exception e) {
			System.out.println("Camera not connected!");
		}    	
//    	this.targetingCameraPower = new Relay(0, Relay.Direction.kForward);
//    	this.targetingCameraPower.set(Relay.Value.kOff);
    	    
    	auton = new Auton();
    	
		inputs = new Inputs(config);
		
		sensors = new Sensors(config);
		
		swerveDriveTrain = new SwerveDriveTrain(config);
		
		climber = new Climber(config);
		
//		shooter = new Shooter(config);
		
//		sweeper = new Sweeper(config);
		
		gearGrabber = new GearGrabber(config);
		
		robotMoves = new RobotMoves();
		
		gyroNavigate = new GyroNavigate();
		
		auton.loadMoves();
		auton.addTelemetryHeaders(telem);
		sensors.addTelemetryHeaders(telem);
//		sensors.target.addTelemetryHeaders(telem);
		swerveDriveTrain.addTelemetryHeaders(telem);
		updatePreferences();
	}

    /**
     * This function is run once each time the robot enters disable mode
     */
    public void disabledInit() {
		updatePreferences();
  		telem.saveSpreadSheet();						// once done you cannot save more data there. 
    	telem.restartTimer();
    	config.load();
    	loadConfig(this.config);
//    	this.shooter.loadConfig(config);
    	this.swerveDriveTrain.loadConfig(this.config);
//    	this.sensors.target.loadConfig(this.config);
    	auton.loadMoves();
    }

    /**
     * This function is called periodically during disable
     */
    public void disabledPeriodic() {
    	
    	outputToDashboard(this.robotMinDisplay);
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomousInit() {
		updatePreferences();
    	telem.restartTimer();
    	auton.resetAuton();
//    	this.targetingCameraPower.set(Relay.Value.kOn);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		controlVars.zeroVars();
    	sensors.readValues();	// read the sensors
    	auton.dispatcher(controlVars, sensors, gyroNavigate, config, attackCode);
    	gearGrabber.GearBrain(controlVars);
//    	shooter.update(controlVars, sensors);
		swerveDriveTrain.update(controlVars, sensors, gyroNavigate);
		auton.writeTelemetryValues(this.telem);
		sensors.writeTelemetryValues(telem);
//		sensors.target.writeTelemetryValues(telem);
		swerveDriveTrain.writeTelemetryValues(this.telem);
    	telem.writeRow();
    	sensors.outputToDashboard(this.sensorsMinDisplay);
    	auton.outputToDashboard(this.autonMinDisplay);
		gearGrabber.outputToDashBoard(this.gearGrabberMinDisplay);
//		shooter.outputToDashboard(this.shooterMinDisplay);
		swerveDriveTrain.outputToDashboard(this.swerveDriveMinDisplay);
	}

	/**
	 * This function is run once each time the robot enters teleop mode
	 */
	public void teleopInit() {
		this.leftRight = "l";
		this.frontRear = "f";
    	telem.restartTimer();
//    	this.targetingCameraPower.set(Relay.Value.kOff);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		
		if (this.calibrateSwerveDrives) {
			calibrateSwerveDrives();
//		} else if (this.calibrateShooterMotor) {
//			calibrateShooter();
		} else {
			controlVars.zeroVars();
			inputs.readValues();
			sensors.readValues();
			mapInputsToSwerveDrives();
			controlVars.setGrabberUp(inputs.getButton(this.grabberUpButton));
			controlVars.setGrabberDownAndOpen(inputs.getButton(this.grabberDownAndOpenButton));
			controlVars.setGrabGear(inputs.getButton(this.grabGearButton));
			controlVars.setDeployGear(inputs.getButton(this.deployGearButton));
			controlVars.setClimbFast(inputs.getButton(this.climbActionFastButton));
			controlVars.setClimbSlow(inputs.getButton(this.climbActionSlowButton));			
//			controlVars.setShoot(inputs.getButton(this.shooterShootButton));
//			controlVars.setSweep(inputs.getButton(this.sweeperSweepButton));
						
			this.robotThink();
			
			swerveDriveTrain.update(controlVars, sensors, gyroNavigate);
			climber.update(controlVars);
//			shooter.update(controlVars, sensors);
//			sweeper.update(controlVars);			
			gearGrabber.update(controlVars);
			
			inputs.outputToDashboard(this.inputsMinDisplay);
			sensors.outputToDashboard(this.sensorsMinDisplay);
			climber.outputToDashBoard(this.climberMinDisplay);
			gearGrabber.outputToDashBoard(this.gearGrabberMinDisplay);
//			shooter.outputToDashboard(this.shooterMinDisplay);
			swerveDriveTrain.outputToDashboard(this.swerveDriveMinDisplay);
			//swerveDriveTrain.writeTelemetryValues(this.telem);
	    	//telem.writeRow();
		}
	}

	/**
	 * This function is run once each time the robot enters test mode
	 */
	public void testInit() {

	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		
	}

	public void robotThink() {
				
	}

	private void mapInputsToSwerveDrives() {
		
		int    pov;

		pov = inputs.getDriverPov();
		if (pov > 0) {
			if (pov > 180)
				pov = pov - 360;
			this.robotMoves.rotateByGyro(controlVars, sensors, pov);
		} else {
			if (this.drivingMode == 2)
				mapJoysticksToSwerveDrives2();
			else
				mapJoysticksToSwerveDrives1();
		}
	}
	
	// Swerve and throttle on right joystick, rotation only on left joystick, throttle on triggers
	private void mapJoysticksToSwerveDrives1() {
		
		double robotAngle;
		double robotSpeed;
		double robotRotation;
		double rightYAxisValue;
		double robotAngleVector;

		robotAngle = inputs.getRightXAxisValue();
		rightYAxisValue = inputs.getRightYAxisValue();
		robotAngleVector = Math.sqrt((rightYAxisValue * rightYAxisValue) + (robotAngle * robotAngle));
//		robotAngleVector -= (Math.cos((Math.abs(rightYAxisValue) * 90.0 * (Math.PI / 180.0))) * 0.5);
		robotAngleVector = Math.min(robotAngleVector, 1.0);
		if (rightYAxisValue < 0.0)
			robotAngleVector *= -1.0;
		if (robotAngleVector != 0.0)
			robotSpeed = robotAngleVector;
		else
			robotSpeed = -inputs.getzAxisValue();

		robotRotation = inputs.getxAxisValue();
		
		robotAngle *= this.robotAngleDivisor;
		robotAngle = expo(robotAngle, this.robotAngleExpo);
		controlVars.setRobotAngle(robotAngle);
		
		robotSpeed *= this.robotSpeedDivisor;
		robotSpeed = expo(robotSpeed, this.robotSpeedExpo);
		controlVars.setRobotSpeed(robotSpeed);
	
		robotRotation *= this.robotRotationDivisor;
		robotRotation = expo(robotRotation, this.robotRotationExpo);			
		controlVars.setRobotRotation(robotRotation);
		
		controlVars.setGyroDrive(this.gyroDrive);
	}
	
	// Rotation on right joystick, Swerve and throttle on left joystick, throttle on triggers
	private void mapJoysticksToSwerveDrives2() {
		
		double robotAngle;
		double robotSpeed;
		double robotRotation;
		double yAxisValue;
		double leftRobotSpeed = 0.0;
		double robotAngleVector;

		// Left Joystick
		robotAngle = inputs.getxAxisValue();
		yAxisValue = inputs.getyAxisValue();
		robotAngleVector = Math.sqrt((yAxisValue * yAxisValue) + (robotAngle * robotAngle));
//		robotAngleVector -= (Math.cos((Math.abs(yAxisValue) * 90.0 * (Math.PI / 180.0))) * 0.5);
		robotAngleVector = Math.min(robotAngleVector, 1.0);
		if (yAxisValue < 0.0)
			robotAngleVector *= -1.0;
		leftRobotSpeed = robotAngleVector;
	
		// Right Joystick
		robotRotation = inputs.getRightXAxisValue();

		if (leftRobotSpeed != 0.0)
			robotSpeed = leftRobotSpeed;
		else
			robotSpeed = -inputs.getzAxisValue();
		
		robotAngle *= this.robotAngleDivisor;
		robotAngle = expo(robotAngle, this.robotAngleExpo);
		controlVars.setRobotAngle(robotAngle);
		
		robotSpeed *= this.robotSpeedDivisor;
		robotSpeed = expo(robotSpeed, this.robotSpeedExpo);
		controlVars.setRobotSpeed(robotSpeed);
	
		robotRotation *= this.robotRotationDivisor;
		robotRotation = expo(robotRotation, this.robotRotationExpo);			
		controlVars.setRobotRotation(robotRotation);
		
		controlVars.setGyroDrive(this.gyroDrive);
	}
	
	private void calibrateSwerveDrives() {
		
		String loc;
		int pov;
		int turnForwardPosition;
		Double talonPosition;
		double speed;
		
		inputs.readValues();
		pov = inputs.getDriverPov();
		if (pov == 315 || pov == 0 || pov == 45)
			this.frontRear = "f";
		else if (pov >= 135 && pov <= 225)
			this.frontRear = "r";			
		if (pov >= 45 && pov <= 135) 
			this.leftRight = "r";
		else if (pov >= 225 && pov <= 315)
			this.leftRight = "l";
		
		loc = this.leftRight + this.frontRear;
		
		if (loc.equalsIgnoreCase("lf"))
			turnForwardPosition = this.lfTurnForwardPosition;
		else if (loc.equalsIgnoreCase("lr"))
			turnForwardPosition = this.lrTurnForwardPosition;
		else if (loc.equalsIgnoreCase("rf"))
			turnForwardPosition = this.rfTurnForwardPosition;
		else if (loc.equalsIgnoreCase("rr"))
			turnForwardPosition = this.rrTurnForwardPosition;
		else
			turnForwardPosition = 440;
			
		SmartDashboard.putNumber("pov", pov);
		SmartDashboard.putString("loc", loc);
		
		if (inputs.getButton(this.calibrateWheelAngle0Button)) {
			swerveDriveTrain.calibrateDriveWheelAngle(loc, turnForwardPosition - this.turn180Range / 2);
		} else if (inputs.getButton(this.calibrateWheelAngle90Button)) {
			swerveDriveTrain.calibrateDriveWheelAngle(loc, turnForwardPosition);
		} else if (inputs.getButton(this.calibrateWheelAngle180Button)) {
			swerveDriveTrain.calibrateDriveWheelAngle(loc, turnForwardPosition + this.turn180Range / 2);
		} else {
			talonPosition = inputs.getxAxisValue();
			//convert wheelAngle to number between 0 and 1
			talonPosition = (((talonPosition - 1.0)) * -0.5);
			talonPosition *= this.turn180Range;
			talonPosition += (turnForwardPosition - this.turn180Range / 2);			
			swerveDriveTrain.calibrateDriveWheelAngle(loc, talonPosition.intValue());
		}
		
		speed = inputs.getyAxisValue();
		if (speed > 0.1 || speed < -0.1)
			swerveDriveTrain.calibrateDriveWheelSpeed(loc, speed);
		else
			swerveDriveTrain.calibrateDriveWheelSpeed(loc, 0.0);			
	}
	
//	private void calibrateShooter () {
//		
//		if (this.inputs.getButton(this.calibrateWheelAngle180Button)) {
//			this.shooter.setShooterMotorRPM(this.shooter.getShooterMotorRPM() - 50);
//			this.shooter.setShooterMotorVoltage(this.shooter.getShooterMotorVoltage() - .05);
//		} else if (this.inputs.getButton(this.calibrateWheelAngle0Button)) {
//			this.shooter.setShooterMotorRPM(this.shooter.getShooterMotorRPM() + 50);
//			this.shooter.setShooterMotorVoltage(this.shooter.getShooterMotorVoltage() + .05);
//		}
//		this.controlVars.setShoot(this.inputs.getButton(this.shooterShootButton));
//		this.shooter.update(this.controlVars, this.sensors);
//	}
	
	private void loadConfig(Config config) {
		
		this.gyroDrive 			  = config.getBoolean("gyroDrive", false);
		this.drivingMode          = config.getInt("drivingMode", 2);
		this.robotAngleDivisor    = config.getDouble("robotAngleDivisor", 1.0);
		this.robotSpeedDivisor    = config.getDouble("robotSpeedDivisor", 1.0);
		this.robotRotationDivisor = config.getDouble("robotRotationDivisor", 1.0);
		this.robotAngleExpo       = config.getInt("robotAngleExpo", 1);
		this.robotSpeedExpo       = config.getInt("robotSpeedExpo", 1);
		this.robotRotationExpo    = config.getInt("robotRotationExpo", 1);
		
		this.turn180Range = config.getInt("turn180Range", 440);
		this.lfTurnForwardPosition = config.getInt("lfTurnForwardPosition", 440);
		this.lrTurnForwardPosition = config.getInt("lrTurnForwardPosition", 440);
		this.rfTurnForwardPosition = config.getInt("rfTurnForwardPosition", 440);
		this.rrTurnForwardPosition = config.getInt("rrTurnForwardPosition", 440);
		
		this.calibrateWheelAngle0Button   = config.getInt("calibrateWheelAngle0Button", 4);
		this.calibrateWheelAngle90Button  = config.getInt("calibrateWheelAngle90Button", 6);
		this.calibrateWheelAngle180Button = config.getInt("calibrateWheelAngle180Button", 3);
		
		this.climbActionFastButton = config.getInt("climbActionFastButton", 105);
		this.climbActionSlowButton = config.getInt("climbActionSlowButton", 106);
		
//		this.shooterShootButton = config.getInt("shooterShootButton", 4);
//		this.sweeperSweepButton = config.getInt("sweeperSweepButton", 2);
		
		this.grabberDownAndOpenButton = config.getInt("grabberDownAndOpenButton", 101);
		this.grabGearButton = config.getInt("grabGearButton", 103);
		this.deployGearButton = config.getInt("deployGearButton", 102);
		this.grabberUpButton = config.getInt("grabberUpButton", 104);
		this.camHorizRes = config.getInt("camHorizRes", 320);
		this.camVertRes = config.getInt("camVertRes", 240);
		this.camFPS = config.getInt("camFPS", 15);

		this.calibrateSwerveDrives = config.getBoolean("calibrateSwerveDrives", false);
//		this.calibrateShooterMotor = config.getBoolean("calibrateShooterMotor", false);
		this.autonMinDisplay = config.getBoolean("autonMinDisplay", true);
		this.climberMinDisplay = config.getBoolean("climberMinDisplay", true);
		this.gearGrabberMinDisplay = config.getBoolean("gearGrabberMinDisplay", true);
		this.inputsMinDisplay = config.getBoolean("inputsMinDisplay", true);
		this.robotMinDisplay = config.getBoolean("robotMinDisplay", true);
		this.sensorsMinDisplay = config.getBoolean("sensorsMinDisplay", true);
//		this.shooterMinDisplay = config.getBoolean("shooterMinDisplay", true);
		this.swerveDriveMinDisplay = config.getBoolean("swerveDriveMinDisplay", true);

	}
	
	private double expo(double value, double exponent) {
		
		double temp;
		
		temp = value;
		for (int i = 1; i < exponent; i++)
			temp *= temp;
		if (value < 0.0 && temp > 0.0)  // preserve sign when multiplying by itself even number of times
			return temp * -1.0;
		else
			return temp;
	}

	public void updatePreferences()  {

		this.attackCode = Preferences.getInstance().getString("AttackCode", "22");
	}
		
    public void outputToDashboard(boolean minDisplay){
		SmartDashboard.putBoolean("CalibrateSwerveDrives", this.calibrateSwerveDrives);
//		SmartDashboard.putBoolean("CalibrateShooterMotor", this.calibrateShooterMotor);
		SmartDashboard.putString("AttackCode", this.attackCode);
    }
}
