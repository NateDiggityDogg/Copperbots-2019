/**
 /**

]\[
]\[
]\[
 * CopperBots FRC 2018 Robot Code
 * 
 */

//
package org.usfirst.frc.team2586.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;

public class Robot extends TimedRobot {

	/*
	 * HARDWARE DECLARATIONS
	 */

	// Controllers
	XboxController operatorController, johnController;
	Joystick leftStick, rightStick;

	// Speed Controllers
	PWMVictorSPX intakeLeft, intakeRight;
	WPI_TalonSRX frontLeft, frontRight, rearLeft, rearRight, lift;

	// Gyro
	ADIS16448_IMU gyro;

	// Encoders
	Encoder leftEnc;
	Encoder rightEnc;
	Encoder liftEnc;

	// Pneumatics
	Compressor comp;
	DoubleSolenoid shifter;
	DoubleSolenoid clamp;
	DoubleSolenoid intakeDeploy;

	// Switches
	DigitalInput liftLow, liftHigh;

	// Smart Dash Choosers
	SendableChooser<String> autoChooser = new SendableChooser<>();
	SendableChooser<Integer>delayChooser = new SendableChooser<>();
	SendableChooser<Boolean>johnModeChooser = new SendableChooser<>();
	final String autoChooserNone = "None";
	final String autoChooserLine = "line";
	final String autoChooserSwitchCenter = "Center Switch";
	final String autoChooserSwitchCenterAngled = "Angled switch auton";
	final String autoChooserSwitchLeft = "Left Switch";
	final String autoChooserSwitchRight = "Right Switch";
	final String autoChooserScaleLeft = "Left Scale";
	final String autoChooserScaleCenter = "Center Scale";
	final String autoChooserScaleRight = "Right Scale";
	final String autoChooserScalePrefLeft = "PREF Left Scale";
	final String autoChooserScalePrefRight = "PREF Right Scale";

	/*
	 * VARIABLE DECLARATIONS
	 */

	// Naming mainDrive
	DifferentialDrive mainDrive;

	// John's tank drive controls for XBOX
	boolean isJohnModeEnabled = false;
	// Disable soft-limits and let the operator do as they please
	boolean limitBreak = false;

	// variable used for initial calibration of teleop gyro
	boolean isTrackingStraight = false;
	boolean overrideControllers = false;
	boolean gyroMode = true;

	// booleans for redundancy controls
	boolean liftOverride = false;

	/*
	 * AUTON STATE VARS
	 */

	// The game data received from the game
	String gameData;

	// Current step in the Auton Program
	int autoStep = 0;

	// Auton State Timer
	Timer autoTimer;
	Timer secondaryTimer;

	// autoDrive state vars
	double dPrev = 0.0;
	double dHeading = 0.0;
	double loopTime = 0.02;
	double error = 0;
	double previousError = 0;
	double sumError = 0;
	double dError = 0;

	double steady;

	// auto delay
	double auto_delay;

	/**
	 * -------------------------------------------------------------------------------------------------------------------------------
	 * ROBOT INITIALIZATION
	 */
	@Override
	public void robotInit() {

		// Camera
		CameraServer.getInstance().startAutomaticCapture();

		// Pneumatics
		comp = new Compressor();
		comp.start();

		shifter = new DoubleSolenoid(0, 1);
		clamp = new DoubleSolenoid(4, 5);
		intakeDeploy = new DoubleSolenoid(6, 7);

		// Controllers
		operatorController = new XboxController(0);
		johnController = new XboxController(4);
		leftStick = new Joystick(3);
		rightStick = new Joystick(2);

		// Encoders
		leftEnc = new Encoder(0, 1);
		rightEnc = new Encoder(2, 3);

		double kPulsesPerRevolution = 1440;
		// theoretical value double kInchesPerRevolution = 18.8496;
		double kInchesPerRevolution = 26;
		double kInchesPerPulse = kInchesPerRevolution / kPulsesPerRevolution;
		leftEnc.setDistancePerPulse(kInchesPerPulse); // [Inches/Pulses]
		rightEnc.setDistancePerPulse(kInchesPerPulse); // [Inches/Pulses]
		// since we do not know the if the decoding is by 1x, 2x, or 4x, I have inputted
		// the x1 value
		// the value for x2 is 720 pulses per revolution and the value for x4 is 1440
		// pulses per revolution
		// 18.85 inches is the value of one rotation, 1x is 0.052 inches per pulse, 2x
		// is 0.026 inches per pulse, and 4x is 0.013 inches per pulse

		// Gyro
		gyro = new ADIS16448_IMU();
		gyro.calibrate();
		gyro.reset();

		// Drivebase
		frontRight = new WPI_TalonSRX(1);
		frontLeft = new WPI_TalonSRX(3);
		rearLeft = new WPI_TalonSRX(4);
		rearRight = new WPI_TalonSRX(2);

		// declaring slave/master for back wheels
		rearLeft.set(ControlMode.Follower, 3);
		rearRight.set(ControlMode.Follower, 1);

		// Lift
		lift = new WPI_TalonSRX(5);
		/*
		 * liftLow = new DigitalInput(7); liftMid = new DigitalInput(8); liftHigh = new
		 * DigitalInput(9);
		 */
		liftLow = new DigitalInput(5);
		liftHigh = new DigitalInput(4);

		// Intake;
		intakeLeft = new PWMVictorSPX(0);
		intakeRight = new PWMVictorSPX(1);

		// declaring the drive system
		mainDrive = new DifferentialDrive(frontLeft, frontRight);
		mainDrive.setSafetyEnabled(false);

		// Auto Program Chooser [Smart Dash]
		autoChooser.addDefault(autoChooserLine, autoChooserLine);
		autoChooser.addObject(autoChooserNone, autoChooserNone);
		autoChooser.addObject(autoChooserSwitchCenter, autoChooserSwitchCenter);
		autoChooser.addObject(autoChooserSwitchCenterAngled, autoChooserSwitchCenterAngled);
		autoChooser.addObject(autoChooserSwitchLeft, autoChooserSwitchLeft);
		autoChooser.addObject(autoChooserSwitchRight, autoChooserSwitchRight);
		autoChooser.addObject(autoChooserScaleLeft, autoChooserScaleLeft);
		autoChooser.addObject(autoChooserScaleCenter, autoChooserScaleCenter);
		autoChooser.addObject(autoChooserScaleRight, autoChooserScaleRight);
		autoChooser.addObject(autoChooserScalePrefLeft, autoChooserScalePrefLeft);
		autoChooser.addObject(autoChooserScalePrefRight, autoChooserScalePrefRight);
		SmartDashboard.putData("Auto Selection", autoChooser);

		// Sendable chooser for autonomous delay [Smart Dash]
		 delayChooser.addDefault("0", 0);
		 delayChooser.addObject("3", 3);
		 delayChooser.addObject("5", 5);
		 SmartDashboard.putData("auto delay", delayChooser);
		 auto_delay = delayChooser.getSelected();
		
		 //Sendable chooser for "John Mode" [Smart Dash]
		 johnModeChooser.addDefault("false", false);
		 johnModeChooser.addObject("true", true);
		 SmartDashboard.putData("John Mode", johnModeChooser);

		// Auton State Vars
		autoTimer = new Timer();
		secondaryTimer = new Timer();
	}

	@Override
	public void robotPeriodic() {
		smartDash();
	}

	/**
	 * -------------------------------------------------------------------------------------------------------------------------
	 * TELEOP CONTROL
	 */
	@Override
	public void teleopInit() {
		// Deploy intake
		intakeDeploy.set(DoubleSolenoid.Value.kForward);
	}

	/**
	 * This is the teleop phase, essentially an infinite "while" loop thats run
	 * during teleop phase, it calls on the two controller classes
	 */

	@Override
	public void teleopPeriodic() {
		if (isJohnModeEnabled) {
			johnMode();
		} else {
			regularDrive();
		}

		// Lift Control
		double liftCommand = deadZoneComp(operatorController.getRawAxis(1) * -1);
		// Override all soft limits
		if (operatorController.getBackButton())
			limitBreak = true;

		if (operatorController.getStartButton())
			limitBreak = false;

		// Send speed command to the lift
		if (limitBreak) {
			lift.set(liftCommand);
		} else {
			if (!liftOverride) {
				liftControl(liftCommand);
			}
		}

		// Intake Control
		// Each trigger should allow you to either intake or eject
		double intakeCommand = operatorController.getTriggerAxis(GenericHID.Hand.kLeft)
				- operatorController.getTriggerAxis(GenericHID.Hand.kRight);
		intakeLeft.set(intakeCommand);
		intakeRight.set(intakeCommand);

		// Claw Control
		if (operatorController.getBButton()) {
			clamp.set(DoubleSolenoid.Value.kReverse); // Open
		}
		if (operatorController.getAButton()) {
			clamp.set(DoubleSolenoid.Value.kForward); // Close
		}

		if (operatorController.getYButton()) {
			clamp.set(DoubleSolenoid.Value.kReverse);
		}

		// Intake Deploy / Retract
		if (operatorController.getPOV() == 270) {
			intakeDeploy.set(DoubleSolenoid.Value.kForward); // Deploy
		}
		if (operatorController.getPOV() == 90) {
			intakeDeploy.set(DoubleSolenoid.Value.kReverse); // Retract
		}

	}

	// limit switch protection used by both teleop and auton
	private void liftControl(double input) {

		// Limit switches to prevent limits
		// True when NOT pressed!!!!
		if (input > 0.0 && liftHigh.get())
			lift.set(input);

		else if (input < 0.0 && liftLow.get())
			lift.set(input);

		else
			lift.set(0.0);
	}

	/**
	 * ------------------------------------------------------------------------------------------------------------------------------
	 * BEGINNING OF AUTONOMOUS PHASE
	 */
	@Override
	public void autonomousInit() {
		// Reset Gyro heading
		gyro.reset();
		dHeading = 0.0;

		shifter.set(DoubleSolenoid.Value.kReverse);

		// Restart the auto sequence
		autoStep = 0;
		autoNextStep();

	}

	// AUTONOMOUS PERIOD
	@Override
	public void autonomousPeriodic() {

		// Run the selected Auton Program
		switch (autoChooser.getSelected()) {
		case autoChooserLine:
			autoProgLine();
			break;

		case autoChooserSwitchCenter:
			autoProgSwitchCenter();
			break;

		case autoChooserSwitchCenterAngled:
			autoProgSwitchCenterAngled();
			break;

		case autoChooserSwitchLeft:
			autoProgSwitchLeft();
			break;

		case autoChooserSwitchRight:
			autoProgSwitchRight();
			break;

		case autoChooserScaleLeft:
			autoProgScaleLeft();
			break;

		case autoChooserScaleCenter:
			autoProgScaleCenter();
			break;

		case autoChooserScaleRight:
			autoProgScaleRight();
			break;
			
		case autoChooserScalePrefLeft:
			autoProgScalePrefLeft();
			break;

		case autoChooserScalePrefRight:
			autoProgScalePrefRight();
			break;
		}

	}

	/*
	 * Cross the Line
	 * 
	 * The only reason to run this is if the encoders or gyro are known to be not
	 * working...
	 */
	private void autoProgLine() {

		// Switch case for AUTONOMOUS SWITCH
		switch (autoStep) {

		case 1:
			if (autoTimer.get() > auto_delay)
				autoNextStep();
			break;

		case 2:
			// Drive forward

			// Drive for a bit
			if (autoDrive(140))
				autoNextStep();
			break;

		case 3:
			// Stop!
			freeze();

			// Hammer time... :)
			break;
		}
	}

	/*
	 * Center Switch
	 * 
	 * Start in center of wall, adjacent up with Exchange Zone.
	 * 
	 * Robot will drive forward slightly Turn 90 Deg toward the correct switch
	 * (based FMS data) Drive forward to the switch platform turn 90 to face switch
	 * Drive forward to switch platform Dump our cube
	 */
	private void autoProgSwitchCenter() {

		// Pick a direction based on FMS data
		double rot = 90;
		if (gameData.startsWith("L"))
			rot = -rot;

		// Switch case for AUTONOMOUS SWITCH
		switch (autoStep) {

		case 1:
			if (autoTimer.get() > auto_delay)
				autoNextStep();
			break;

		case 2:
			if (autoDrive(40.0))
				autoNextStep();
			break;

		case 3:
			if (autoTurn(rot))
				autoNextStep();
			break;

		case 4:
			if (autoDrive(48.0)) {
				autoNextStep();
			}
			break;

		case 5:
			liftControl(0.6);
			if (autoTimer.get() > 2.0) {
				lift.set(0);
				autoNextStep();
			}
			break;

		case 6:
			if (autoTurn(0.0))
				autoNextStep();
			break;

		case 7:
			if (autoDrive(70.0) || secondaryTimer.get() > 4.0) {
				autoNextStep();
			}
			break;

		case 8:
			intakeDeploy.set(DoubleSolenoid.Value.kReverse);
			autoNextStep();
			break;

		case 9:
			// Drop it like it's hot...
			clamp.set(DoubleSolenoid.Value.kReverse); // Open
			if (autoDump(0.3)) {
				autoNextStep();
			}
			// Stop!
			mainDrive.stopMotor();

			break;
		}

	}

	private void autoProgSwitchCenterAngled() {
		double rot = 50.6;
		if (gameData.startsWith("L")) {
			rot = -rot;
		}
		switch (autoStep) {
		case 1:
			if (autoDrive(24))
				autoNextStep();
			break;

		case 2:
			if (autoTurn(rot))
				autoNextStep();
			break;

		case 3:
			if (autoDrive(73.76))
				autoNextStep();
			break;

		case 4:
			liftControl(0.6);
			if (autoTimer.get() > 2.0) {
				lift.set(0);
				autoNextStep();
			}
			break;

		case 5:
			if (autoTurn(0))
				autoNextStep();
			break;

		case 6:
			if (autoDrive(24))
				autoNextStep();
			break;

		case 7:
			intakeDeploy.set(DoubleSolenoid.Value.kReverse);
			autoNextStep();
			break;

		case 8:
			// Drop it like it's hot...
			clamp.set(DoubleSolenoid.Value.kReverse); // Open
			if (autoDump(0.3)) {
				autoNextStep();
			}
			// Stop!
			mainDrive.stopMotor();

			break;

		}
	}

	private void autoProgSwitchLeft() {
		double rot = 90;
		if (gameData.startsWith("L")) {
			switch (autoStep) {
			case 1:
				if (autoDrive(140))
					autoNextStep();
				break;

			case 2:
				liftControl(0.6);
				if (autoTimer.get() > 1.75) {
					liftControl(0.0);
					autoNextStep();
				}
				break;

			case 3:
				if (autoTurn(rot))
					autoNextStep();
				break;

			case 4:
				if (autoDrive(24) || autoTimer.get() > 2)
					autoNextStep();
				break;

			case 5:
				if (autoDump(-0.3)) {
					autoNextStep();
				}
				clamp.set(DoubleSolenoid.Value.kReverse);
			}
		} else {
			switch (autoStep) {
			case 1:
				if (autoDrive(120))
					autoNextStep();
				break;

			}
		}
	}

	private void autoProgSwitchRight() {
		double rot = -90;
		if (gameData.startsWith("R")) {
			switch (autoStep) {
			case 1:
				if (autoDrive(140))
					autoNextStep();
				break;

			case 2:
				liftControl(0.6);
				if (autoTimer.get() > 1.75) {
					liftControl(0.0);
					autoNextStep();
				}
				break;

			case 3:
				if (autoTurn(rot))
					autoNextStep();
				break;

			case 4:
				if (autoDrive(24))
					autoNextStep();
				break;

			case 5:
				if (autoDump(0.3)) {
					autoNextStep();
				}
				clamp.set(DoubleSolenoid.Value.kReverse);
			}
		} else {
			switch (autoStep) {
			case 1:
				if (autoDrive(140))
					autoNextStep();
				break;

			}
		}
	}

	private void autoProgScaleLeft() {
		if (gameData.startsWith("L")) {
			switch (autoStep) {
			case 1:
				if (autoDrive(264))
					autoNextStep();
				break;

			case 2:

				break;

			case 3:
				if (autoTurn(45))
					autoNextStep();
				break;

			case 4:
				if (autoDump(0.3))
					autoNextStep();
				break;

			}
		} else {
			switch (autoStep) {
			case 1:
				if (autoDrive(264))
					autoNextStep();
			}
		}
	}

	private void autoProgScaleCenter() {

	}

	private void autoProgScaleRight() {
		if (gameData.startsWith("R")) {
			switch (autoStep) {
			case 1:
				if (autoDrive(264))
					autoNextStep();
				break;

			case 2:

				break;

			case 3:
				if (autoTurn(-45))
					autoNextStep();
				break;

			case 4:
				if (autoDump(0.3))
					autoNextStep();
				break;

			}
		} else {
			switch (autoStep) {
			case 1:
				if (autoDrive(264))
					autoNextStep();
			}
		}

	}

	private void autoProgScalePrefLeft() {
		if (gameData.charAt(1) == 'L') {
			autoProgScaleLeft();
		} else if (gameData.startsWith("L")) {
			autoProgSwitchLeft();
		} else {
			autoProgLine();
		}
		
	}
	
	private void autoProgScalePrefRight() {
		if (gameData.charAt(1) == 'R') {
			autoProgScaleRight();
		} else if (gameData.startsWith("R")) {
			autoProgSwitchRight();
		} else {
			autoProgLine();
		}
	}

	/*
	 * Auto Drive
	 * 
	 * Call from auto state machine, when it is finished it will return True so that
	 * you can go to the next step.
	 * 
	 * This function will use the encoders and gyro to drive along a straight line
	 * to a set distance or rotate on the spot to a set heading.
	 * 
	 * Must reset encoders and autoTimer between steps.
	 */
	private boolean autoDrive(double distance) {

		// Max drive speed
		// TODO: Increase / remove after validation.
		double maxSpeed = 0.9;

		//
		// Linear
		//

		// Get Encoder values [Inches]
		double l = leftEnc.getDistance();
		double r = rightEnc.getDistance();

		// If an encoder fails, we assume that it stops generating pulses
		// so use the larger of the two (absolute distance)
		double d;
		if (Math.abs(l) > Math.abs(r)) {
			d = l;
		} else {
			d = r;
		}

		error = distance - d;
		// Linear-Proportional control
		// double kP = maxSpeed / 36.0; // Start to slow down at 36 inches from target
		double kP = .0311;
		// Linear-Integral control
		double kI = 0;
		if (error < 24) {
			sumError = sumError += error / loopTime;
		} else {
			sumError = 0;
		}

		// Linear-Derivative control
		double kD = 0.004;
		dError = (error - previousError) / loopTime;

		double lin = (kP * error) + (kI * sumError) + (kD * dError);

		previousError = error;

		// Ramp up to speed to reduce wheel slippage
		double max_ramp_up = 0.075;
		if (lin > dPrev + max_ramp_up)
			lin = dPrev + max_ramp_up;
		dPrev = lin;

		// Limit max speed
		lin = absMax(lin, maxSpeed);

		//
		// Rotation
		//

		double kP_rot = maxSpeed / 45.0; // start slowing down at 45 deg.
		double e_rot = dHeading - gyro.getAngleZ();
		double rot = e_rot * kP_rot;

		// Max rotation speed
		rot = absMax(rot, maxSpeed);

		// Nothing left but to do it...
		mainDrive.arcadeDrive(lin, rot);

		// Determine if the robot made it to the target
		// and then wait a bit so that it can correct any overshoot.
		if (error > 2 || e_rot > 3.0)
			autoTimer.reset();
		else if (autoTimer.get() > 0.75)
			return true;

		// Keep trying...
		return false;
	}

	/*
	 * Auto Turn
	 * 
	 * Call from auto state machine, when it is finished it will return True so that
	 * you can go to the next step.
	 * 
	 * This function will use the gyro to rotate to a set heading
	 * 
	 * Must reset encoders and autoTimer between steps.
	 */
	private boolean autoTurn(double heading) {

		// Max drive speed
		// TODO: Increase / remove after validation.
		double maxSpeed = 0.7;

		// Update the target heading for autoDrive function
		dHeading = heading;

		//
		// Rotation
		//
		double kP_rot = maxSpeed / 45.0; // start slowing down at 45 deg.
		double e_rot = heading - gyro.getAngleZ();
		double rot = e_rot * kP_rot;

		// Max rotation speed
		rot = absMax(rot, maxSpeed);

		// Nothing left but to do it...
		mainDrive.arcadeDrive(0.0, rot);

		// Determine if the robot made it to the target
		// and then wait a bit so that it can correct any overshoot.
		if (Math.abs(e_rot) > 3.0)
			autoTimer.reset();

		else if (autoTimer.get() > 0.75)
			return true;

		// Keep trying...
		return false;
	}

	private boolean autoDump(double x) {
		intakeLeft.set(x);
		intakeRight.set(x);
		if (autoTimer.get() > 1) {
			freeze();
			return true;
		}
		return false;
	}

	private void freeze() {
		mainDrive.stopMotor();
		lift.stopMotor();
		intakeLeft.stopMotor();
		intakeRight.stopMotor();
	}

	// Limits the output of a PWM signal
	double absMax(double input, double maxValue) {

		// Just in case the max is negative
		maxValue = Math.abs(maxValue);

		if (input > 0)
			return Math.min(input, maxValue);
		else
			return Math.max(input, -maxValue);
	}

	double absMin(double input, double minValue) {

		// Just in case the max is negative
		minValue = Math.abs(minValue);

		if (input > 0)
			return Math.max(input, minValue);
		else
			return Math.min(input, -minValue);
	}

	private void autoNextStep() {
		// Reset encoders
		leftEnc.reset();
		rightEnc.reset();

		// Reset the Auton timer
		autoTimer.reset();
		autoTimer.start();

		secondaryTimer.reset();
		secondaryTimer.start();

		freeze();

		// Go to the next step
		autoStep++;
	}

	/**
	 * -------------------------------------------------------------------------------------------------------------------------------
	 * Custom Functions
	 */
	public void regularDrive() {

		// Get joystick inputs for drive base
		double driveLeft = deadZoneComp(leftStick.getY() * -1);
		double driveRight = deadZoneComp(rightStick.getY() * -1);

		if (!overrideControllers) {
			if (gyroMode) {
				teleopGyro(driveLeft, driveRight);
			} else {
				mainDrive.tankDrive(driveLeft, driveRight);
			}
		}
		// Shifter Control
		if (leftStick.getRawButton(1)) {// shift low
			shifter.set(DoubleSolenoid.Value.kReverse);
		}
		if (rightStick.getRawButton(1)) {// shift high
			shifter.set(DoubleSolenoid.Value.kForward);
		}

		if (leftStick.getRawButton(6)) {
			overrideControllers = true;
			gyroMode = false;
			mainDrive.tankDrive(0.5, -0.5);
		}
		if (leftStick.getRawButton(7)) {
			overrideControllers = true;
			gyroMode = false;
			mainDrive.tankDrive(-0.5, 0.5);
		}
		if (!leftStick.getRawButton(6) && !rightStick.getRawButton(7)) {
			overrideControllers = false;
		}
		if (rightStick.getRawButton(11)) {
			gyro.reset();
			gyroMode = true;
		}
		if (rightStick.getRawButton(10)) {
			gyro.reset();
			gyroMode = false;
		}
		if (rightStick.getRawButton(2)) {
			liftOverride = true;
			liftControl(0.3);
		}
		if (rightStick.getRawButton(3)) {
			liftOverride = true;
			liftControl(-0.3);
		}
		if (!rightStick.getRawButton(6) && !rightStick.getRawButton(7)) {
			liftOverride = false;
		}
	}

	public void johnMode() {

		// Get joystick inputs for drive base
		double driveLeft = deadZoneComp(johnController.getRawAxis(1) * -1);
		double driveRight = deadZoneComp(johnController.getRawAxis(5) * -1);

		if (!overrideControllers) {
			if (gyroMode) {
				teleopGyro(driveLeft, driveRight);
			} else {
				mainDrive.tankDrive(driveLeft, driveRight);
			}
		}
		// Shifter Control
		if (johnController.getRawAxis(2) > 0.5) {// shift low w/ left trigger
			shifter.set(DoubleSolenoid.Value.kReverse);
		}
		if (johnController.getRawAxis(3) > 0.5) {// shift high w/ high trigger
			shifter.set(DoubleSolenoid.Value.kForward);
		}

		if (johnController.getPOV() == 90) {
			overrideControllers = true;
			gyroMode = false;
			mainDrive.tankDrive(0.5, -0.5);
		}
		if (johnController.getPOV() == 270) {
			overrideControllers = true;
			gyroMode = false;
			mainDrive.tankDrive(-0.5, 0.5);
		}
		if (johnController.getPOV() != 270 && johnController.getPOV() != 90) {
			overrideControllers = false;
		}
		if (johnController.getAButton()) {
			gyro.reset();
			gyroMode = true;
		}
		if (johnController.getBButton()) {
			gyro.reset();
			gyroMode = false;
		}
		if (johnController.getRawButton(6)) {
			liftOverride = true;
			liftControl(0.3);
		}
		if (johnController.getRawButton(5)) {
			liftOverride = true;
			liftControl(-0.3);
		}
		if (!johnController.getRawButton(5) && !johnController.getRawButton(6)) {
			liftOverride = false;
		}
	}

	public void smartDash() {

		// Encoders
		SmartDashboard.putNumber("Encoder Left [RAW]", leftEnc.getRaw());
		SmartDashboard.putNumber("Encoder Right [RAW]", rightEnc.getRaw());

		SmartDashboard.putNumber("Encoder Left [INCH]", leftEnc.getDistance());
		SmartDashboard.putNumber("Encoder Right [INCH]", rightEnc.getDistance());

		// Limit Switches
		SmartDashboard.putBoolean("topSwitch", liftHigh.get());
		SmartDashboard.putBoolean("lowSwitch", liftLow.get());

		// Gyro
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngleZ());

		// Get Selected Auton Program
		SmartDashboard.putString("Auto Program", autoChooser.getSelected());

		SmartDashboard.putBoolean("John-Mode", isJohnModeEnabled);
		SmartDashboard.putNumber("Auton delay", auto_delay);

		// Get Field data from FMS
		gameData = DriverStation.getInstance().getGameSpecificMessage().toUpperCase();
	}

	// function to keep robot tracking straight during teleop while joysticks are in
	// similar positions
	public void teleopGyro(double leftStick, double rightStick) {
		double error = 0.2;
		double averageSpeed = (leftStick + rightStick) / 2;
		if (leftStick >= rightStick - error && leftStick <= rightStick + error) {
			if (!isTrackingStraight) {
				gyro.reset();
				isTrackingStraight = true;
			}
			double kP_rot = 0.5 / 45.0; // start slowing down at 45 deg.
			double e_rot = 0 - gyro.getAngleZ();
			double rot = e_rot * kP_rot;

			mainDrive.arcadeDrive(averageSpeed, rot);

		} else {
			mainDrive.tankDrive(leftStick, rightStick);
			isTrackingStraight = false;
		}
	}

	public double deadZoneComp(double input) {
		double deadZone = 0.15;
		if (input <= deadZone && input >= -deadZone) {
			return 0;
		} else {
			return input;
		}
	}
}