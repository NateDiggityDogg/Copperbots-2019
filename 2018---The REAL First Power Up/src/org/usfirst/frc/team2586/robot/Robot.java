/**
 * CopperBots FRC 2018 Robot Code
 * 
 */

package org.usfirst.frc.team2586.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
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
	XboxController operatorController;
	Joystick leftStick, rightStick;

	// Speed Controllers
	WPI_TalonSRX frontLeft, frontRight, rearLeft, rearRight, lift;
	WPI_VictorSPX intakeLeft, intakeRight;

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
	DigitalInput liftLow, liftHigh, liftMid;

	// Smart Dash Choosers
	SendableChooser<String> autoChooser = new SendableChooser<>();
	final String autoChooserNone = "None";
	final String autoChooserLine = "line";
	final String autoChooserSwitchCenter = "Center Switch";
	final String autoChooserSwitchLeft = "Left Switch";
	final String autoChooserSwitchRight = "Right Switch";

	/*
	 * VARIABLE DECLARATIONS
	 */

	// Naming mainDrive
	DifferentialDrive mainDrive;

	// Disable soft-limits and let the operator do as they please
	boolean limitBreak = false;

	/*
	 * AUTON STATE VARS
	 */

	// The game data received from the game
	String gameData;

	// Current step in the Auton Program
	int autoStep = 0;

	// Auton State Timer
	Timer autoTimer;

	// autoDrive state vars
	double dPrev = 0.0;
	double dHeading = 0.0;

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
		clamp = new DoubleSolenoid(2, 3);
		intakeDeploy = new DoubleSolenoid(4, 5);

		// Controllers
		// ******driverController = new XboxController(0);
		operatorController = new XboxController(0);
		leftStick = new Joystick(3);
		rightStick = new Joystick(2);

		// Encoders
		leftEnc = new Encoder(0, 1);
		rightEnc = new Encoder(2, 3);

		leftEnc.setDistancePerPulse(10.0 / 10.0); // [Inches/Pulses]
		rightEnc.setDistancePerPulse(10.0 / 10.0); // [Inches/Pulses]

		// Gyro
		gyro = new ADIS16448_IMU();
		gyro.calibrate();

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
		liftLow = new DigitalInput(9);
		liftHigh = new DigitalInput(8);

		// Intake
		intakeLeft = new WPI_VictorSPX(7);
		intakeRight = new WPI_VictorSPX(6);

		// declaring the drive system
		mainDrive = new DifferentialDrive(frontLeft, frontRight);
		/** Redundant line? */
		mainDrive.setSafetyEnabled(false);

		// declaring sendable chooser for auton delay
		SendableChooser delayUpload = new SendableChooser();

		// Auto Program Chooser [Smart Dash]
		autoChooser.addDefault(autoChooserLine, autoChooserLine);
		autoChooser.addObject(autoChooserNone, autoChooserNone);
		autoChooser.addObject(autoChooserSwitchCenter, autoChooserSwitchCenter);
		autoChooser.addObject(autoChooserSwitchLeft, autoChooserSwitchLeft);
		autoChooser.addObject(autoChooserSwitchRight, autoChooserSwitchRight);
		SmartDashboard.putData("Auto Selection", autoChooser);
		/**
		 * &&&&&&&double auto_delay = SmartDashboard.getNumber("Autonomous
		 * Delay(Seconds)", 0); double ms_auto_delay = auto_delay / 1000;
		 */

		// Auton State Vars
		autoTimer = new Timer();
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

		// Get joystick inputs for drive base
		double driveLeft = leftStick.getY() * -1;
		double driveRight = rightStick.getY() * -1;

		mainDrive.tankDrive(driveLeft, driveRight);

		// Shifter Control
		if (leftStick.getRawButton(1)) {// shift low
			shifter.set(DoubleSolenoid.Value.kForward);
		}
		if (rightStick.getRawButton(1)) {// shift high
			shifter.set(DoubleSolenoid.Value.kReverse);
		}

		// Lift Control
		double liftCommand = operatorController.getRawAxis(1) * -1;

		// Override all soft limits
		if (operatorController.getBackButton())
			limitBreak = true;

		if (operatorController.getStartButton())
			limitBreak = false;

		// Send speed command to the lift
		if (limitBreak) {
			lift.set(liftCommand);
		} else {
			liftControl(liftCommand);
		}

		// Intake Control
		// Each trigger should allow you to either intake or eject
		double intakeCommand = operatorController.getTriggerAxis(GenericHID.Hand.kLeft)
				- operatorController.getTriggerAxis(GenericHID.Hand.kRight);
		intakeLeft.set(intakeCommand);
		intakeRight.set(intakeCommand);

		// Claw Control
		if (operatorController.getAButton()) {
			clamp.set(DoubleSolenoid.Value.kReverse); // Open
		}
		if (operatorController.getBButton()) {
			clamp.set(DoubleSolenoid.Value.kForward); // Close
		}
		if (operatorController.getXButton()) {
			clamp.set(DoubleSolenoid.Value.kOff); // float
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
			// Drive forward
			mainDrive.arcadeDrive(0.35, 0.0);

			// Drive for a bit
			if (autoTimer.get() > 1.0)
				autoNextStep();
			break;

		case 2:
			// Stop!
			mainDrive.stopMotor();

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
		double rot = 45;
		if (gameData.startsWith("R"))
			rot = -rot;

		// Switch case for AUTONOMOUS SWITCH
		switch (autoStep) {

		case 1:
			if (autoDrive(24.0))
				autoNextStep();
			break;

		case 2:
			if (autoTurn(rot))
				autoNextStep();
			break;

		case 3:
			if (autoDrive(48.0))
				autoNextStep();
			break;

		case 4:
			if (autoTurn(0.0))
				autoNextStep();
			break;

		case 5:
			if (autoDrive(36.0))
				autoNextStep();
			break;

		case 6:
			// Drop it like it's hot...
			clamp.set(DoubleSolenoid.Value.kReverse); // Open

			// Stop!
			mainDrive.stopMotor();

			break;
		}

		// Raise the arm up to the mid-position switch
		if (!liftMid.get())
			// lift up slowly
			liftControl(0.3);
		else
			// hold position
			lift.set(0.0);

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
		double maxSpeed = 0.4;

		//
		// Linear
		//

		// Get Encoder values [Inches]
		double l = leftEnc.getDistance();
		double r = rightEnc.getDistance();

		// If an encoder fails, we assume that it stops generating pulses
		// so use the larger of the two (absolute distance)
		double d;
		if (Math.abs(l) > Math.abs(r))
			d = l;
		else
			d = r;

		// Proportional control to get started
		// TODO: add integral term later perhaps
		double kP = maxSpeed / 36.0; // Start to slow down at 36 inches from target
		double e_lin = distance - d;
		double lin = e_lin * kP;

		// Ramp up to speed to reduce wheel slippage
		// TODO: We could probably use .getRate() and control the actual acceleration...
		double max_ramp_up = 0.075;
		if (lin > dPrev + max_ramp_up)
			lin = dPrev + max_ramp_up;
		dPrev = lin;

		// Limit max speed
		lin = absMax(lin, maxSpeed);

		//
		// Rotation
		//

		/** Where is kP_rot derived from? & why 0.5? */
		double kP_rot = maxSpeed / 45.0; // start slowing down at 45 deg.
		double e_rot = dHeading - gyro.getYaw();
		double rot = e_rot * kP_rot;

		// Max rotation speed
		rot = absMax(rot, maxSpeed);

		// Nothing left but to do it...
		mainDrive.arcadeDrive(lin, rot);

		// Determine if the robot made it to the target
		// and then wait a bit so that it can correct any overshoot.
		if (e_lin > 0.5 || e_rot > 2.0)
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
		double maxSpeed = 0.4;

		// Update the target heading for autoDrive function
		dHeading = heading;

		//
		// Rotation
		//
		/** Where is kP_rot derived from? & why 0.5? */
		double kP_rot = maxSpeed / 45.0; // start slowing down at 45 deg.
		double e_rot = heading - gyro.getYaw();
		double rot = e_rot * kP_rot;

		// Max rotation speed
		rot = absMax(rot, maxSpeed);

		// Nothing left but to do it...
		mainDrive.arcadeDrive(0.0, rot);

		// Determine if the robot made it to the target
		// and then wait a bit so that it can correct any overshoot.
		if (e_rot > 2.0)
			autoTimer.reset();

		else if (autoTimer.get() > 0.75)
			return true;

		// Keep trying...
		return false;
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

	private void autoNextStep() {
		// Reset encoders
		leftEnc.reset();
		rightEnc.reset();

		// Reset the Auton timer
		autoTimer.reset();
		autoTimer.start();

		// Go to the next step
		autoStep++;
	}

	/**
	 * -------------------------------------------------------------------------------------------------------------------------------
	 * Custom Functions
	 */

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
		SmartDashboard.putNumber("Gyro Angle", gyro.getYaw());

		// Get Selected Auton Program
		SmartDashboard.putString("Auto Program", autoChooser.getSelected());

		// Get Field data from FMS
		gameData = DriverStation.getInstance().getGameSpecificMessage().toUpperCase();
	}

}