/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //Speed controller
  //TODO: Set the correct values for these.
  private static final boolean MOTOR_INVERT = false;//if motor moves in wrong direction, flip this
  private static final int WINCH_ID = 5;//change this to the correct talon ID.
  
  //Sensor
  //TODO: Set the correct values for these
  private static final int CPR = 4096;//clicks per rotation
  private static final boolean SENSOR_INVERT = false;//if sensor outputs are wrong, flip this
  private static final int PID_SLOT = 0;
  private static final int TIMEOUT = 30;
  private static final double kF = 0.0;
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  //Game Piece characteristics
  private static final double HP_RADIUS = 8;//inches
  private static final double CARGO_RADIUS = 6.5;//inches

  //Field Characteristics (Relative to ground)
  private static final double gLOW_HATCH_HEIGHT = 19;//to center
  private static final double gROCKET_PORT1 = 27.5;//to center
  private static final double gCARGOSHIP_PORT = 31.5 + CARGO_RADIUS + 1;// ball has to clear this, 1 inch extra
  private static final double ROCKET_HEIGHTCHANGE = 28.0;//center to center between rocket goals

  //Lift characteristics
  private static final double CARGO_MECH_BOTTOM = 7;//from ground to center of mech in lift starting position
  private static final double HP_MECH_BOTTOM = 15;//from ground to center of mech in lift starting position
  private static final double DRUM_DIAMETER = 2.1;//drum of winch
  private static final double DRUM_CIRCUMFERENCE = Math.PI * DRUM_DIAMETER;//inches

  //Field Characteristics (Relative to mechanisms in lift starting position)
  private static final double mLOW_HATCH_HEIGHT = gLOW_HATCH_HEIGHT - HP_MECH_BOTTOM;
  private static final double mROCKET_PORT1 = gROCKET_PORT1 - CARGO_MECH_BOTTOM;
  private static final double mCARGOSHIP_PORT = gCARGOSHIP_PORT - CARGO_MECH_BOTTOM;

  //String array of setpoint names
  String[] setpointNames ={
    "Starting Config",
    "Low Hatch",
    "Rocket Port 1",
    "Cargo Ship Port",
    "Rocket Hatch 2",
    "Rocket Port 2",
    "Rocket Hatch 3",
    "Rocket Port 3"
  };

  //Lift Setpoints Array in height order (bottom to top)
  double[] setpoints ={
      0.0,//bottom
      mLOW_HATCH_HEIGHT,
      mROCKET_PORT1,
      mCARGOSHIP_PORT,
      (mLOW_HATCH_HEIGHT + ROCKET_HEIGHTCHANGE),//Rocket Hatch 2
      (mROCKET_PORT1 + ROCKET_HEIGHTCHANGE),//Rocket Port 2
      (mLOW_HATCH_HEIGHT + (ROCKET_HEIGHTCHANGE * 2)),//Rocket Hatch 3
      (mROCKET_PORT1 + (ROCKET_HEIGHTCHANGE * 2))//Rocket Port 3
  };

  //see closed loop doc for info on how PID Functions:
  //  https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html
  //Notes on the PIDF Controller:
  //  -Takes in target and sensor velocity measurements every 100ms
  //  -Takes values in raw sensor units
  //  -motor output calculated so that 1023 is interpreted as full. 341 sensor units
  //   of error will produce full motor output.

  
  //TODO: What do we want our Gain profile to look like?
  //      Once we have PID working decently, do we want to try Motion Magic?
  //        https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#motion-magic-control-mode
  //      Would allow a trapezoidal profiling, so we could control cruise velocity, and starting (de)acceleration rates.


  //TODO: For testing, do the following
  //    1) bring up the sensors/encoders so that they are working as desired.
  //          https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html

  //    2) Record the maximum sensor velocity at 100% output (part of the bring up procedure)? for Motion Magic
  //          (kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
  //    3) Tune gain values to get PID functioning properly
  //        https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#motion-magic-position-velocity-current-closed-loop-closed-loop
  //    4) Calculate kF for Motion Magic? (Time permitting)
  TalonSRX winch;
  XboxController buttonBoard;
  
  @Override
  public void robotInit() {
      winch = new TalonSRX(WINCH_ID);
      //set winch to do nothing at the beginning
      winch.set(ControlMode.PercentOutput,0);
      //whether to invert motor direction
      winch.setInverted(MOTOR_INVERT);
      //start in brake mode, for regenerative braking.
      winch.setNeutralMode(NeutralMode.Brake);
      
      //fastest time period where speed can be ramped from 0 to 1. In seconds.
      //winch.configClosedloopRamp(.1);
      
      //set the maximum speed for the controller (percent of max speed)
      //winch.configPeakOutputForward(.5);
      //winch.configPeakOutputReverse(.5);

      //can set the controller to adjust power based on battery voltage automatically.
      //winch.enableVoltageCompensation();

      //Sensor config
      winch.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,//TODO: Is this correct for our encoder?
                                         0,
                                         TIMEOUT
                                        );
      winch.setSensorPhase(SENSOR_INVERT);
      
      //nominal/peak values
      winch.configNominalOutputForward(0);
      winch.configNominalOutputReverse(0);
      winch.configPeakOutputForward(1);
      winch.configPeakOutputReverse(-1);

      //Configure gain values
      winch.config_kF(PID_SLOT, kF);
      winch.config_kP(PID_SLOT, kP);
      winch.config_kI(PID_SLOT, kI);
      winch.config_kD(PID_SLOT, kD);


      int absolutePosition = winch.getSensorCollection().getPulseWidthPosition();

      /* Mask out overflows, keep bottom 12 bits */
      absolutePosition &= 0xFFF;
      if (SENSOR_INVERT) { absolutePosition *= -1; }
      if (MOTOR_INVERT) { absolutePosition *= -1; }
      
      /* Set the quadrature (relative) sensor to match absolute */
      winch.setSelectedSensorPosition(absolutePosition, 0, TIMEOUT);
      
      //TODO: Force into starting position, then set baseline relative position? See the sample code:
      //https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/PositionClosedLoop/src/main/java/frc/robot/Robot.java
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Joystick Y Value", buttonBoard.getRawAxis(1));
    SmartDashboard.putNumber("Encoder Value", winch.getSelectedSensorPosition());
    SmartDashboard.putNumber("Heading", heading);
    SmartDashboard.putString("Target Position", setpointNames[x]);
  }
  
  @Override
  public void autonomousInit() {

  }


  @Override
  public void autonomousPeriodic() {

  }

  double heading = 0;//Lift target
  int x = 0;//Setpoints array address value

  public static int toClicks(double inches) {
    return (int) (inches / DRUM_CIRCUMFERENCE * CPR);
  }

  @Override
  public void teleopPeriodic() {

    //Elevator PID Control
    /*
    //Go to absolute lift bottom
    if(buttonBoard.getRawButtonPressed(1)) {
      heading = setpoints[0];
    }

    //Go to rocket level 3
    if(buttonBoard.getRawButtonPressed(1)) {
      heading = setpoints[setpoints.length - 1];
    }

    //Lift moves up one setpoint
    if(buttonBoard.getRawButtonPressed(3) && x <= (setpoints.length - 1)) {
      heading = setpoints[++x];
    }

    //Lift moves down one setpoint
    if(buttonBoard.getRawButtonPressed(2) && x >= 0) {
      heading = setpoints[--x];
    }
    
    //Move elevator into the desired position
    winch.set(ControlMode.Position,toClicks(heading));
    */

    //For initial testing
    winch.set(ControlMode.PercentOutput, buttonBoard.getRawAxis(1));//figure out motor direction
    winch.getSensorCollection().getPulseWidthPosition();//figure out sensor direction
  }


  @Override
  public void testPeriodic() {
  }
}
