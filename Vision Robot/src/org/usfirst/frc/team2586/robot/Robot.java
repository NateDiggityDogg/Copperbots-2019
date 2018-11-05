/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2586.robot;


import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	
	CameraServer cam;

	public void robotInit() {
		
		NetworkTableInstance homeInstance = NetworkTableInstance.getDefault();
		homeInstance.setServerTeam(2586);
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(640, 480);
		
		NetworkTable table = homeInstance.getTable("Vision");
		NetworkTableEntry xEntry = table.getEntry("X");
		xEntry.setBoolean(true);
		
		

	}

	public void autonomousInit() {
		
	}

	public void autonomousPeriodic() {

	}

	public void teleopPeriodic() {
		
	}

	public void testPeriodic() {
		
	}
}
