/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5465.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot 
{
	private Joystick joy;
	private static SmartDashboard dashboard;
	private RobotArm arm;
	private RobotDrive drive;
	private AutoDrive auto;
	private RobotElevator elevator;
	private DigitalInput armswitch;
	private DigitalOutput rampswitch;
	
	public void robotInit() 
	{
		drive = new RobotDrive();
		auto = new AutoDrive(drive);
		arm = new RobotArm();
		//arm.start();
		armswitch = new DigitalInput(0);
		rampswitch = new DigitalOutput(1);
		
		elevator = new RobotElevator();
		//elevator.start();
		
		joy = new Joystick(0);
		
		SmartDashboard.setDefaultNumber("Setpoint", 0.0);
		SmartDashboard.setDefaultNumber("ArmEncoder", 0.0);
		SmartDashboard.setDefaultBoolean("RampSwitch", false);
	}
	
	@SuppressWarnings("static-access")
	public void driveDis()
	{
		drive.resetEncoders();
		drive.setpoint = 0.0;
		drive.recalibrateGyro();
		dashboard.putNumber("Encoder 1", drive.getLMotorEncoder());
		dashboard.putNumber("Encoder 2", drive.getRMotorEncoder());
		dashboard.putNumber("Angle", drive.getAngle());
		dashboard.putNumber("Setpoint", drive.getSetPoint());
		dashboard.putNumber("Error", drive.getError());
		dashboard.putNumber("Distance", drive.getDistance());
	}
	
	public void dis()
	{
		arm.setDisabled(true);
		//driveDis();
	}
	
	public void disabledInit()
	{
		dis();
	}
	
	public void disabledPeriodic()
	{
		dis();
	}

	public void autonomousInit() 
	{
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		boolean switchscale= SmartDashboard.getBoolean("ScaleorSwitch(Switch == Checked)", true);
		String robotpose = SmartDashboard.getString("RobotStartSide:(L(Left), M(Middle), R(Right))", "");
		if(gameData.length() == 3 && robotpose.length() == 1 && (robotpose.charAt(0) == 'L' || robotpose.charAt(0) == 'M' || robotpose.charAt(0) == 'R' ))
		{
			if(switchscale) RobotMap.autoinstructions = robotpose + "T" + gameData.substring(0, 2);
			else RobotMap.autoinstructions = robotpose + "F" + gameData.substring(0,2);
		}
		
		else RobotMap.autoinstructions = "";
	}

	public void autonomousPeriodic() 
	{
		auto.autoPeriodic();
	}
	
	public void teleopInit()
	{
		arm.setDisabled(false);
	}
	
	public void teleopPeriodic() 
	{
		boolean yes = !armswitch.get();
		rampswitch.set(SmartDashboard.getBoolean("RampSwitch", false));
		
		SmartDashboard.putBoolean("MagSwitchOnOff", yes);
		
		SmartDashboard.putNumber("ElevatorEncoder", elevator.getSensorPosition());
		double set = 0.5*(-1*joy.getRawAxis(3)+1)*4096.0;
		//double set = -1*joy.getY();
		
		int set1 = (int)set;
		SmartDashboard.putNumber("Setpoint", set1);
		elevator.setPosition(set1);
		
		/**
		double forward = -1*this.joy.getY();
		double turn = this.joy.getZ();
		this.drive.teleopPIDDrive(forward, turn);**/
	}

	public void testPeriodic() 
	{
		
	}
}
