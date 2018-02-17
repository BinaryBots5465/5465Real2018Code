/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5465.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot 
{	
	private Joystick joy;
	private RobotDrive drive;
	private Timer timer = new Timer();
	private static SmartDashboard dashboard;
	private double counter = 0;
	
	public void robotInit()
	{
		joy = new Joystick(3);
		drive = new RobotDrive();
		timer.start();
		dashboard.setDefaultNumber("Encoder 1", 0.0);
		dashboard.setDefaultNumber("Encoder 2", 0.0);
		dashboard.setDefaultNumber("Angle", 0.0);
		dashboard.setDefaultNumber("SetPoint", 0.0);
		dashboard.setDefaultNumber("Error", 0.0);
		dashboard.setDefaultNumber("Distance", 0.0);
		dashboard.setDefaultBoolean("Go to 60",false);
	}

	
	public void autonomousInit() 
	{
		
	}
	
	public void autonomousPeriodic()
	{
		
	}
	public void dis()
	{
		drive.resetEncoders();
		drive.setpoint = 0.0;
		drive.recalibrateGyro();
		counter = 0;
		dashboard.putNumber("Encoder 1", drive.getLMotorEncoder());
		dashboard.putNumber("Encoder 2", drive.getRMotorEncoder());
		dashboard.putNumber("Angle", drive.getAngle());
		dashboard.putNumber("Setpoint", drive.getSetPoint());
		dashboard.putNumber("Error", drive.getError());
		dashboard.putNumber("Distance", drive.getDistance());
	}
	
	public void disabledInit()
	{
		dis();
	}
	
	public void disabledPeriodic()
	{
		dis();
	}
	
	public void teleopPeriodic() 
	{
		if(dashboard.getBoolean("Go to 60", false))
		{	
			boolean ret = drive.autoDrive(60);
			if(ret)
			{
				boolean ret1 = drive.autoTurn(90);
				if(ret1)
				{
					dashboard.putBoolean("Go to 60", false);
				}
			}
		}
		
		else
		{
			double forward = -1*this.joy.getY();
			double turn = this.joy.getZ();
			
			this.drive.teleopPIDDrive(forward, turn);
		}
		
		dashboard.putNumber("Encoder 1", drive.getLMotorEncoder());
		dashboard.putNumber("Encoder 2", drive.getRMotorEncoder());
		dashboard.putNumber("Angle", drive.getAngle());
		dashboard.putNumber("Setpoint", drive.getSetPoint());
		dashboard.putNumber("Error", drive.getError());
		dashboard.putNumber("Distance", drive.getDistance());

	}
	
	
	public void devisnay(double a)
	{
		dashboard.putString("in devnay", "Yup");
		
		double start = timer.get();
		double current = start;
		
		double end = start + 3;
		while(current < end)
		{
			drive.teleopPIDDrive(0.25, 0.0);
			//drive.set(a*0.25, 0);
			current = timer.get();
		}
	}
	
	public void delay(double seconds)
	{
		double start = timer.get();
		double current = start;
		
		double end = start + seconds;
		while(current < end)
		{
			current = timer.get();
		}
	}

	
	public void testPeriodic() 
	{
	}
}
