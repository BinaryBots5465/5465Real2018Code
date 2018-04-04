package org.usfirst.frc.team5465.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class RobotArm extends Thread
{
	private TalonSRX motor1;
	private int timeoutMs = 10;
	private double kp = 1.8/6480.0;
	private double ki = 2.0/100.0;
	private double kd = 0.0;
	private boolean disabled;
	
	private double lasterror, sumerror;
	private static Timer timer;
	private double start = 0;
	
	private int setpoint;
	
	public RobotArm()
	{
		motor1 = new TalonSRX(RobotMap.arm);
		motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeoutMs);
		motor1.setSelectedSensorPosition(0, 0, timeoutMs);
		setpoint = 0;
		disabled = false;
		
		timer = new Timer();
		timer.start();
	}
	
	public void run()
	{
		start = timer.get();
		while(true)
		{
			if(!disabled) this.teleopPeriodic();
			else this.motor1.set(ControlMode.PercentOutput, 0.0);
		}
	}
	
	public void setPosition(int setpoint)
	{
		this.setpoint = setpoint;
	}
	
	public void teleopPeriodic()
	{
		double currt = timer.get();
		double dt = (currt - start);
		
		int endposition = (int) setpoint;
		int curr = motor1.getSelectedSensorPosition(0);
		double error = (endposition - curr)*1.0;
		if(setpoint != 0) sumerror +=ki*error;
		if(setpoint == 0 && Math.abs(error) < 500)
		{
			sumerror = 0.0;
			motor1.setSelectedSensorPosition(0, 0, timeoutMs);
		}
		double derivative = (error - lasterror)/dt;
		double speed = kp * error + kd*derivative + sumerror;		
		motor1.set(ControlMode.PercentOutput, speed);
		
		lasterror = error;
		start = currt;
	}
	
	public void setDisabled(boolean a)
	{
		disabled = a;
	}
	
	public double getSensorPosition()
	{
		return motor1.getSelectedSensorPosition(0);
	}
}
