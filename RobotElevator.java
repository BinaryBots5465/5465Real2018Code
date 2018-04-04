package org.usfirst.frc.team5465.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class RobotElevator extends Thread
{
	private TalonSRX motor1;
	private int timeoutMs = 10;
	private int totalcount = 4096;
	private double kp = 1.2/4096.0;
	private double ki = 0;
	private double kd = 1.0/20000.0;
	private boolean disabled;
	
	private double lasterror, sumerror;
	private static Timer timer;
	private double start = 0;
	
	private int setpoint;
	
	public RobotElevator()
	{
		motor1 = new TalonSRX(RobotMap.elevator);
		motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeoutMs);
		motor1.setSelectedSensorPosition(0, 0, timeoutMs);
		
		setpoint = 0;
		disabled = false;
		kd = 0.0;
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
		sumerror +=ki*error;
		
		double derivative = (error - lasterror)/dt;
		
		double speed = kp * error + kd*derivative + sumerror;		
		motor1.set(ControlMode.PercentOutput, -1*speed);
		
		lasterror = error;
		start = currt;
	}
	
	public void setDisabled(boolean a)
	{
		disabled = a;
	}
	
	public double getSensorPosition()
	{
		return -1*motor1.getSelectedSensorPosition(0);
	}
}
