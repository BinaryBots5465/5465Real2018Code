package org.usfirst.frc.team5465.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class RobotDrive 
{
	private TalonSRX l1;
	private TalonSRX l2;
	private TalonSRX r1;
	private TalonSRX r2;
	private ADXRS450_Gyro gyro;
	public double setpoint;
	private double kp = 1.0/170.0;
	private double wheelradius = 3;
	private double error = 0.0;
	
	public RobotDrive()
	{
		l1 = new TalonSRX(RobotMap.lmotor1);
		l2 = new TalonSRX(RobotMap.lmotor2);
		l1.configForwardSoftLimitEnable(false, 10);
		l1.configReverseSoftLimitEnable(false, 10);
		l2.configForwardSoftLimitEnable(false, 10);
		l2.configReverseSoftLimitEnable(false, 10);
		
		r1 = new TalonSRX(RobotMap.rmotor1);
		r2 = new TalonSRX(RobotMap.rmotor2);
		r1.configForwardSoftLimitEnable(false, 10);
		r1.configReverseSoftLimitEnable(false, 10);
		r2.configForwardSoftLimitEnable(false, 10);
		r2.configReverseSoftLimitEnable(false, 10);
		
		l1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative , 0, 10);
		r1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative , 0, 10);
		
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
		setpoint = 0;
	}
	
	public void recalibrateGyro()
	{
		this.gyro.reset();
	}
	

	public void teleopPIDDrive(double forward, double turn)
	{
		if(Math.abs(turn) > 0.05)
		{
			this.set(forward, turn);	
			setpoint = gyro.getAngle();
		}
		else
		{
			//this.set(forward,0);
			error = gyro.getAngle() - setpoint;
			double add = error*kp;
			//this.set(forward, 0);
			this.setRaw(forward - add, -1*(forward + add));
		}
	}
	
	public boolean autoDrive(double target)
	{
		if(Math.abs(target -this.getDistance()) > 3)
		{
			teleopPIDDrive(0.15,0);
			return false;
		}
		else
		{
			this.set(0.0, 0.0);
			return true;
		}
	}
	
	public boolean autoTurn(double angle)
	{
		
		if(Math.abs(angle - this.getAngle()) > 1)
		{
			this.set(0.0, 0.15);
			return false;
		}
		else
		{
			this.set(0.0, 0.0);
			setpoint = this.getAngle();
			return true;
			}
	}
	
	public double getError()
	{
		return error;
	}
	
	public double getSetPoint()
	{
		return this.setpoint;
	}
	
	public void set(double forward, double turn)
	{
		l1.set(ControlMode.PercentOutput, forward + turn);
		l2.set(ControlMode.PercentOutput, forward + turn);
		r1.set(ControlMode.PercentOutput, -1*forward + turn);
		r2.set(ControlMode.PercentOutput, -1*forward + turn);
	}
	
	public void setRaw(double left, double right)
	{
		l1.set(ControlMode.PercentOutput, left);
		l2.set(ControlMode.PercentOutput, left);
		r1.set(ControlMode.PercentOutput, right);
		r2.set(ControlMode.PercentOutput, right);
	}
	
	public double getLMotorEncoder()
	{
		return -1*l1.getSelectedSensorPosition(0);
	}
	
	public double getRMotorEncoder()
	{
		return r1.getSelectedSensorPosition(0);
	}
	
	public double getDistance()
	{
		return (((this.getLMotorEncoder()+this.getRMotorEncoder())*0.5)/4096.0)*Math.cos(this.error*Math.PI/180.0)*2*Math.PI*wheelradius;
	}
	
	public void changeToAngle(double a)
	{
		
	}
	
	public void resetEncoders()
	{
		l1.setSelectedSensorPosition(0, 0, 10);
		r1.setSelectedSensorPosition(0, 0, 10);
	}
	
	public double getAngle()
	{
		return gyro.getAngle();
	}
}
