
package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	Victor liftMotor1, liftMotor2, intakeMotorRight, intakeMotorLeft, rollerMotorRight, rollerMotorLeft, autoWinch;
	DoubleSolenoid grabberPiston1, grabberPiston2;
	Victor[] motors;
	Joystick driverRightJoystick;
	Joystick driverLeftJoystick;
	Joystick operatorJoystick;
	public static final double INTAKE_ARM_IN_POWER = 0.2;
	public static final double INTAKE_ARM_OUT_POWER = -0.4;
	public static final double ROLLERS_IN_POWER = 0.4;
	public static final double ROLLER_OUT_POWER = -0.2;
	public static final double AUTO_WINCH_IN_POWER = 0.6;
	boolean grabberState;
	
    public void robotInit() {
    	driverLeftJoystick = new Joystick(0);
    	driverRightJoystick = new Joystick(1);
    	operatorJoystick = new Joystick(2);
    	
    	liftMotor1 = new Victor(0);
    	liftMotor2 = new Victor(1);
    	intakeMotorRight = new Victor(2);
    	intakeMotorLeft = new Victor(3);
    	rollerMotorRight = new Victor(8);
    	rollerMotorLeft = new Victor(9);
    	autoWinch = new Victor(10);
    	grabberPiston1 = new DoubleSolenoid(0, 1);
    	grabberPiston2 = new DoubleSolenoid(2, 3);
    	
    	motors = new Victor[4];
    	motors[0] = new Victor(4);
    	motors[1] = new Victor(5);
    	motors[2] = new Victor(6);
    	motors[3] = new Victor(7);
    	
    	//true == closed, false == open
    	grabberState = false;
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	driveFwdRot(getDriveForward(), getDriveRotation());
        setLiftPower(operatorJoystick.getAxis(Joystick.AxisType.kY));
        
        if(operatorJoystick.getRawButton(8)) {
        	setIntakeArmsPower(INTAKE_ARM_IN_POWER);
        } else if(operatorJoystick.getRawButton(9)) {
        	setIntakeArmsPower(INTAKE_ARM_OUT_POWER);
        } else {
        	setIntakeArmsPower(0);
        }
        
        if(operatorJoystick.getRawButton(5)) {
        	setRollerPower(ROLLER_OUT_POWER);
        } else if(operatorJoystick.getRawButton(3)) {
        	setRollerPower(ROLLERS_IN_POWER);
        } else {
        	setRollerPower(0);
        }
        
        if(operatorJoystick.getRawButton(1) && operatorJoystick.getRawButton(1) != grabberState) {
        	grabberState = !grabberState;
        	setGrabberPistons(grabberState ? Value.kForward: Value.kReverse);
        }
        
        if(operatorJoystick.getRawButton(10)) {
        	runAutoWinch(AUTO_WINCH_IN_POWER);
        } else {
        	runAutoWinch(0);
        }
        
        SmartDashboard.putString("trest", "tesr");
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    public double getDriveForward() {
        return -driverLeftJoystick.getY();
    }

    public double getDriveRotation() {
        final double turnVal = driverRightJoystick.getX();
        final double sign = turnVal < 0 ? -1 : 1;
        return Math.pow(Math.abs(turnVal), 1.4) * sign;
    }

    public void driveFwdRot(double fwd, double rot) {
        double left = fwd + rot, right = fwd - rot;
        double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
        left /= max;
        right /= max;
        rawDrive(left, right);
    }


    public void rawDrive(double left, double right) {
        int i = 0;
        //right
        motors[0].set(-right);
        motors[1].set(-right);
        //left
        motors[2].set(left);
        motors[3].set(left);
    }
    
    public void setLiftPower(double power) {
    	liftMotor1.set(power);
    	liftMotor1.set(power);
    }
    
    public void setIntakeArmsPower(double power) {
    	intakeMotorLeft.set(power);
    	intakeMotorRight.set(power);
    }
    
    public void setRollerPower(double power) {
    	rollerMotorLeft.set(power);
    	rollerMotorRight.set(power);
    }
    
    public void setGrabberPistons(Value state) {
    	grabberPiston1.set(state);
    	grabberPiston2.set(state);
    }
    
    public void runAutoWinch(double power) {
    	autoWinch.set(power);
    }
}
