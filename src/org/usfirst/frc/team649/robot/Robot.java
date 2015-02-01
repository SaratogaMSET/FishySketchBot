
package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;

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
	Victor[] motors;
	Joystick driverRightJoystick;
	Joystick driverLeftJoystick;
	
    public void robotInit() {
    	motors = new Victor[4];
    	for(int i =0; i<4; i++) {
    		motors[i] = new Victor(i);
    	}
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

        for (; i < motors.length / 2; i++) {
            motors[i].set(left);
        }

        for (; i < motors.length; i++) {
            motors[i].set(-right);
        }
    }

}
