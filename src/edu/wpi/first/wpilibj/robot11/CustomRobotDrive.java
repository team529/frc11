/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.robot11;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author zach
 */
 public class CustomRobotDrive extends RobotDrive {
    public CustomRobotDrive(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
    }
    public CustomRobotDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
            SpeedController frontRightMotor, SpeedController rearRightMotor){
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    }
    
    /**
     * Arcade drive implements single stick driving.
     * This function lets you directly provide joystick values from any source.
     * @param moveValue The value to use for forwards/backwards
     * @param rotateValue The value to use for the rotate right/left
     * @param squaredInputs If set, increases the sensitivity at low speeds
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
        // local variables to hold the computed PWM values for the motors
        double leftMotorSpeed;
        double rightMotorSpeed;


        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if(false){ // Use alternate algorithm
            rotateValue *= moveValue;
        }

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
    }

}
