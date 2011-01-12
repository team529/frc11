/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.Accelerometer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    
    RobotDrive drive;
    Jaguar jagLeft;
    Jaguar jagRight;

    DriverStation ds;
    Joystick jsLeft;
    Joystick jsRight;
    
    Encoder encLeft;
    Encoder encRight;

    DigitalInput lineRight;
    DigitalInput lineMid;
    DigitalInput lineLeft;

    Accelerometer accel;
    Gyro gyro;


    static private int crioSlotDigital;
    static private int crioSlotAnalog;
    static private int crioSlotPneumatic;

    public void robotInit() {

        jagLeft = new Jaguar(crioSlotDigital, 1);
        jagRight = new Jaguar(crioSlotDigital, 2);

        drive = new RobotDrive(jagLeft, jagRight);

        ds = DriverStation.getInstance();
        jsLeft = new Joystick(1);
        jsRight = new Joystick(2);

        encLeft = new Encoder(crioSlotDigital, 1, 2);
        encRight = new Encoder(crioSlotDigital, 3, 4);

        lineRight = new DigitalInput(crioSlotDigital, 5);
        lineMid = new DigitalInput(crioSlotDigital, 6);
        lineLeft = new DigitalInput(crioSlotDigital, 7);

        accel = new Accelerometer(crioSlotAnalog, 1, 2, 3);
        gyro = new Gyro(crioSlotAnalog, 4);

    }
    
    public void disabledInit(){
        
    }

    public void disabledPeriodic(){

    }

    public void autonomousInit(){
        
    }

    public void autonomousPeriodic() {

    }

    public void teleopInit(){
        
    }
    
    public void teleopPeriodic() {
        
    }
    
}
