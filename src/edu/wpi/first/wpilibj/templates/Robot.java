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


    private Jaguar jagLeft;
    private Jaguar jagRight;

    private RobotDrive drive;
   
    private DriverStation ds;
    private Joystick jsLeft;
    private Joystick jsRight;
    
    private Encoder encLeft;
    private Encoder encRight;

    private DigitalInput lineRight;
    private DigitalInput lineMid;
    private DigitalInput lineLeft;


    private Accelerometer accelX;
    private Accelerometer accelY;
    private Accelerometer accelZ;
    private Gyro gyroXY;


    private static final int kSlotDigital = 2;
    private static final int kSlotAnalog = 1;
    private static final int kSlotPneumatic = 7;

    private static final double kEncDistPerPulse = 30.0;  // in
    private static final double kAccelSensitivity = 0.03; // V/(m/s^2)
    private static final double kAccelZero = 2.50;        // V
    private static final double kGyroSensitivity = 0.05;  // V/(rad/s)

    public void robotInit() {

        jagLeft = new Jaguar(kSlotDigital, 1);
        jagRight = new Jaguar(kSlotDigital, 2);

        drive = new RobotDrive(jagLeft, jagRight);

        ds = DriverStation.getInstance();
        jsLeft = new Joystick(1);
        jsRight = new Joystick(2);

        encLeft = new Encoder(kSlotDigital, 1, 2);
        encRight = new Encoder(kSlotDigital, 3, 4);

        lineRight = new DigitalInput(kSlotDigital, 5);
        lineMid = new DigitalInput(kSlotDigital, 6);
        lineLeft = new DigitalInput(kSlotDigital, 7);

        accelX = new Accelerometer(kSlotAnalog, 1);
        accelY = new Accelerometer(kSlotAnalog, 2);
        accelZ = new Accelerometer(kSlotAnalog, 3);
        gyroXY = new Gyro(kSlotAnalog, 4);


        encLeft.setDistancePerPulse(kEncDistPerPulse);
        encRight.setDistancePerPulse(kEncDistPerPulse);
        encLeft.reset();
        encRight.reset();

        accelX.setSensitivity(kAccelSensitivity);
        accelY.setSensitivity(kAccelSensitivity);
        accelZ.setSensitivity(kAccelSensitivity);
        accelX.setZero(kAccelZero);
        accelY.setZero(kAccelZero);
        accelZ.setZero(kAccelZero);

        gyroXY.setSensitivity(kGyroSensitivity);
        gyroXY.reset();

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
