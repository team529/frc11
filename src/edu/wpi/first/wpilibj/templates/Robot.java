/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
    DriverStation ds;

    DigitalInput lineRt;
    DigitalInput lineCtr;
    DigitalInput lineLeft;
    

    public void robotInit() {

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
