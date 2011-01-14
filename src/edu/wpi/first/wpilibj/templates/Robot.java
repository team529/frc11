/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    private CANJaguar cjagLeft;
    private CANJaguar cjagRight;
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

    private ADXL345_I2C accel;
    private Thermometer therm;
    private Gyro gyroXY;

    private PIDController turnController;

    private static final int kSlotDigital = 2;
    private static final int kSlotAnalog = 1;
    private static final int kSlotPneumatic = 7;
    private static final boolean kUseCAN = true;

    private static final int kEncCodesPerRev = 250;       // rev
    private static final double kEncDistPerPulse = 30.0;  // in
    private static final double kGyroSensitivity = 0.05;  // V/(rad/s)

    private static final boolean kInvertLineSensor = false; // Test dark line on white
    private static final double kDeadband = 0.2;
    private static final double kMaxSpeed = 1000; // kEncCodesPerRev / min
    private static final double kAutoSpeed = 0.4;
    private static final double kAutoCurve = 0.5;
   

    public void robotInit() {

        if(kUseCAN){
            cjagLeft = new CANJaguar(10);
            cjagRight = new CANJaguar(11);

            cjagLeft.changeControlMode(CANJaguar.ControlMode.kSpeed);
            cjagRight.changeControlMode(CANJaguar.ControlMode.kSpeed);

            cjagLeft.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            cjagRight.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);

            cjagLeft.configEncoderCodesPerRev(kEncCodesPerRev);
            cjagRight.configEncoderCodesPerRev(kEncCodesPerRev);

            cjagLeft.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            cjagRight.configNeutralMode(CANJaguar.NeutralMode.kBrake);

            cjagLeft.setPID(0.08, 0.000001, 0.01); //TODO: tune
            cjagRight.setPID(0.08, 0.000001, 0.01);

            cjagLeft.enableControl();
            cjagRight.enableControl();

            drive = new RobotDrive(cjagLeft, cjagRight);
            drive.setMaxOutput(kMaxSpeed);
        }else{
            jagLeft = new Jaguar(kSlotDigital, 1);
            jagRight = new Jaguar(kSlotDigital, 2);
            
            drive = new RobotDrive(jagLeft, jagRight);
        }

        

        ds = DriverStation.getInstance();
        jsLeft = new Joystick(1);
        jsRight = new Joystick(2);

        encLeft = new Encoder(kSlotDigital, 1, 2);
        encRight = new Encoder(kSlotDigital, 3, 4);

        lineRight = new DigitalInput(kSlotDigital, 5);
        lineMid = new DigitalInput(kSlotDigital, 6);
        lineLeft = new DigitalInput(kSlotDigital, 7);

        accel = new ADXL345_I2C(kSlotDigital, ADXL345_I2C.DataFormat_Range.k4G);
        therm = new Thermometer(kSlotAnalog, 2);
        gyroXY = new Gyro(kSlotAnalog, 1);


        encLeft.setDistancePerPulse(kEncDistPerPulse);
        encRight.setDistancePerPulse(kEncDistPerPulse);
        encLeft.reset();
        encRight.reset();

        turnController = new PIDController(0.08, 0.0, 0.30, gyroXY, new PIDOutput() {
            public void pidWrite(double output) {
                drive.arcadeDrive(0, output);
            }
        }, .005);
        turnController.setInputRange(-360.0, 360.0);
        turnController.setTolerance(1 / 90. * 100);
        //turnController.setContinuous(true);
        turnController.disable();

        // Calibration works well enough
        //gyroXY.setSensitivity(kGyroSensitivity);
        //gyroXY.reset();

    }
    
    public void disabledInit(){
        turnController.disable();
    }

    public void disabledPeriodic(){

    }

    public void autonomousInit(){
        turnController.disable();
    }

    public void autonomousPeriodic() {
        if(lineLeft.get() ^ kInvertLineSensor){
            drive.drive(kAutoSpeed, -kAutoCurve);
        }else if(lineRight.get() ^ kInvertLineSensor){
            drive.drive(kAutoSpeed, -kAutoCurve);
        }else{
            drive.drive(kAutoSpeed, 0.0);
        }
    }

    public void teleopInit(){
        //turnController.setSetpoint(gyroXY.pidGet());
        //turnController.enable();
    }
    
    public void teleopPeriodic() {
        Vector dir = new Vector();
        dir.setX(jsLeft.getX());
        dir.setY(jsLeft.getY());
        //dir.normalize();
        //dir.rotate(gyroXY.getAngle());

        //System.out.println("Gyro" + gyroXY.getAngle());
        if(dir.getR2() > kDeadband){
            drive.arcadeDrive(dir.getX(), dir.getY());
        }else{
            //turnController.setSetpoint(gyroXY.pidGet());
            //turnController.disable();
            drive.stopMotor();
        }

        
    }
}
