/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.robot11;


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
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

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
    private CANJaguar cjagLeftD;
    private CANJaguar cjagRightD;

    private Jaguar jagLeft;
    private Jaguar jagRight;
    private Jaguar jagLeftD;
    private Jaguar jagRightD;

    private PIDSpeedController motorLeft;
    private PIDSpeedController motorRight;

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

    // Location of cRIO modules
    private static final int kSlotDigital = 4;
    private static final int kSlotAnalog = 1;
    private static final int kSlotPneumatic = 8;

    // Use two motors per transmission?
    private static final boolean kUseDualMotors = true;
    // Use CAN jags? (Encoders / lim switches read over CAN too)
    private static final boolean kUseCAN = true;
    // Use PID onboard jags. Illegal. 
    private static final boolean kUseOnboardPid = true;
    // Use PID calc on cRIO to control speed, instead of voltage
    private static final boolean kUsePidSpeed = false;
    // Use gyro to use relative controls
    private static final boolean kUseGyro = false;

    //
    private static final int kEncCodesPerRev = 250;       // rev
    private static final double kEncDistPerPulse = 250;   // [dist]
    private static final double kGyroSensitivity = 0.05;  // V/(rad/s)

    // PID parameters for speed control
    //TODO: Tune & set as final
    private static double kSpeedP = 0.08;
    private static double kSpeedI = 0.000001;
    private static double kSpeedD = 0.005;

    // Test dark line on white carpet
    private static final boolean kInvertLineSensor = false;
    // Sqrt of deadbad value for joystick
    private static final double kDeadband = 0.2;
    // Maximum speed. Will autoadjust using PIDSpCtrl
    private static final double kMaxSpeed = 1000; // kEncCodesPerRev / min
    // Autonomous speed as %
    private static final double kAutoSpeed = 0.4;
    // Autonomous turing radius as %
    private static final double kAutoCurve = 0.5;

    private long period;

    private void canJaguarInit(CANJaguar cjag) throws CANTimeoutException{
        if(kUseOnboardPid){
            cjag.changeControlMode(CANJaguar.ControlMode.kSpeed);
            cjag.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            cjag.configEncoderCodesPerRev(kEncCodesPerRev);
            cjag.setPID(kSpeedP, kSpeedI, kSpeedD);
        }else{
            cjag.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
        }
        cjag.configNeutralMode(CANJaguar.NeutralMode.kBrake);
        cjag.enableControl();
    }

    public void robotInit() {

        if(kUseCAN){
            try {
                cjagLeft = new CANJaguar(10);
                cjagRight = new CANJaguar(11);

                canJaguarInit(cjagLeft);
                canJaguarInit(cjagRight);

                if(kUseDualMotors){
                    cjagLeftD = new CANJaguar(12);
                    cjagRightD = new CANJaguar(13);

                    canJaguarInit(cjagLeftD);
                    canJaguarInit(cjagRightD);

                    motorLeft = new PIDSpeedController(cjagLeft, cjagLeftD);
                    motorRight = new PIDSpeedController(cjagRight, cjagRightD);
                }else{
                    motorLeft = new PIDSpeedController(cjagLeft);
                    motorRight = new PIDSpeedController(cjagRight);
                }
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }else{
            jagLeft = new Jaguar(kSlotDigital, 1);
            jagRight = new Jaguar(kSlotDigital, 2);

            encLeft = new Encoder(kSlotDigital, 1, 2);
            encRight = new Encoder(kSlotDigital, 3, 4);
            encLeft.setDistancePerPulse(kEncDistPerPulse);
            encRight.setDistancePerPulse(kEncDistPerPulse);
            encLeft.reset();
            encRight.reset();

            if(kUseDualMotors){
                jagLeftD = new Jaguar(kSlotDigital, 3);
                jagRightD = new Jaguar(kSlotDigital, 4);

                motorLeft = new PIDSpeedController(jagLeft, jagLeftD, encLeft);
                motorRight = new PIDSpeedController(jagRight, jagRightD, encRight);
            }else{
                motorLeft = new PIDSpeedController(jagLeft, encLeft);
                motorRight = new PIDSpeedController(jagRight, encRight);
            }
        }

        drive = new RobotDrive(motorLeft, motorRight);

        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        //drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
        //drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
        
        if(kUsePidSpeed){
            drive.setMaxOutput(kMaxSpeed);
            motorLeft.enablePID();
            motorRight.enablePID();
            motorLeft.setPID(kSpeedP, kSpeedI, kSpeedD);
            motorRight.setPID(kSpeedP, kSpeedI, kSpeedD);
        }else{
            drive.setMaxOutput(1.0);
            motorLeft.disablePID();
            motorRight.disablePID();
        }

        ds = DriverStation.getInstance();
        jsLeft = new Joystick(1);
        jsRight = new Joystick(2);

        
        lineRight = new DigitalInput(kSlotDigital, 5);
        lineMid = new DigitalInput(kSlotDigital, 6);
        lineLeft = new DigitalInput(kSlotDigital, 7);

        accel = new ADXL345_I2C(kSlotDigital, ADXL345_I2C.DataFormat_Range.k4G);
        therm = new Thermometer(kSlotAnalog, 2);
        gyroXY = new Gyro(kSlotAnalog, 1);


        

/*        turnController = new PIDController(0.08, 0.0, 0.30, gyroXY, new PIDOutput() {
            public void pidWrite(double output) {
                drive.arcadeDrive(0, output);
            }
        }, .005);*/
        //turnController.setInputRange(-360.0, 360.0);
        //turnController.setTolerance(1 / 90. * 100);
        //turnController.setContinuous(true);
        //turnController.disable();

        // Calibration works well enough
        gyroXY.setSensitivity(kGyroSensitivity);
        gyroXY.reset();

        SmartDashboard.init();
    }
    
    public void disabledInit(){
        //turnController.disable();
    }

    public void disabledPeriodic(){

    }

    public void autonomousInit(){
        //turnController.disable();
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

        if(kUseGyro){
            dir.normalize();
            dir.rotate(gyroXY.getAngle());
            dir.unormalize();
        }
        

        //System.out.println("Gyro" + gyroXY.getAngle());
        if(dir.getR2() > kDeadband){
            drive.arcadeDrive(dir.getX(), dir.getY());
        }else{
            //turnController.setSetpoint(gyroXY.pidGet());
            //turnController.disable();
            drive.stopMotor();
        }

       
/*
        if(jsLeft.getTrigger() && (period++ % 100 == 0)){
            System.out.println("\n\nDebugging:");
            if(kUseCAN){
                try {
                    System.out.println("Left CAN Jag: Speed:" + cjagLeft.getSpeed() + "; Vout:" + cjagLeft.getOutputVoltage() + "; Iout:" + cjagLeft.getOutputCurrent() + "; PID:(" + cjagLeft.getP() + "," + cjagLeft.getI() + "," + cjagLeft.getD() + ")");
                    System.out.println("Rght CAN Jag: Speed:" + cjagRight.getSpeed() + "; Vout:" + cjagRight.getOutputVoltage() + "; Iout:" + cjagRight.getOutputCurrent() + "; PID:(" + cjagRight.getP() + "," + cjagRight.getI() + "," + cjagRight.getD() + ")");
                } catch (CANTimeoutException ex) {
                    //ex.printStackTrace();
                    System.out.println("CAN Error");
                }
            }
            System.out.println("Gyro:" + gyroXY.getAngle());
            System.out.println("Therm: " + therm.getTemperature());
            System.out.println("Accel: X: " + accel.getAcceleration(ADXL345_I2C.Axes.kX) + "");
            System.out.println("Line: Left: " + lineLeft.get() + "; Mid: " + lineMid.get() + "; Right: " + lineRight.get() + "; Inverted: " + kInvertLineSensor);
            
        }
 *
 */
        if(kUseCAN && (period++ % 100 == 0)){ // Don't overload the CAN network
            try {
                SmartDashboard.log(cjagLeft.getSpeed(), "Jag L Speed");
                SmartDashboard.log(cjagLeft.getOutputVoltage(), "Jag L Vout");
                SmartDashboard.log(cjagLeft.getOutputCurrent(), "Jag L Iout");
                SmartDashboard.log(cjagRight.getSpeed(), "Jag R Speed");
                SmartDashboard.log(cjagRight.getOutputVoltage(), "Jag R Vout");
                SmartDashboard.log(cjagRight.getOutputCurrent(), "Jag R Iout");

            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
                SmartDashboard.log(529, "Jag L Speed");
                SmartDashboard.log(529, "Jag L Vout");
                SmartDashboard.log(529, "Jag L Iout");
                SmartDashboard.log(529, "Jag R Speed");
                SmartDashboard.log(529, "Jag R Vout");
                SmartDashboard.log(529, "Jag R Iout");
                System.out.println("CAN Error");
            }
        }
        
        SmartDashboard.log(gyroXY.getAngle(), "Gyro");
        SmartDashboard.log(therm.getTemperature(), "Temp (C)");
        SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kX), "Accel X");
        SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kY), "Accel Y");
        SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kZ), "Accel Z");
        SmartDashboard.log(lineLeft.get(), "Line L");
        SmartDashboard.log(lineMid.get(), "Line M");
        SmartDashboard.log(lineRight.get(), "Line R");
        SmartDashboard.log(kSpeedP, "Speed loop P");
        SmartDashboard.log(kSpeedI, "Speed loop I");
        SmartDashboard.log(kSpeedD, "Speed loop D");
    }
}
