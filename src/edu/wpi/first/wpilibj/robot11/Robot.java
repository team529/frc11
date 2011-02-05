/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.robot11;


import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
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

    private CANJaguar cjagArm;
    private CANJaguar cjagArmD;

    private Jaguar jagLeft;
    private Jaguar jagRight;
    private Jaguar jagLeftD;
    private Jaguar jagRightD;
    

    //private PIDSpeedController motorLeft;
    //private PIDSpeedController motorRight;

    private CustomRobotDrive drive;
    private RobotArm arm;
   
    private DriverStation ds;
    private DriverStationEnhancedIO dsX;
    private Joystick jsLeft;
    private Joystick jsRight;
    
    private Encoder encLeft;
    private Encoder encRight;

    private DigitalInput lineRight;
    private DigitalInput lineMid;
    private DigitalInput lineLeft;

    private Compressor compressor;
    private Solenoid[] solenoids;
    private PressureTransducer transducer;

    private ADXL345_I2C accel;
    private Thermometer therm;
    private Gyro gyroXY;

    private PositionTracker posTrack;

    private PIDController turnController;

    // Location of cRIO modules
    private static final int kSlotDigital = 4;
    private static final int kSlotAnalog = 1;
    private static final int kSlotPneumatic = 8;

    // Use two motors per transmission?
    private static final boolean kUseDualMotors = true;
    // Use CAN jags? (Encoders / lim switches read over CAN too)
    private static final boolean kUseCAN = true;
    // Use PID onboard jags. TODO: Tune
    private static final boolean kUseOnboardPid = false;
    // Use PID calc on cRIO to ctrl speed, using PIDSpCtrl. Removed.
    private static final boolean kUsePidSpeed = kUseOnboardPid;
    // Use gyro to use relative controls. TODO: Test craziness
    private static final boolean kUseGyro = false;
    // Use absolute position tracking. TODO: Test accuracy/precision
    private static final boolean kUsePositionTracker = false;
    // Use position control for arm. TODO: Tune
    private static final boolean kUseArmPosition = false;

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


    // Robot Constants for tracking position
    // Scaling factor, should be redundant.
    private static final double kEncDistScale = 1.0;
    // Distance between the wheels of the robot
    private static final double kWheelSpacing = 20;
    // Update only 1:k loops. Probably unessassary?
    private static final int kPositionUpdatePeriod = 1;
    // Initial position of the robot
    private static final double kInitialX = 0;
    private static final double kInitialY = 0;
    private static final double kInitialTheta = Math.PI / 2;




    private static final byte syncGroup = 0x40;
    // Track looping period
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

                    //motorLeft = new PIDSpeedController(cjagLeft, cjagLeftD);
                    //motorRight = new PIDSpeedController(cjagRight, cjagRightD);

                    drive = new CustomRobotDrive(cjagLeft, cjagLeftD, cjagRight, cjagRightD);
                }else{
                    //motorLeft = new PIDSpeedController(cjagLeft);
                    //motorRight = new PIDSpeedController(cjagRight);

                    drive = new CustomRobotDrive(cjagLeft, cjagLeftD);
                }

                cjagArm = new CANJaguar(20);
                cjagArmD = new CANJaguar(21);

                arm = new RobotArm(cjagArm, cjagArmD, kUseArmPosition);

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

                //motorLeft = new PIDSpeedController(jagLeft, jagLeftD, encLeft);
                //motorRight = new PIDSpeedController(jagRight, jagRightD, encRight);
                
                drive = new CustomRobotDrive(jagLeft, jagLeftD, jagRight, jagRightD);
            }else{
                //motorLeft = new PIDSpeedController(jagLeft, encLeft);
                //motorRight = new PIDSpeedController(jagRight, encRight);

                drive = new CustomRobotDrive(jagLeft, jagLeftD);
            }

            //jagArm = new Jaguar(kSlotDigital, 5);
            //jagArmD = new Jaguar(kSlotDigital, 6);
        }

        //drive = new CustomRobotDrive(motorLeft, motorRight);

        drive.setInvertedMotor(CustomRobotDrive.MotorType.kFrontLeft, false);
        //drive.setInvertedMotor(CustomRobotDrive.MotorType.kRearLeft, true);
        drive.setInvertedMotor(CustomRobotDrive.MotorType.kFrontRight, true);
        //drive.setInvertedMotor(CustomRobotDrive.MotorType.kRearRight, false);
        
        if(kUsePidSpeed){
            drive.setMaxOutput(kMaxSpeed);
            /*
            motorLeft.enablePID();
            motorRight.enablePID();
            motorLeft.setPID(kSpeedP, kSpeedI, kSpeedD);
            motorRight.setPID(kSpeedP, kSpeedI, kSpeedD);
             */
        }else{
            drive.setMaxOutput(1.0);
            /*
            motorLeft.disablePID();
            motorRight.disablePID();
             */
        }

        ds = DriverStation.getInstance();
        dsX = ds.getEnhancedIO();

        jsLeft = new Joystick(1);
        jsRight = new Joystick(2);

        
        lineRight = new DigitalInput(kSlotDigital, 5);
        lineMid = new DigitalInput(kSlotDigital, 6);
        lineLeft = new DigitalInput(kSlotDigital, 7);


        // Pressure Switch: 14; Compressor Relay: 1
        compressor = new Compressor(kSlotDigital, 14, kSlotDigital, 1);
        solenoids = new Solenoid[8];
        for(int i = 0; i < 8; i++){
            solenoids[i] = new Solenoid(kSlotPneumatic, i+1);
        }

        transducer = new PressureTransducer(kSlotAnalog, 3);

        accel = new ADXL345_I2C(kSlotDigital, ADXL345_I2C.DataFormat_Range.k4G);
        therm = new Thermometer(kSlotAnalog, 2);
        gyroXY = new Gyro(kSlotAnalog, 1);

        // Calibration works well enough
        gyroXY.setSensitivity(kGyroSensitivity);
        gyroXY.reset();


        if(kUsePositionTracker){
            if(kUseCAN){
                posTrack = new PositionTracker(cjagLeft, cjagRight);
            }else{
                posTrack = new PositionTracker(encLeft, encRight);
            }
            // Initialize position information
            posTrack.setWheelSpacing(kWheelSpacing);
            posTrack.setEncScale(kEncDistScale);
            posTrack.setX(kInitialX);
            posTrack.setY(kInitialY);
            posTrack.setTheta(kInitialTheta);
        }
        

/*        turnControllerd = new PIDController(0.08, 0.0, 0.30, gyroXY, new PIDOutput() {
            public void pidWrite(double output) {
                drive.arcadeDrive(0, output);
            }
        }, .005);*/
        //turnController.setInputRange(-360.0, 360.0);
        //turnController.setTolerance(1 / 90. * 100);
        //turnController.setContinuous(true);
        //turnController.disable();

        

        //SmartDashboard.init();
    }
    
    public void disabledInit(){
        //turnController.disable();
    }

    public void disabledPeriodic(){
        if(kUsePositionTracker && (period % kPositionUpdatePeriod == 0)){
            posTrack.update();
        }

        log();
        
    }

    public void autonomousInit(){
        //turnController.disable();
    }

    public void autonomousPeriodic() {
        if(kUsePositionTracker && (period % kPositionUpdatePeriod == 0)){
            posTrack.update();
        }

        if(lineLeft.get() ^ kInvertLineSensor){
            drive.drive(kAutoSpeed, -kAutoCurve);
        }else if(lineRight.get() ^ kInvertLineSensor){
            drive.drive(kAutoSpeed, -kAutoCurve);
        }else{
            drive.drive(kAutoSpeed, 0.0);
        }

        log();
    }

    public void teleopInit(){
        //turnController.setSetpoint(gyroXY.pidGet());
        //turnController.enable();
    }
    
    public void teleopPeriodic() {
        if(kUsePositionTracker && (period % kPositionUpdatePeriod == 0)){
            posTrack.update();
        }

        if(ds.isNewControlData()){
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

            for(int i = 0; i < 8; i++){
                // Map solenoids 1-8 to buttons 2-9
                jsRight.getRawButton(i + 2);
            }

            //jagArm.set(jsRight.getY(), syncGroup);
            //jagArmD.set(jsRight.getY(), syncGroup);
            //CANJaguar.updateSyncGroup(syncGroup);
            if(kUseCAN){
                arm.set(jsRight.getY());
            }

            // tune PID
            if(period % 10 == 0){
                /*
                 * Tuning PID:
                 * I = 0; D = 0
                 * P++ until oscilations occur
                 * P = P / 2
                 * I++ until change is quick enough
                 * D++ to compensate for load
                 *
                 * Increasing each parameter:
                 *      Rise T  Overshoot   Settle T    StdySt
                 * P:
                 *
                 */
                try {
                    double f = jsLeft.getThrottle() * jsRight.getThrottle();
                    SmartDashboard.log(f, "PID Adj");
                    
                    cjagLeft.setPID(f, 0, 0);
                    cjagLeftD.setPID(f, 0, 0);
                    cjagRight.setPID(f, 0, 0);
                    cjagRightD.setPID(f, 0, 0);
                    /*
                    arm.m_leftJag.setPID(f, 0, 0);
                    arm.m_rightJag.setPID(f, 0, 0);
                     *
                     */
                    //motorRight.setPID(f, 0, 0);
                    //motorRight.setPID(f, 0, 0);
                } catch (CANTimeoutException ex) {
                    ex.printStackTrace();
                }
            }
        }

        log();
    }

    private void log(){
        SmartDashboard.log(ds.getBatteryVoltage(), "Battery Voltage");
        SmartDashboard.log(ds.getLocation(), "Field Pos");
        SmartDashboard.log(ds.getAlliance() == DriverStation.Alliance.kRed ? "Red" : "Blue",
                           "Alliance");

        if(kUseCAN && (period++ % 20 == 0)){ // Don't overload the CAN network
            try {
                SmartDashboard.log(cjagLeft.getSpeed(), "Jag L Speed");
                SmartDashboard.log(cjagLeft.getPosition(), "Jag L Pos");
                SmartDashboard.log(cjagLeft.getOutputVoltage(), "Jag L Vout");
                SmartDashboard.log(cjagLeft.getOutputCurrent(), "Jag L Iout");
                SmartDashboard.log(cjagLeftD.getOutputCurrent(), "Jag LD Iout");

                SmartDashboard.log(cjagRight.getSpeed(), "Jag R Speed");
                SmartDashboard.log(cjagRight.getPosition(), "Jag R Pos");
                SmartDashboard.log(cjagRight.getOutputVoltage(), "Jag R Vout");
                SmartDashboard.log(cjagRight.getOutputCurrent(), "Jag R Iout");
                SmartDashboard.log(cjagRightD.getOutputCurrent(), "Jag RD Iout");
                SmartDashboard.log(false, "CAN Error");

            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
                SmartDashboard.log(529, "Jag L Speed");
                SmartDashboard.log(529, "Jag L Vout");
                SmartDashboard.log(529, "Jag L Iout");

                SmartDashboard.log(529, "Jag R Speed");
                SmartDashboard.log(529, "Jag R Vout");
                SmartDashboard.log(529, "Jag R Iout");
                SmartDashboard.log(true, "CAN Error");
            }
        }else if(!kUseCAN){
            SmartDashboard.log(encLeft.getDistance(), "Enc L Dist");
            SmartDashboard.log(encLeft.getRate(), "Enc L Speed");
            
            SmartDashboard.log(encRight.getDistance(), "Enc R Dist");
            SmartDashboard.log(encRight.getRate(), "Enc R Speed");
        }

        SmartDashboard.log(gyroXY.getAngle(), "Gyro");
        SmartDashboard.log(therm.getTemperature(), "Temp (C)");

        SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kX), "Accel X");
        SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kY), "Accel Y");
        SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kZ), "Accel Z");

        SmartDashboard.log(lineLeft.get(), "Line L");
        SmartDashboard.log(lineMid.get(), "Line M");
        SmartDashboard.log(lineRight.get(), "Line R");

        SmartDashboard.log(transducer.get(), "Pressure");

        SmartDashboard.log(kSpeedP, "Speed loop P");
        SmartDashboard.log(kSpeedI, "Speed loop I");
        SmartDashboard.log(kSpeedD, "Speed loop D");

        if(kUsePositionTracker){
            SmartDashboard.log(posTrack.getX(), "Pos X");
            SmartDashboard.log(posTrack.getY(), "Pos Y");
            SmartDashboard.log(posTrack.getTheta() / Math.PI * 180, "Pos Angle");
        }
    }


}
