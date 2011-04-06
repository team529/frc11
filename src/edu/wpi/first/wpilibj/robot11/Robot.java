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
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
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
    private CANJaguar cjagMinibot;
    private Jaguar jagLeft;
    private Jaguar jagRight;
    private Jaguar jagLeftD;
    private Jaguar jagRightD;
    private Jaguar jagArm;
    private Jaguar jagArmD;
    private Jaguar jagMinibot;
    private CustomRobotDrive drive;
    private DriverStation ds;
    private DriverStationEnhancedIO dsX;
    private Joystick jsLeft;
    private Joystick jsRight;
    private DigitalInput armIndex;
    private DigitalInput lineRight;
    private DigitalInput lineMid;
    private DigitalInput lineLeft;
    private Compressor compressor;
    private Solenoid[] solenoids;
    private PressureTransducer transducer;
    private ADXL345_I2C accel;
    private Gyro gyroXY;
    private PIDController turnController;
    // Location of cRIO modules
    private static final int kSlotDigital = 4;
    private static final int kSlotAnalog = 1;
    private static final int kSlotPneumatic = 8;
    // Use CAN jags? (Encoders / lim switches read over CAN too)
    private static boolean kUseCAN = true;
    
    // Solenoid mapping for each button on joystick
    // 1-8. Positive enable, Negative disable. 0 for none
    // 1st array is driver stick. 2nd is arm operator
    private static int kSolenoidMapping[][] = {
        //   00  01  02  03  04  05  06  07  08  09  10  11
        {0, 0, 1, -1, 2, -2, 0, 0, 0, 0, 0, 0}, // Left Joystick
        {0, 1, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5}, // Right Joystick
    };
    private static final byte syncGroup = 0x40;
    // Track looping period
    private long period;
    // Autonomous state
    private int aState;

    private void canJaguarInit(CANJaguar cjag) throws CANTimeoutException {
        cjag.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
        cjag.configNeutralMode(CANJaguar.NeutralMode.kBrake);
        //cjag.setVoltageRampRate(0.1);
        //cjag.enableControl();
    }

    public void robotInit() {
        if (kUseCAN) {
            try {
                cjagLeft = new CANJaguar(10);
                cjagRight = new CANJaguar(11);
                canJaguarInit(cjagLeft);
                canJaguarInit(cjagRight);

                cjagLeftD = new CANJaguar(12);
                cjagRightD = new CANJaguar(13);
                canJaguarInit(cjagLeftD);
                canJaguarInit(cjagRightD);

                drive = new CustomRobotDrive(cjagLeft, cjagLeftD, cjagRight, cjagRightD);

                cjagArm = new CANJaguar(20);
                cjagArmD = new CANJaguar(21);
                canJaguarInit(cjagArm);
                canJaguarInit(cjagArmD);

                cjagMinibot = new CANJaguar(30);
                canJaguarInit(cjagMinibot);

            } catch (CANTimeoutException ex) {
                //ex.printStackTrace();
            }
        }
        if (!kUseCAN) {
            jagLeft = new Jaguar(kSlotDigital, 1);
            jagRight = new Jaguar(kSlotDigital, 2);

            jagLeftD = new Jaguar(kSlotDigital, 3);
            jagRightD = new Jaguar(kSlotDigital, 4);

            drive = new CustomRobotDrive(jagLeft, jagLeftD, jagRight, jagRightD);

            jagArm = new Jaguar(kSlotDigital, 5);
            jagArmD = new Jaguar(kSlotDigital, 6);

            jagMinibot = new Jaguar(kSlotDigital, 7);
        }

        //drive = new CustomRobotDrive(motorLeft, motorRight);

        drive.setInvertedMotor(CustomRobotDrive.MotorType.kFrontLeft, false);
        drive.setInvertedMotor(CustomRobotDrive.MotorType.kRearLeft, false);
        drive.setInvertedMotor(CustomRobotDrive.MotorType.kFrontRight, false);
        drive.setInvertedMotor(CustomRobotDrive.MotorType.kRearRight, false);

        ds = DriverStation.getInstance();
        dsX = ds.getEnhancedIO();

        jsLeft = new Joystick(1);
        jsRight = new Joystick(2);

        lineRight = new DigitalInput(kSlotDigital, 5);
        lineMid = new DigitalInput(kSlotDigital, 7);
        lineLeft = new DigitalInput(kSlotDigital, 6);

        // Pressure Switch: 14; Compressor Relay: 1
        compressor = new Compressor(kSlotDigital, 14, kSlotDigital, 1);
        compressor.start();
        solenoids = new Solenoid[8];
        for (int i = 0; i < 8; i++) {
            solenoids[i] = new Solenoid(kSlotPneumatic, i + 1);
        }

        transducer = new PressureTransducer(kSlotAnalog, 3);

        armIndex = new DigitalInput(kSlotDigital, 3);

        accel = new ADXL345_I2C(kSlotDigital, ADXL345_I2C.DataFormat_Range.k4G);
        gyroXY = new Gyro(kSlotAnalog, 1);

        // Calibration works well enough
        //gyroXY.setSensitivity(kGyroSensitivity);
        gyroXY.reset();

        // Turn controller for auto-orient
        turnController = new PIDController(0.07, 0.005, 0.010, gyroXY, new PIDOutput() {

            public void pidWrite(double output) {
                drive.arcadeDrive(0, output * -0.5);
            }
        }, .005);
        //turnController.setInputRange(-360.0, 360.0);
        //turnController.setTolerance(1 / 90. * 100);
        //turnController.setContinuous(true);
        turnController.disable();

    }

    public void disabledInit() {
        //turnController.disable();
        turnController.disable();
        aState = 0;
    }

    public void disabledPeriodic() {
        poll();

    }

    public void autonomousInit() {
        turnController.disable();
        aState = 1;
    }

    public void autonomousPeriodic() {
        poll();
    }

    public void teleopInit() {
        turnController.disable();

        // Shut off all solenoids
        /* When disabled, everything is off. Enabling robot causes
         * unintended consequences. Scary.
         */
        if (aState == 0) { // 0 ~ disabled prior
            for (int i = 0; i < 8; i++) {
                solenoids[i].set(false);
            }
        }
        aState = 2;
    }

    public void teleopPeriodic() {
        double angle = gyroXY.getAngle();

        if (turnController.isEnable()) {
            // Auto-orient
            double err = turnController.getError();
            SmartDashboard.log(err, "Turn Err");
            SmartDashboard.log(turnController.get(), "Turn Out");
            if (jsLeft.getTrigger()) {
                // Trigger to disable
                turnController.disable();
            }
        } else {
            // Drive normally
            drive.arcadeDrive(-jsLeft.getY(), -jsLeft.getX(), true, jsLeft.getTrigger());

            // Use buttons 8 & 9 on drive joystick to auto-orient
            // Trigger to disable
            if (jsLeft.getRawButton(8)) {
                turnController.setSetpoint(angle - (angle % 90));
                turnController.reset();
                turnController.enable();
            } else if (jsLeft.getRawButton(9)) {
                turnController.setSetpoint(angle - (angle % 90) + 90);
                turnController.reset();
                turnController.enable();
            }
        }


        // Activate Solenoids. See line #126 (2nd bookmark)
        for (int i = 1; i <= 11; i++) {
            // Use kSolenoidMapping to map joysticks to solenoids
            int sol;
            if (jsLeft.getRawButton(i)) {
                sol = kSolenoidMapping[0][i];
                if (sol < 0) {
                    solenoids[-sol - 1].set(false);
                } else if (sol > 0) {
                    solenoids[sol - 1].set(true);
                }
            }

            if (jsRight.getRawButton(i)) {
                sol = kSolenoidMapping[1][i];
                if (sol < 0) {
                    solenoids[-sol - 1].set(false);
                } else if (sol > 0) {
                    solenoids[sol - 1].set(true);
                }
            }
        }


        if (jsLeft.getRawButton(7) && (period % 10 == 0)) {
            if (compressor.enabled()) {
                compressor.stop();
                SmartDashboard.log(false, "Compressor");
            } else {
                compressor.start();
                SmartDashboard.log(true, "Compressor");
            }
        }

        // Arm control
        if (kUseCAN) {
            try {
                cjagArm.set(-0.5 * jsRight.getY(), syncGroup);
                cjagArmD.set(-0.5 * jsRight.getY(), syncGroup);
                CANJaguar.updateSyncGroup(syncGroup);
            } catch (CANTimeoutException ex) {
                //ex.printStackTrace();
            }

        } else {
            jagArm.set(-0.5 * jsRight.getY());
            jagArmD.set(-0.5 * jsRight.getY());
        }

        if (jsRight.getRawButton(10)) {
            if (kUseCAN) {
                cjagMinibot.set(0.4);
            } else {
                jagMinibot.set(0.4);
            }
        } else if (jsRight.getRawButton(11)) {
            if (kUseCAN) {
                cjagMinibot.set(-0.4);
            } else {
                jagMinibot.set(-0.4);
            }
        }

        poll();
    }

    private void poll() {
        /* Called on each period
         * Logs things, updates position, etc
         */
        period++; // Used to only excecute on 1/nth of cyles

        log();
    }

    private void log() {
        SmartDashboard.log(ds.getBatteryVoltage(), "Battery Voltage");
        //SmartDashboard.log(ds.getLocation(), "Field Pos");
        SmartDashboard.log(ds.getAlliance() == DriverStation.Alliance.kRed ? "Red" : "Blue",
                "Alliance");

        if (kUseCAN && (period % 2 == 0)) { // Don't overload the CAN network
            try {
                SmartDashboard.log(cjagLeft.getSpeed(), "Jag L Speed");
                //SmartDashboard.log(cjagLeft.getPosition(), "Jag L Pos");

                SmartDashboard.log(cjagLeft.getOutputVoltage(), "Jag L Vout");
                SmartDashboard.log(cjagLeft.getOutputCurrent(), "Jag L Iout");
                SmartDashboard.log(cjagLeftD.getOutputCurrent(), "Jag LD Iout");

                SmartDashboard.log(cjagRight.getSpeed(), "Jag R Speed");
                //SmartDashboard.log(cjagRight.getPosition(), "Jag R Pos");

                SmartDashboard.log(cjagRight.getOutputVoltage(), "Jag R Vout");
                SmartDashboard.log(cjagRight.getOutputCurrent(), "Jag R Iout");
                SmartDashboard.log(cjagRightD.getOutputCurrent(), "Jag RD Iout");

                SmartDashboard.log(false, "CAN Error");

            } catch (CANTimeoutException ex) {
                //ex.printStackTrace();
                SmartDashboard.log(529, "Jag L Speed");
                SmartDashboard.log(529, "Jag L Vout");
                SmartDashboard.log(529, "Jag L Iout");

                SmartDashboard.log(529, "Jag R Speed");
                SmartDashboard.log(529, "Jag R Vout");
                SmartDashboard.log(529, "Jag R Iout");
                SmartDashboard.log(true, "CAN Error");
            }
        }


        SmartDashboard.log(gyroXY.getAngle(), "Gyro");

        //SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kX), "Accel X");
        //SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kY), "Accel Y");
        //SmartDashboard.log(accel.getAcceleration(ADXL345_I2C.Axes.kZ), "Accel Z");

        SmartDashboard.log(lineLeft.get(), "Line L");
        SmartDashboard.log(lineMid.get(), "Line M");
        SmartDashboard.log(lineRight.get(), "Line R");

        //SmartDashboard.log(jsRight.getY(), "Arm JS");
        SmartDashboard.log(!armIndex.get(), "Arm Lowered");

        SmartDashboard.log(transducer.getPSI(), "Pressure (PSI)");
    }
}
