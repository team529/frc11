/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.robot11;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.parsing.ISensor;

/**
 *
 * @author zach
 */
public class PIDSpeedController implements PIDOutput, ISensor, SpeedController{
    private SpeedController m_motor, m_motorD;
    private Encoder m_enc;
    private CANJaguar m_speedJag;
    private PIDController m_pid;
    private boolean m_pidEnabled = true;
    private boolean m_jagSpeedMode = false;
    
    SpeedSource m_speedSource;
    SpeedOutput m_speedOutput;

    private boolean m_useDual = false;
    private boolean m_useCan = true;
    private boolean m_useCanEnc = false;
    
    private double m_speedRange = 1.0;
    private final double kAdjSpeed = 1.05;

    private final double kDefaultP = 0.08;
    private final double kDefaultI = 0.000001;
    private final double kDefaultD = 0.005;


    private class SpeedSource implements PIDSource{
        public double pidGet() {
            return getSpeed();
        }
    }

    private class SpeedOutput implements PIDOutput{

        public void pidWrite(double output) {
            setMotor(output);
        }

    }


    public PIDSpeedController(SpeedController motor, Encoder enc){
        m_motor = motor;
        m_enc = enc;

        m_useDual = false;
        m_useCanEnc = false;

        initPID();
    }

    public PIDSpeedController(CANJaguar motor){
        m_motor = motor;
        m_speedJag = motor;

        try {
            if(motor.getControlMode().equals(CANJaguar.ControlMode.kSpeed) ){
                m_jagSpeedMode = true;
            }
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }

        m_useDual = false;
        m_useCan = true;
        m_useCanEnc = true;

        initPID();
    }

    public PIDSpeedController(SpeedController motor, SpeedController motorD, Encoder enc){
        m_motor = motor;
        m_motorD = motorD;
        m_enc = enc;

        m_useDual = true;
        m_useCanEnc = false;

        initPID();
    }

    public PIDSpeedController(CANJaguar motor, CANJaguar motorD){
        m_motor = motor;
        m_motorD = motorD;
        m_speedJag = motor;
        try {
            if (!motor.getControlMode().equals(motorD.getControlMode())) {
                System.out.println("ERROR: Jaguar controls not equal!!!!");
            }
            if(motor.getControlMode().equals(CANJaguar.ControlMode.kSpeed) ){
                m_jagSpeedMode = true;
            }
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }


        m_useDual = true;
        m_useCan = true;
        m_useCanEnc = true;

        initPID();
    }

    private void initPID(){
        //m_pid = new PIDController(kDefaultP, kDefaultI, kDefaultD,
        //                          m_speedSource, m_speedOutput);
        //m_pid.setInputRange(-m_speedRange, m_speedRange);
    }

    public double getSpeed(){
        double speed = 0;
        if(m_useCanEnc){
            try {
                speed = m_speedJag.getSpeed();
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }else{
            speed = m_enc.getRate();
        }
        if(Math.abs(speed) > m_speedRange){
            m_speedRange = speed * kAdjSpeed;
            m_pid.setInputRange(-m_speedRange, m_speedRange);
        }
        return speed;
    }

    public void setMotor(double output){
        if(m_useCan){
            if(m_jagSpeedMode){
                output *= m_speedRange;
            }
            try {
                m_motor.set(output, (byte) 16);
                m_motorD.set(output, (byte) 16);
                CANJaguar.updateSyncGroup((byte) 16);
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
                m_motor.set(output);
                m_motorD.set(output);
            }
        }else{
            m_motor.set(output);
            m_motorD.set(output);
        }
    }

    public void pidWrite(double output) {
        set(output);
    }

    public double get() {
        return m_motor.get();
    }

    public void set(double speed, byte syncGroup) {
        set(speed);
    }

    public void set(double speed) {
        if (m_pidEnabled) {
            m_pid.setSetpoint(speed);
        } else {
           setMotor(speed);
        }
    }

    public void disable() {
        //m_pid.disable();
        m_motor.disable();
        if(m_useDual){
            m_motorD.disable();
        }
    }

    public void setPID(double p, double i, double d){
        //m_pid.setPID(p, i, d);

    }


    public void enablePID(){
        m_pidEnabled = true;
        //m_pid.enable();
        setMotor(0);
    }

    public void disablePID(){
        m_pidEnabled = false;
        //m_pid.disable();
        setMotor(0);
    }

    public double getSpeedRange(){
        return m_speedRange;
    }

    public void setSpeedRange(double sr){
        m_speedRange = sr;
    }
}
