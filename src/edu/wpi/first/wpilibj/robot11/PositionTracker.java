/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.robot11;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * Uses wheel encoders to track position of the robot
 * Must be updated often for accurate results.
 * Assumes CWD with no slipping
 *
 * @author zach
 */
public class PositionTracker {
    private Encoder m_leftEnc;
    private Encoder m_rightEnc;
    private CANJaguar m_leftJag;
    private CANJaguar m_rightJag;

    private boolean m_useJaguars = false;
    private double m_leftDist;
    private double m_rightDist;

    private double m_wheelSpacing = 1.0;
    private double m_encScale = 1.0;

    private double m_xPos = 0;
    private double m_yPos = 0;
    private double m_theta = 0;

    public PositionTracker(Encoder leftEnc, Encoder rightEnc){
        m_leftEnc = leftEnc;
        m_rightEnc = rightEnc;
        m_useJaguars = false;

        // Throw out initial values
        getLeftDist();
        getRightDist();
    }

    public PositionTracker(CANJaguar leftJag, CANJaguar rightJag){
        m_leftJag = leftJag;
        m_rightJag = rightJag;
        m_useJaguars = true;

        getLeftDist();
        getRightDist();
    }

    public void update(){
        double dl, dr;
        double vavg;
        dl = getLeftDist();
        dr = getRightDist();
        vavg = 0.5 * (dl + dr);

        m_theta += (0.5 * (dl - dr) / m_wheelSpacing) * 180 / Math.PI;
        m_xPos += vavg * Math.cos(m_theta);
        m_yPos += vavg * Math.sin(m_theta);
    }

    private double getLeftDist(){
        double cpos = m_leftDist, lastpos;
        if(m_useJaguars){
            try {
                cpos = m_leftJag.getPosition();
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }else{
            cpos = m_leftEnc.getDistance();
        }
        lastpos = m_leftDist;
        m_leftDist = cpos;
        return (cpos - lastpos) / m_encScale;
    }

    private double getRightDist(){
        double cpos = m_rightDist, lastpos;
        if(m_useJaguars){
            try {
                cpos = m_rightJag.getPosition();
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }else{
            cpos = m_rightEnc.getDistance();
        }
        lastpos = m_rightDist;
        m_rightDist = cpos;
        return (cpos - lastpos) / m_encScale;
    }

    public double getTheta() {
        return m_theta;
    }

    public double getX() {
        return m_xPos;
    }

    public double getY() {
        return m_yPos;
    }

    public double getWheelSpacing(){
        return m_wheelSpacing;
    }

    public double getEncScale(){
        return m_encScale;
    }

    public void setTheta(double theta) {
        this.m_theta = theta % (Math.PI * 2);
    }

    public void setX(double x) {
        this.m_xPos = x;
    }

    public void setY(double y) {
        this.m_yPos = y;
    }

    public void setWheelSpacing(double ws){
        if(ws <= 0) // Spacing between wheels must be a positive distance
            return;
        this.m_wheelSpacing = ws;
    }

    public void setEncScale(double sc){
        if(sc == 0) // Scaling cannot be 0, will cause dbz err
            return;
        this.m_encScale = sc;
    }


}
