/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.util.MathUtils;

/**
 *
 * @author zach
 */
public class Vector {

    private double x;
    private double y;
    //static final int kRectangular = 1;
    //static final int kPolar = 2;

    /**
     * @return the x
     */
    public double getX() {
        return x;
    }

    /**
     * @param x the x to set
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * @return the y
     */
    public double getY() {
        return y;
    }

    /**
     * @param y the y to set
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * @return the r
     */
    public double getR() {
        return Math.sqrt(x * x + y * y);
    }

    public double getR2(){
        return x * x + y * y;
    }

    /**
     * @param r the r to set
     */
    public void setR(double r) {
        if (getR2() == 0) {
            x = r;
            y = 0;
        } else {
            x *= r / getR();
            y *= r / getR();
        }
    }

    /**
     * @return the theta
     */
    public double getTh() {
        // Surprisingly enough, they crippled java.lang.Math
        return MathUtils.atan2(y, x);
    }

    public double getThDeg() {
        return getTh() * 180 / Math.PI;
    }
    
    /**
     * @param th the theta to set
     */
    public void setTh(double th) {
        double r = getR();
        x = r * Math.cos(th);
        y = r * Math.sin(th);
    }

    public void setThDeg(double th) {
        setTh(th / 180 * Math.PI);
    }

    public void setRect(double x, double y) {
        setX(x);
        setY(y);
    }

    public void setPolar(double r, double th) {
        setR(r);
        setTh(th);
    }

    /**
     * Normalize joystick values to unit circle
     * Only needs to be called once after reading joystick values
     */
    public void normalize() {
        double th = getTh();
        double k = Math.max(Math.abs(Math.sin(th)), Math.abs(Math.cos(th)));
        x *= k;
        y *= k;
    }

    public void rotate(double angle){
        setThDeg(getThDeg() + angle);
    }
    
    public void zero(){
        x = 0;
        y = 0;
    }
}
