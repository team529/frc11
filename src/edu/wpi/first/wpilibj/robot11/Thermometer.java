/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.robot11;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.parsing.ISensor;

/**
 *
 * @author zach
 */
public class Thermometer extends SensorBase implements ISensor {

    static final double kDefaultVoltsPerDegree = 0.009;
    static final double kDefaultZeroVolts = 2.5;
    static final double kDefaultZeroDegrees = 25; // C
    private AnalogChannel m_analogChannel;
    private boolean m_allocatedChannel;
    private double m_voltsPerDegree;
    private double m_zeroVolts;
    private double m_zeroDegrees;

    public Thermometer(final int channel) {
        m_allocatedChannel = true;
        m_analogChannel = new AnalogChannel(channel);
    }

    public Thermometer(final int slot, final int channel) {
        m_allocatedChannel = true;
        m_analogChannel = new AnalogChannel(slot, channel);
    }

    /*
    protected void free() {
    if (m_analogChannel != null && m_allocatedChannel) {
    m_analogChannel.free();
    }
    m_analogChannel = null;
    }
     */
    public double getTemperature() {
        return (m_analogChannel.getAverageVoltage() - m_zeroVolts) / m_voltsPerDegree;
    }

    /**
     * @param m_voltsPerDegree the m_voltsPerDegree to set
     */
    public void setSensitivity(double voltsPerDegree) {
        this.m_voltsPerDegree = voltsPerDegree;
    }

    /**
     * @param m_zeroVolts the m_zeroVolts to set
     */
    public void setZero(double zeroVolts) {
        this.m_zeroVolts = zeroVolts;
    }
}
