package edu.wpi.first.wpilibj.robot11;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SensorBase;

/**
 *
 * @author zach
 */
public class PressureTransducer extends SensorBase implements PIDSource {

    private AnalogChannel m_sensor;
    private static final double k_analog_vcc = 5.00;

    public PressureTransducer(int channel) {
        m_sensor = new AnalogChannel(channel);
    }

    public PressureTransducer(int slot, int channel) {
        m_sensor = new AnalogChannel(slot, channel);
    }

    public double getRaw(){
        return m_sensor.getVoltage();
    }

    public double get(){
        // Vout / Vcc * 100 = .625 * PSI + 10  -- from datasheet
        // PSI = (Vout * 100 / Vcc - 10) / 0.625  -- derived from above
        return (m_sensor.getVoltage() * 100 / k_analog_vcc - 10) / 0.625;
    }

    public double getPSI(){
        return get();
    }

    public double pidGet(){
        return get();
    }
}