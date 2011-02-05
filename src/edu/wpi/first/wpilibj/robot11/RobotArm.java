/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.robot11;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author zach
 */
public class RobotArm {

    public CANJaguar m_leftJag;
    public CANJaguar m_rightJag;

    private AnalogChannel m_pot;
    private Encoder m_enc;
    private DigitalInput m_indexSw;

    private boolean kUsePositionControl = false;

    private static final int kEncCodesPerRev = 100;
    private static final double kPosP = 0.05;
    private static final double kPosI = 0;
    private static final double kPosD = 0.00001;

    private static final byte syncGroup = 0x20;

    public RobotArm(CANJaguar leftArmJag, CANJaguar rightArmJag, boolean usePositionControl){
        m_leftJag = leftArmJag;
        m_rightJag = rightArmJag;
        kUsePositionControl = usePositionControl;

        configureCAN(m_leftJag);
        configureCAN(m_rightJag);
    }

    public void set(double v){
        try {
            m_leftJag.set(v, syncGroup);
            m_rightJag.set(v, syncGroup);
            CANJaguar.updateSyncGroup(syncGroup);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    private void configureCAN(CANJaguar cjag){
        try {
            if (kUsePositionControl) {
                cjag.changeControlMode(CANJaguar.ControlMode.kPosition);
                cjag.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
                cjag.configEncoderCodesPerRev(kEncCodesPerRev);
                cjag.setPID(kPosP, kPosI, kPosD);
            } else {
                cjag.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            }
            cjag.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            cjag.enableControl();
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
}
