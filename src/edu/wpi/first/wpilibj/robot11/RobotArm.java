/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.robot11;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author zach
 */
public class RobotArm {

    public class Target {
        private static final int kGround = 0;
        private static final int kLow = 1;
        private static final int kFeeder = 2;
        private static final int kMid = 3;
        private static final int kHigh = 4;

        private static final boolean kFront = true;
        private static final boolean kRear = false;

        private static final boolean kRaised = true;
        private static final boolean kLowered = false;

        private static final boolean kCenter = true;
        private static final boolean kEnd = false;
        
        private int m_vert;
        private boolean m_front;
        private boolean m_raised;
        private boolean m_center;

        // Position of motor to reach targets
        // Ground, Low, Feeder, Mid, High
        // Sub: Front, Front Raised, Back, Back Raised piston
        // Sub-Sub: End, Center
        public static final double kImpH = -999;
        private final double kHeights[][][] = { //TODO *** 30 measurements!
            // Fr     FrC     FrRa    FrRaC    Bk     BkC      BkRa   BkRaC
            { {0.000, 0.000}, {kImpH, kImpH}, {1.000, 1.000}, {kImpH, kImpH} }, // Ground
            { {2.000, 3.000}, {4.000, 5.000}, {6.000, 7.000}, {8.000, 9.000} }, // Low
            { {10.00, 10.00}, {11.00, 11.00}, {12.00, 12.00}, {13.00, 13.00} }, // Feeder
            { {14.00, 15.00}, {16.00, 17.00}, {18.00, 19.00}, {20.00, 21.00} }, // Mid
            { {22.00, 23.00}, {24.00, 25.00}, {26.00, 27.00}, {28.00, 29.00} }, // High
        };

        public Target(){

        }

        public Target(int v, boolean f, boolean r, boolean c){
            set(v, f, r, c);
        }

        public void set(int v, boolean f, boolean r, boolean c){
            m_vert = v;
            m_front = f;
            m_raised = r;
            m_center = c;
        }

        public void setCenter(boolean m_center) {
            this.m_center = m_center;
        }

        public void setFront(boolean m_front) {
            this.m_front = m_front;
        }

        public void setRaised(boolean m_raised) {
            this.m_raised = m_raised;
        }

        public void setVert(int m_vert) {
            this.m_vert = m_vert;
        }

        public boolean isCenter() {
            return m_center;
        }

        public boolean isFront() {
            return m_front;
        }

        public boolean isRaised() {
            return m_raised;
        }

        public void swapPiston(){
            m_raised = !m_raised;
        }

        public int getVert() {
            return m_vert;
        }

        public double getHeight(){
            int index2, index3;
            index2 = (m_front == kFront ? 0 : 2) + (m_raised == kRaised ? 1 : 0);
            index3 = (m_center == kCenter ? 1 : 0);
            return kHeights[m_vert][index2][index3];
        }
    }

    public CANJaguar m_leftJag;
    public CANJaguar m_rightJag;

    private Solenoid m_solenoid;

    private AnalogChannel m_pot;
    private Encoder m_enc;
    private DigitalInput m_indexSw;

    private boolean kUsePositionControl = false;

    private static final int kEncCodesPerRev = 100;
    private static final double kPosPIDInvert = 1; // +/- 1
    private static final double kPosP = 0.900 * kPosPIDInvert;
    private static final double kPosI = 0.001 * kPosPIDInvert;
    private static final double kPosD = 0.005 * kPosPIDInvert;

    private static final byte syncGroup = 0x20;

    
    private Target m_target ;
    
    public RobotArm(CANJaguar leftArmJag, CANJaguar rightArmJag, boolean usePositionControl){
        m_leftJag = leftArmJag;
        m_rightJag = rightArmJag;
        kUsePositionControl = usePositionControl;

        configureCAN(m_leftJag);
        configureCAN(m_rightJag);
    }
    public boolean setTarget(Target t){
        Target lazyT;
        double height;
        lazyT = new Target(t.getVert(), t.isFront(), 
                (m_solenoid.get() ? Target.kRaised : Target.kLowered),
                t.isCenter());
        height = lazyT.getHeight();
        if(height == Target.kImpH){
            lazyT.swapPiston();
            height = lazyT.getHeight();
            if(height == Target.kImpH){
                return true;
            }
            m_solenoid.set(!m_solenoid.get());
        }
        m_target = lazyT;
        if(kUsePositionControl){
            set(height);
        }
        return false;
    }
    public void setRaw(double v){
        try {
            m_leftJag.set(v, syncGroup);
            m_rightJag.set(v, syncGroup);
            CANJaguar.updateSyncGroup(syncGroup);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
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
