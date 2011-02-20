/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.robot11;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
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

    private boolean kUseCRioPositionControl = true;
    private boolean kUseJagPositionControl = false;

    private static final double kArmScale = 100;
    private static final int kEncCodesPerRev = 40;
    private static final double kPosPIDInvert = -1; // +/- 1
    private static final double kPosP = 0.300 * kPosPIDInvert;
    private static final double kPosI = 0.003 * kPosPIDInvert;
    private static final double kPosD = 0.050 * kPosPIDInvert;

    private static final byte syncGroup = 0x20;

    private double m_setpoint;
    private double m_offset = 0.0;
    private PIDController m_pid;
    
    private Target m_target ;
    
    public RobotArm(CANJaguar leftArmJag, CANJaguar rightArmJag, boolean usePositionControl){
        m_leftJag = leftArmJag;
        m_rightJag = rightArmJag;
        kUseJagPositionControl = usePositionControl;
        kUseCRioPositionControl = false;

        configureCAN(m_leftJag);
        configureCAN(m_rightJag);
    }
    public RobotArm(CANJaguar leftArmJag, CANJaguar rightArmJag, Encoder armEncoder, boolean usePositionControl){
        m_leftJag = leftArmJag;
        m_rightJag = rightArmJag;

        m_enc = armEncoder;
        kUseJagPositionControl = false;
        kUseCRioPositionControl = usePositionControl;
        if(kUseCRioPositionControl){
            m_pid = new PIDController(kPosP, kPosI, kPosD, m_enc, new PIDOutput(){
                public void pidWrite(double output){
                    setRaw(scaleOutput(output, getArmAngle()));
                    
                }
            });
            m_pid.setInputRange(-100, 200);
            m_pid.setOutputRange(-1.0, 1.0);
            m_pid.setTolerance(5);
            
        }
    }
    public double scaleOutput(double v, double angle){
        v *= 1;
        double force = Math.cos(angle);
        double kForceScale = 0.2;
        /*
        if(v < 0){
            if(force < 0){
                v /= (-force + 1);
            }else{
                v /= (force + 1);
            }
        }else{
            if(force > 0){
                v /= (force + 1);
            }else{
                v /= (-force + 1);
            }
        }*/
        v += force * kForceScale;
        return v;
    }
    public double getArmAngle(){
        return m_enc.getDistance() - Math.PI / 2;
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
        /*if(kUsePositionControl){
            setRaw(height);
        }*/
        set(height);
        return false;
    }
    public void setRaw(double v){
        try {
            m_leftJag.setX(v, syncGroup);
            m_rightJag.setX(v, syncGroup);
            CANJaguar.updateSyncGroup(syncGroup);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
    public void setOffset(double v){
        m_offset = v;
    }
    public double getOffset(){
        return  m_offset;
    }
    public void set(double v){
        try {
            if(kUseJagPositionControl){
                m_leftJag.setX(v * kArmScale, syncGroup);
                m_rightJag.setX(v * kArmScale, syncGroup);
                CANJaguar.updateSyncGroup(syncGroup);
            }else if(kUseCRioPositionControl){
                if(m_setpoint != v + m_offset){
                    m_setpoint = v + m_offset;
                    m_pid.setSetpoint(m_setpoint);
                    m_pid.enable();
                }
            }else{
                setRaw(v);
            }
            
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
    public void poll(){
        //TODO: Thread
        if(kUseJagPositionControl){

        }else if(kUseCRioPositionControl){
            if(m_pid.onTarget()){
                m_pid.disable();
            }
        }
    }
    public double get(){
        try {
            if(kUseJagPositionControl){
                return m_leftJag.getPosition();
            }else if(kUseCRioPositionControl){
                return m_setpoint;
            }else{
                return m_leftJag.getX();
            }
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
            return 0;
        }
    }
    public double getSetpoint(){
        try {
            if(kUseCRioPositionControl){
                return m_setpoint;
            }else{
                return m_leftJag.getX();
            }
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
            return 0;
        }

    }

    private void configureCAN(CANJaguar cjag){
        try {
            
            if (kUseJagPositionControl) {
                cjag.changeControlMode(CANJaguar.ControlMode.kPosition);
                cjag.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
                cjag.configEncoderCodesPerRev(kEncCodesPerRev);
                //cjag.configMaxOutputVoltage(10);
                //cjag.setVoltageRampRate(1.0);
                cjag.setPID(kPosP, kPosI, kPosD);
                cjag.enableControl();
            } else {
             
                cjag.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            
            }

            cjag.configNeutralMode(CANJaguar.NeutralMode.kBrake);
            
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
    public boolean onTarget(){
        if(kUseJagPositionControl){
            return false;
        }else if(kUseCRioPositionControl){
            return m_pid.onTarget();
        }else{
            return false;
        }
    }
    public void setPID(double p, double i, double d){
        if(kUseJagPositionControl){
            try {
                m_leftJag.setPID(p, i, d);
                m_rightJag.setPID(p, i, d);
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }else if(kUseCRioPositionControl){
            m_pid.setPID(p, i, d);
        }

    }
}
