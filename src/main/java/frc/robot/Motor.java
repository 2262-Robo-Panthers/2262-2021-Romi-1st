package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;

public class Motor {
    private Spark motorDriver; // the motor driver for this motor
    private EncoderEx encoder; // the encoder for this motor(only required in certin runModes)

    private PIDController motorPIDController = new PIDController(0, 0, 0);
    private PIDCoefficients runToPositionPID = new PIDCoefficients(0, 0, 0);
    private PIDCoefficients runUsingEncoderPID = new PIDCoefficients(0, 0, 0);

    //target
    private int targetPos;
    private double targetPow;

    //encoder
    private int posOffset;
    private float maxEncoderSpeed = 1000;

    //other
    private boolean SettingChanged = false;
    private RunMode currentRunMode = RunMode.RUN_WITHOUT_ENCODER;
    private Direction currentDirection = Direction.FORWARD;

    public Motor(int motorPort, Integer encoderPortA, Integer encoderPortB)
    {
        setMotorDriver(motorPort);
        setEncoder(encoderPortA, encoderPortB);
    }

    public Motor(int motorPort, Integer encoderPortA)
    {
        setMotorDriver(motorPort);
        setEncoder(encoderPortA, null);
    }


    /////////////////////////////
    //setter and getter methods//
    /////////////////////////////
    
    public void setMotorDriver(Spark motorDriver){
        if(motorDriver != null){
            this.motorDriver = motorDriver; 
            SettingChanged = true;
        }
    }//sets the spark motor driver for this motor

    public void setMotorDriver(int port){motorDriver = new Spark(port); SettingChanged = true;} //sets the motor driver to a new spark with the new port
    
    public Spark getMotorDriver(){return motorDriver;}//gets the spark motor driver for this motor
   
    public void setEncoder(EncoderEx encoder){this.encoder = encoder; SettingChanged = true;}//sets the encoder

    public void setEncoder(Integer port1, Integer port2){
        if(port1 != null)
        {
            if(port2 == null) port2 = port1 + 1;
            encoder = new EncoderEx(port1, port2);
            SettingChanged = true;
        }
    }//sets encoder based on the ports

    public Encoder getEncoder(){return encoder;}//gets the current encoder

    public void setPID(RunMode runMode, PIDCoefficients PIDs){
        if(PIDs != null){
            SettingChanged = true;
            if(runMode == RunMode.RUN_TO_POSITON) runToPositionPID = PIDs;
            else if(runMode == RunMode.RUN_WITH_ENCODER) runUsingEncoderPID = PIDs;
            else SettingChanged = false;
        }
    }//sets the pid for a certin run mode(can be null!)

    public PIDCoefficients getPID(RunMode runMode){
        if(runMode == RunMode.RUN_TO_POSITON) return runToPositionPID;
        else if(runMode == RunMode.RUN_WITH_ENCODER) return runUsingEncoderPID;
        else {return null;}
    }//gets the pid for a certin run mode(can be null!)

    public void setDirection(Direction direction){
        if(direction != null){
            currentDirection = direction;
            SettingChanged = true;
        }
    }//sets the direction of the motor
    
    public Direction getDirection(){
        return currentDirection;
    }//gets the direction of the motor

    public void setMotorPower(double power){
        targetPow = Util.clamp((float)power, 1);
        SettingChanged = true;
    }//sets the speed of the motor in range of -1 to 1

    public double getMotorTargetPower(){
        return motorDriver.get();
    }//gets the set speed of the motor in range of -1 to 1

    public double getMotorPower()
    {
        if(encoder != null) return encoder.getVelocity()/maxEncoderSpeed;
        return 0;
    }//gets the speed of the motor based on the encoder speed(returns 0 if no encoder)

    public void setRunMode(RunMode runMode)
    {
        if(runMode != null){
            if(runMode == RunMode.RUN_WITHOUT_ENCODER || encoder != null){
                currentRunMode = runMode;
                SettingChanged = true;
            }
        }
    }//sets the current run mode
    
    public void setTargetPos(int pos){targetPos = pos; SettingChanged = true;}

    public int getTargetPos(){return targetPos;}

    public int getCurrentPos(){
        if(encoder != null){return encoder.get() + posOffset;}
        return 0;
    }

    public void setCurrentPos(int pos){
        if(encoder != null){
            posOffset = -encoder.get() + pos; SettingChanged = true;
        }
    }

    public void resetEncoder(){setCurrentPos(0);}

    public int getPosOffset() {return posOffset;}

    public void setPosOffset(int posOffset) {this.posOffset = posOffset; SettingChanged = true;}

    public float getMaxEncoderSpeed() {return maxEncoderSpeed;}

    public void setMaxEncoderSpeed(float maxEncoderSpeed) {this.maxEncoderSpeed = maxEncoderSpeed; SettingChanged = true;}
   
    private void updateSettings()
    {
        //direction
        if(currentDirection == Direction.FORWARD){
            motorDriver.setInverted(false);
            if(encoder != null) encoder.setReverseDirection(false);
        }
        else{
            motorDriver.setInverted(true);
            if(encoder != null) encoder.setReverseDirection(true);
        }

        //other
        if(currentRunMode == RunMode.RUN_WITHOUT_ENCODER){
            motorDriver.set(targetPow);
        }
        else if(currentRunMode == RunMode.RUN_WITH_ENCODER){
            motorPIDController.setPID(runUsingEncoderPID.p, runUsingEncoderPID.i, runUsingEncoderPID.d);
            motorPIDController.setSetpoint(targetPow*maxEncoderSpeed);
            motorPIDController.setIntegratorRange(-targetPow, targetPow);
        }
        else if(currentRunMode == RunMode.RUN_TO_POSITON){
            motorPIDController.setPID(runToPositionPID.p, runToPositionPID.i, runToPositionPID.d);
            motorPIDController.setSetpoint(targetPos);
            motorPIDController.setIntegratorRange(-targetPow, targetPow);
        }
    } //updtaes the settings in all other objects
    
    //////////////
    //run method//
    //////////////
    public void runMotor()
    { 
        if(SettingChanged){
            updateSettings();
            SettingChanged = false;
        }

        if(currentRunMode == RunMode.RUN_WITH_ENCODER){
            if(encoder != null){ 
                encoder.updateVelocity();
                motorDriver.set(motorPIDController.calculate(encoder.getVelocity()));
            }
        }
        else if(currentRunMode == RunMode.RUN_TO_POSITON){
            if(encoder != null) motorDriver.set(Util.clamp((float)motorPIDController.calculate(encoder.get()), (float)Math.abs(targetPow)));
        }
    }
}//runs the motor

/*
public class Motor {
    private Spark motorDriver; // the motor driver for this motor
    private Encoder encoder; // the encoder for this motor(only required in certin runModes)

    private PIDController motorPIDController = new PIDController(0, 0, 0);
    private PIDCoefficients runToPositionPID = new PIDCoefficients(0, 0, 0);
    private PIDCoefficients runUsingEncoderPID = new PIDCoefficients(0, 0, 0);

    private int targetPos;
    private int posOffset;

    private int targetPow;

    private int maxEncoderSpeed = 1000;
    private boolean SettingChanged = false;
    private RunMode currentRunMode = RunMode.RUN_WITHOUT_ENCODER;

    public Motor(int motorPort, Integer encoderPortA, Integer encoderPortB)
    {
        setMotorDriver(motorPort);
        setEncoder(encoderPortA, encoderPortB);
    }

    public Motor(int motorPort, Integer encoderPortA)
    {
        setMotorDriver(motorPort);
        setEncoder(encoderPortA, null);
    }


    /////////////////////////////
    //setter and getter methods//
    /////////////////////////////
    //sets the spark motor driver for this motor
    public void setMotorDriver(Spark motorDriver){this.motorDriver = motorDriver;}

    //gets the spark motor driver for this motor
    public Spark getMotorDriver(){return motorDriver;}

    //sets the motor driver to a new spark with the new port
    public void setMotorDriver(int port){motorDriver = new Spark(port);}

    //gets the motor driver port if a driver is specified else returns -1
    public int getMotorDriverPort()
    {
        if(motorDriver != null){return motorDriver.getChannel();}
        else return -1;
    }

    public void setEncoder(Encoder encoder){this.encoder = encoder;}

    public void setEncoder(Integer port1, Integer port2)
    {
        if(port1 != null)
        {
            if(port2 == null) port2 = port1 + 1;
            encoder = new Encoder(port1, port2);
        }
    }

    public Encoder getEncoder(){return encoder;}

    //sets the pid for a certin run mode(can be null!)
    public void setPID(RunMode runMode, PIDCoefficients PIDs)
    {
        if(runMode == RunMode.RUN_TO_POSITON) runToPositionPID = PIDs;
        else if(runMode == RunMode.RUN_WITH_ENCODER) runUsingEncoderPID = PIDs;
        else SmartDashboard.putString("error while running Motor.setPID: ", "the runMode specified does not have a pid value associated with it!");
    }

    //gets the pid for a certin run mode(can be null!)
    public PIDCoefficients getPID(RunMode runMode)
    {
        if(runMode == RunMode.RUN_TO_POSITON) return runToPositionPID;
        else if(runMode == RunMode.RUN_WITH_ENCODER) return runUsingEncoderPID;
        else {return null;}
    }

    //sets the direction of the motor
    public void setDirection(Direction direction)
    {
        if(motorDriver != null && encoder != null){
            if(direction == Direction.FORWARD) {
                motorDriver.setInverted(false);
                encoder.setReverseDirection(false);
            }
            else if(direction == Direction.REVERCE){
                motorDriver.setInverted(true);
                encoder.setReverseDirection(true);
            }
            else SmartDashboard.putString("error while running Motor.setDirection: ", "the Direction used is not valid!");
        }
        else SmartDashboard.putString("error while running Motor.setDirection: ", "no motor driver");
    }

    //gets the direction of the motor
    public Direction getDirection()
    {
        if(motorDriver != null){
            if(motorDriver.getInverted()) return Direction.REVERCE;
            return Direction.FORWARD;
        }
        else {return null;}
    }

    //sets the speed of the motor in range of -1 to 1
    public void setMotorPower(double power)
    {
        if(motorDriver != null) motorDriver.set(MathUtil.clamp(power, -1, 1));
        else SmartDashboard.putString("error while running Motor.setMotorPower: ", "no motor driver");
    }

    //sets the speed of the motor in range of -1 to 1
    public double getMotorPower()
    {
       if(motorDriver != null) return motorDriver.get();
       else return 0;
    }

    //sets the current run mode and the pid values in the controller
    public void setRunMode(RunMode runMode)
    {
        if(runMode == RunMode.RUN_WITHOUT_ENCODER) currentRunMode = runMode;
        else if(runMode == RunMode.RUN_WITH_ENCODER){
            if(encoder != null){
                if(runUsingEncoderPID != null){
                    motorPIDController.setPID(runUsingEncoderPID.p, runUsingEncoderPID.i, runUsingEncoderPID.d);
                    currentRunMode = runMode;
                }
                else SmartDashboard.putString("error while running Motor.setRunMode: ", "the PID values for runUsingEncoderPID were not defined!");
            }
            else SmartDashboard.putString("error while running Motor.setRunMode: ", "the encoder is not set so mode can not be changed");
        }
        else if(runMode == RunMode.RUN_TO_POSITON){
            if(encoder != null){
                if(runToPositionPID != null){
                    motorPIDController.setPID(runToPositionPID.p, runToPositionPID.i, runToPositionPID.d);
                    currentRunMode = runMode;
                }
                else SmartDashboard.putString("error while running Motor.setRunMode: ", "the PID values for runToPositionPID were not defined!");
            }
            else SmartDashboard.putString("error while running Motor.setRunMode: ", "the encoder is not set so mode can not be changed");
        }
        else SmartDashboard.putString("error while running Motor.setRunMode: ", "the runMode was not specified!");
    }
    
    public void setTargetPos(int pos){targetPos = pos;}

    public int getTargetPos(){return targetPos;}

    public int getCurrentPos(){
        if(encoder != null) return encoder.get() + posOffset;
        else{
            SmartDashboard.putString("error while running Motor.getCurrentPos: ", "the encoder is not set so position can not be gotten");
            return 0;
        }
    }

    public void setCurrentPos(int pos){
        if(encoder != null) posOffset = -encoder.get() + pos;
        else SmartDashboard.putString("error while running Motor.setCurrentPos: ", "the encoder is not set so current position can not be set");
    }

    public void resetEncoder(){setCurrentPos(0);}

    public int getPosOffset() {
        return posOffset;
    }

    public void setPosOffset(int posOffset) {
        this.posOffset = posOffset;
    }

    public int getMaxEncoderSpeed() {
        return maxEncoderSpeed;
    }

    public void setMaxEncoderSpeed(int maxEncoderSpeed) {
        this.maxEncoderSpeed = maxEncoderSpeed;
    }
   
   
    //////////////
    //run method//
    //////////////
    //runs the motor
    public void runMotor()
    {
        if(currentRunMode == RunMode.RUN_WITH_ENCODER){
            motorPIDController.calculate(encoder.get(), targetPow*maxEncoderSpeed);
        }
        else if(currentRunMode == RunMode.RUN_TO_POSITON){}
        else if(currentRunMode != RunMode.RUN_WITHOUT_ENCODER) SmartDashboard.putString("error while running Motor.runMotor: ", "the runMode was not specified!");
    }


}
*/

class MotorController extends Thread{
    List<Motor> motors = new ArrayList<>();

    public MotorController(){}

    public void addMotor(int motorPort, Integer encoderPortA, Integer encoderPortB){
        motors.add(new Motor(motorPort, encoderPortA,encoderPortB));
    }

    public void addMotor(int motorPort, Integer encoderPortA){
        motors.add(new Motor(motorPort, encoderPortA));
    }

    public void addMotor(Motor motor){if(motor != null) motors.add(motor);}

    public void Run()
    {
        while(!this.isInterrupted()){for(Motor m:motors) m.runMotor();}
    }

    public void Stop(){this.interrupt();}
}

class EncoderEx extends Encoder
{
    private int lastEncoderPos = 0;
    private long lastReadTime = System.currentTimeMillis();
    private float velocity = 0;

    public EncoderEx(final int channelA, final int channelB, boolean reverseDirection) {
        super(channelA, channelB, reverseDirection);
    }

    public EncoderEx(final int channelA, final int channelB) {
        super(channelA, channelB);
    }

    public EncoderEx(final int channelA, final int channelB, boolean reverseDirection, final EncodingType encodingType) {
        super(channelA, channelB, reverseDirection, encodingType);
    }

    public EncoderEx(final int channelA, final int channelB, final int indexChannel, boolean reverseDirection) {
        super(channelA, channelB, indexChannel, reverseDirection);
    }

    public EncoderEx(final int channelA, final int channelB, final int indexChannel) {
        super(channelA, channelB, indexChannel);
    }

    public EncoderEx(DigitalSource sourceA, DigitalSource sourceB, boolean reverseDirection) {
        super(sourceA, sourceB, reverseDirection);
    }

    public EncoderEx(DigitalSource sourceA, DigitalSource sourceB) {
        super(sourceA, sourceB);
    }

    public EncoderEx(DigitalSource sourceA, DigitalSource sourceB, boolean reverseDirection, final EncodingType encodingType) {
        super(sourceA, sourceB, reverseDirection, encodingType);
    }

    public EncoderEx(DigitalSource sourceA, DigitalSource sourceB, DigitalSource indexSource, boolean reverseDirection) {
        super(sourceA, sourceB, indexSource, reverseDirection);
    }

    public EncoderEx(DigitalSource sourceA, DigitalSource sourceB, DigitalSource indexSource) {
        super(sourceA, sourceB, indexSource);
    }

    @Override
    public int get()
    {
        lastEncoderPos = super.get();
        lastReadTime = System.nanoTime();
        return lastEncoderPos;
    } 

    public void updateVelocity()
    {
        velocity = ((super.get() - lastEncoderPos) / (System.nanoTime() - lastReadTime))*1000000000;
        get();
    }//returns the current velocity in ticks per second

    public float getVelocity(){return velocity;}

    public float updateAndGetVelocity(){
        updateVelocity();
        return getVelocity();
    }
}

enum RunMode
{
    RUN_WITH_ENCODER,
    RUN_WITHOUT_ENCODER,
    RUN_TO_POSITON
}

enum Direction
{
    FORWARD,
    REVERCE
}