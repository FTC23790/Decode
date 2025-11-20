package org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Complex_Motor_Setup {

    private DcMotor motor;
    private DcMotorEx motorEx;
    public void init(HardwareMap hwMap, String motorID) {
        motor = hwMap.get(DcMotor.class, motorID);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setMotorSpeed(double speed){
        motor.setPower(speed);
    }
    public void setMotorVelocity (double V) {
        motorEx.setVelocity(V);
    }
    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }
    public void motorMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }
    public void setPower(double power){
        motor.setPower(power);
    }
    public int CurrentPos(){
        return motor.getCurrentPosition();
    }
    public void TargetPos(int hight){
        motor.setTargetPosition(hight);
    }
}
