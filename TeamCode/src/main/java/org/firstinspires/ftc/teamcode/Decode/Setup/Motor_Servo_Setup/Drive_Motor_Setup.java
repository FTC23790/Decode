package org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive_Motor_Setup {

    private DcMotor motor;
    public void init(HardwareMap hwMap, String motorID) {
        motor = hwMap.get(DcMotor.class, motorID);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setMotorSpeed(double speed){
        motor.setPower(speed);
    }
    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }
    public int CurrentPos(){
        return motor.getCurrentPosition();
    }
}
