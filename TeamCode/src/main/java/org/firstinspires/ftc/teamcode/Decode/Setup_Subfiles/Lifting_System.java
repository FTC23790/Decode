package org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Motor_Servo_Setup.Complex_Motor_Setup;
public class Lifting_System {
    boolean L_Bump;
    boolean R_Bump;
    int Lift_Hight;
    Complex_Motor_Setup RS = new Complex_Motor_Setup();
    // Right Slide motor objcet
    //Complex_Motor_Setup LS = new Complex_Motor_Setup();
    // Left slide motor object
    public void Lift_MotorCal (HardwareMap hardwareMap) {
        RS.init(hardwareMap, "CH3RightSlide");
        //LS.init(hardwareMap, "String-litleral-motor-name");
        RS.setDirection(DcMotorSimple.Direction.FORWARD);
        //LS.setDirection(DcMotorSimple.Direction.FORWARD);
        //setup block
    }

    public void Lift_Grabber (boolean Left_Bumper, boolean Right_Bumper) {
        L_Bump = Left_Bumper;
        R_Bump = Right_Bumper;
    }

    public void RightLiftSys () {
        if (L_Bump == true) {
            RS.setMotorSpeed(+1);
            RS.motorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift_Hight = RS.CurrentPos();
            //exted
        } else if (R_Bump == true) {
            RS.setMotorSpeed(-1);
            RS.motorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift_Hight = RS.CurrentPos();
            //retract
        } else {
            RS.TargetPos(Lift_Hight);
            RS.motorMode(DcMotor.RunMode.RUN_TO_POSITION);
            // hold pos
        }
    }

    public int RSPrint () {
        return RS.CurrentPos();
    }

    public void RSTarget (int numb) {
        RS.setMotorSpeed(-1);
        RS.TargetPos(numb);
        Lift_Hight = numb;
    }
}
