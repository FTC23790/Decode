package org.firstinspires.ftc.teamcode.Decode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Drive_Motor_Setup;

public class Scrapbin_DriveTrain {

    Drive_Motor_Setup FR0 = new Drive_Motor_Setup();
    Drive_Motor_Setup FL1 = new Drive_Motor_Setup();
    Drive_Motor_Setup BR2 = new Drive_Motor_Setup();
    Drive_Motor_Setup BL3 = new Drive_Motor_Setup();

    double Left;
    double Right;
    double Power;

    public void Bin_Drive_MotorCal (HardwareMap hardwareMap) {
        FR0.init(hardwareMap, "String-littleral-name");
        FL1.init(hardwareMap, "String-littleral-name");
        BR2.init(hardwareMap, "String-littleral-name");
        BL3.init(hardwareMap, "String-littleral-name");
        FR0.setDirection(DcMotorSimple.Direction.FORWARD);
        FL1.setDirection(DcMotorSimple.Direction.FORWARD);
        BR2.setDirection(DcMotorSimple.Direction.FORWARD);
        BL3.setDirection(DcMotorSimple.Direction.FORWARD);
        //setup block
    }
    public void Bin_Drive_Grabber (double left_side, double right_side, double PowerMod) {
        Left = left_side;
        Right = right_side;
        Power = PowerMod;
        // feed in info from opmode
    }
    //public void Bin_Drive_Running () {
        //FR0.setMotorSpeed(Left);
        //FL1.setMotorSpeed(Left);
        //BR2.setMotorSpeed(Right);
        //BL3.setMotorSpeed(Right);
    //}
}
