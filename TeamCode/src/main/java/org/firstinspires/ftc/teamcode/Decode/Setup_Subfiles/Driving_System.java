package org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Motor_Servo_Setup.Drive_Motor_Setup;



public class Driving_System {
    private double MathX;
    private double DriveX;
    private double MathY;
    private double DriveY;
    private double DriveR;
    private double MathIMUY;
    private double Theta;
    private double NTheta;
    private double rad;
    private double robotAngle;
    private double Power;
    private double RawFR;
    private double RawBR;
    private double RawFL;
    private double RawBL;
    private double MaxRaw;
    private double FR;
    private double BR;
    private double FL;
    private double BL;
    Drive_Motor_Setup CH0FR = new Drive_Motor_Setup();
    // FR motor object
    Drive_Motor_Setup CH1BR = new Drive_Motor_Setup();
    // BR motor object
    Drive_Motor_Setup EH0FL = new Drive_Motor_Setup();
    // FL motor object
    Drive_Motor_Setup EH1BL = new Drive_Motor_Setup();
    // FR motor object

        //varible asighment
    public void Drive_MotorCal (HardwareMap hardwareMap) {
        CH0FR.init(hardwareMap, "CH0FrontRight");
        CH1BR.init(hardwareMap, "CH1BackRight");
        EH0FL.init(hardwareMap, "EH0FrontLeft");
        EH1BL.init(hardwareMap, "EH1BackLeft");
        CH0FR.setDirection(DcMotorSimple.Direction.FORWARD);
        CH1BR.setDirection(DcMotorSimple.Direction.REVERSE);
        EH0FL.setDirection(DcMotorSimple.Direction.REVERSE);
        EH1BL.setDirection(DcMotorSimple.Direction.FORWARD);
        //setup block
    }

    public void Drive_Grabber (double StickX, double StickY, double StickR, double PowerMod, double IMUY) {
        MathX = StickX;
        MathY = StickY;
        DriveR = StickR;
        MathIMUY = IMUY;
        Power = PowerMod;
        // feed in info from opmode
    }
    private void Drive_Maths () {
        robotAngle = MathIMUY;
        Theta = Math.atan2(MathY, MathX);
        rad = Math.hypot(MathY, MathX);
        NTheta = AngleUnit.normalizeRadians(Theta - robotAngle);
        DriveY = rad * Math.sin(NTheta);
        DriveX = rad * Math.cos(NTheta);
        // field centraic moduel

        RawFR = +1 * (DriveY - DriveX - DriveR);
        RawBR = -1 * (DriveY + DriveX - DriveR);
        RawFL = +1 * (DriveY + DriveX + DriveR);
        RawBL = -1 * (DriveY - DriveX + DriveR);
        MaxRaw = 1;
        //Dircetion matrix

        if (MaxRaw < Math.abs(RawFR)) {
            MaxRaw = Math.abs(RawFR);
        }
        if (MaxRaw < Math.abs(RawBR)) {
            MaxRaw = Math.abs(RawBR);
        }
        if (MaxRaw < Math.abs(RawFL)) {
            MaxRaw = Math.abs(RawFL);
        }
        if (MaxRaw < Math.abs(RawBL)) {
            MaxRaw = Math.abs(RawBL);
        }
        //speed matrix
    }

    public void Drive_Running () {
        Drive_Maths();
        FR = ((RawFR / MaxRaw) * Power);
        BR = ((RawBR / MaxRaw) * Power);
        FL = ((RawFL / MaxRaw) * Power);
        BL = ((RawBL / MaxRaw) * Power);
        CH0FR.setMotorSpeed(FR);
        CH1BR.setMotorSpeed(BR);
        EH0FL.setMotorSpeed(FL);
        EH1BL.setMotorSpeed(BL);
    }
    public double RAPrint () {
        return robotAngle;
    }

    public void Move_Forward () {
        CH0FR.setMotorSpeed(-0.7);
        CH1BR.setMotorSpeed(+0.7);
        EH0FL.setMotorSpeed(-0.7);
        EH1BL.setMotorSpeed(+0.7);
    }
    public void Stop () {
        CH0FR.setMotorSpeed(0);
        CH1BR.setMotorSpeed(0);
        EH0FL.setMotorSpeed(0);
        EH1BL.setMotorSpeed(0);
    }
}
