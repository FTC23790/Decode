package org.firstinspires.ftc.teamcode.Decode.SubSystems;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Drive_Motor_Setup;
import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Servo_Setup;
public class Scoring_System {
    Drive_Motor_Setup CH2IN = new Drive_Motor_Setup();
    Drive_Motor_Setup EH2OUT = new Drive_Motor_Setup();
    Servo_Setup EH1S = new Servo_Setup();
    Servo_Setup CH0S = new Servo_Setup();
    private double InTake;
    private boolean OutTake_Button;
    private double PassThrough;
    private double Outtake_RPM;
    public void Score_MotorCal (HardwareMap hardwareMap) {
        CH2IN.init(hardwareMap, "CH2Intake");
        EH2OUT.init(hardwareMap, "EH2OutTake");
        EH1S.init(hardwareMap, "RBP");
        CH0S.init(hardwareMap, "LBP");
        CH2IN.setDirection(DcMotorSimple.Direction.FORWARD);
        EH2OUT.setDirection(DcMotorSimple.Direction.REVERSE);
        EH1S.setDirection(DcMotorSimple.Direction.FORWARD);
        // setup
    }

    public void Scoring_Grabber (double In, boolean Outbutton, double OutRPM, double Pass) {
        InTake = In;
        OutTake_Button = Outbutton;
        Outtake_RPM = OutRPM;
        PassThrough = Pass;
    }

    public void Scoring_Running () {
        if (OutTake_Button == true) {
            EH2OUT.setMotorSpeed(Outtake_RPM);
        } else {
            EH2OUT.setMotorSpeed(0);
        }
        CH2IN.setMotorSpeed(InTake);
        EH1S.setPower(PassThrough);
        CH0S.setPower(PassThrough);
    }
    public int Out_Return () {
        return EH2OUT.CurrentPos();
    }

}
