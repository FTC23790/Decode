package org.firstinspires.ftc.teamcode.Decode.SubSystems;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Drive_Motor_Setup;
import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Cont_Servo_Setup;
public class Scoring_System {
    Cont_Servo_Setup CH0IN = new Cont_Servo_Setup();
    // change object name baseded on port
    Drive_Motor_Setup EH2OUT = new Drive_Motor_Setup();
    //Cont_Servo_Setup EH1S = new Cont_Servo_Setup();
    //Cont_Servo_Setup CH0S = new Cont_Servo_Setup();
    private double InTake;
    private boolean OutTake_Button;
    private double PassThrough;
    private double Outtake_RPM;
    public void Score_MotorCal (HardwareMap hardwareMap) {
        CH0IN.init(hardwareMap, "CH0IntakeServo");
        EH2OUT.init(hardwareMap, "EH2OutTake");
        //EH1S.init(hardwareMap, "RBP");
        //CH0S.init(hardwareMap, "LBP");
        CH0IN.setDirection(DcMotorSimple.Direction.FORWARD);
        //unsure on direation of servo
        EH2OUT.setDirection(DcMotorSimple.Direction.REVERSE);
        //EH1S.setDirection(DcMotorSimple.Direction.FORWARD);
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
        CH0IN.setPower(InTake);
        //EH1S.setPower(PassThrough);
        //CH0S.setPower(PassThrough);
    }
    public int Out_Return () {
        return EH2OUT.CurrentPos();
    }

}
