package org.firstinspires.ftc.teamcode.Decode.SubSystems;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Drive_Motor_Setup;
public class Scoring_System {
    Drive_Motor_Setup CH2IN = new Drive_Motor_Setup();
    Drive_Motor_Setup EH2OUT = new Drive_Motor_Setup();
    private boolean InTake_Button;
    private boolean OutTake_Button;
    private boolean Pusher_Button; // LINE ADDED BY NICK FOR TESTING
    private double Left_Bump;
    private double Right_Bump;
    private double Outtake_RPM;
    public void Score_MotorCal (HardwareMap hardwareMap) {
        CH2IN.init(hardwareMap, "CH2Intake");
        EH2OUT.init(hardwareMap, "EH2OutTake");
        CH2IN.setDirection(DcMotorSimple.Direction.FORWARD);
        EH2OUT.setDirection(DcMotorSimple.Direction.REVERSE);
        // setup
    }

    public void Scoring_Grabber (boolean Inbutton, boolean Outbutton, double OutRPM) {
        InTake_Button = Inbutton;
        OutTake_Button = Outbutton;
        Outtake_RPM = OutRPM;
    }

    public void Scoring_Running () {
        if (InTake_Button == true) {
            CH2IN.setMotorSpeed(1);
        } else {
            CH2IN.setMotorSpeed(0);
        }
        if (OutTake_Button == true) {
            EH2OUT.setMotorSpeed(Outtake_RPM);
        } else {
            EH2OUT.setMotorSpeed(0);
        }
    }
    public int Out_Return () {
        return EH2OUT.CurrentPos();
    }

    // BLOCK ADDED BY NICK FOR TESTING
    CRServo PusherServo;
    public void Servo_Init_TEMPORARY(HardwareMap hwmap) {
        PusherServo = hwmap.get(CRServo.class, "ball pusher");
        PusherServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void Servo_Testing_TEMPORARY(boolean servoButton) {
        if (servoButton == true) {
            PusherServo.setPower(1);
        } else {
            PusherServo.setPower(0);
        }
    }
    // BLOCK ADDED BY NICK FOR TESTING

}
