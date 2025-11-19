package org.firstinspires.ftc.teamcode.Decode.SubSystems;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Drive_Motor_Setup;
import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Cont_Servo_Setup;
public class Scoring_System {
    Cont_Servo_Setup CH0IN = new Cont_Servo_Setup();
    // change object name baseded on port
    Drive_Motor_Setup EH2OUT = new Drive_Motor_Setup();
    Cont_Servo_Setup EHS0Pass = new Cont_Servo_Setup();
    private double InTake;
    private boolean OutTake_Button;
    private double PassThrough;
    private double Outtake_RPM;
    public void Score_MotorCal (HardwareMap hardwareMap) {
        CH0IN.init(hardwareMap,"CHS0Intake");
        EH2OUT.init(hardwareMap,"EH2OutTake");
        EHS0Pass.init(hardwareMap,"EHS0PassThrough");
        CH0IN.setDirection(DcMotorSimple.Direction.FORWARD);
        EH2OUT.setDirection(DcMotorSimple.Direction.REVERSE);
        EHS0Pass.setDirection(DcMotorSimple.Direction.FORWARD);
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
        EHS0Pass.setPower(PassThrough);
    }
    public int Out_Return () {
        return EH2OUT.CurrentPos();
    }

}
