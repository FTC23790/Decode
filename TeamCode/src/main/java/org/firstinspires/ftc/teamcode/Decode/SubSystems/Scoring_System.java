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
    private double OutTPS;
    private boolean InTake_Button;
    private boolean OutTake_Button;
    private double PassThrough;
    public void Score_MotorCal (HardwareMap hardwareMap) {
        CH0IN.init(hardwareMap,"CHS0Intake");
        EH2OUT.init(hardwareMap,"EH2OutTake");
        EHS0Pass.init(hardwareMap,"EHS0PassThrough");
        CH0IN.setDirection(DcMotorSimple.Direction.FORWARD);
        EH2OUT.setDirection(DcMotorSimple.Direction.REVERSE);
        EHS0Pass.setDirection(DcMotorSimple.Direction.FORWARD);
        // setup
        OutTPS = 2800;
        // 2,800 ticks per second for outtake motor
    }

    public void Scoring_Grabber (boolean In, boolean Outbutton, double Pass) {
        InTake_Button = In;
        OutTake_Button = Outbutton;
        PassThrough = Pass;
    }

    public void Scoring_Running () {
        if (OutTake_Button == true) {
            EH2OUT.setMotorVelocity(OutTPS * 0.4);
        } else {
            EH2OUT.setMotorVelocity(0);
        }
        if (InTake_Button == true) {
            CH0IN.setPower(1);
        } else {
            CH0IN.setPower(0);
        }
        EHS0Pass.setPower(PassThrough);
    }
    public int Out_Return () {
        return EH2OUT.CurrentPos();
    }

}
