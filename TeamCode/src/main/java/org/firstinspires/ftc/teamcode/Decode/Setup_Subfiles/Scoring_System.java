package org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles;

import android.widget.Button;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Motor_Servo_Setup.Drive_Motor_Setup;
public class Scoring_System {
    Drive_Motor_Setup CH2IN = new Drive_Motor_Setup();
    Drive_Motor_Setup EH2OUT = new Drive_Motor_Setup();
    private boolean InTake_Button;
    private boolean OutTake_Button;
    public void Score_MotorCal (HardwareMap hardwareMap) {
        CH2IN.init(hardwareMap, "CH2Intake");
        EH2OUT.init(hardwareMap, "EH2OutTake");
        CH2IN.setDirection(DcMotorSimple.Direction.FORWARD);
        EH2OUT.setDirection(DcMotorSimple.Direction.REVERSE);
        // setup
    }

    public void Scoring_Grabber (boolean Inbutton, boolean Outbutton) {
        InTake_Button = Inbutton;
        OutTake_Button = Outbutton;
    }

    public void Scoring_Running () {
        if (InTake_Button == true) {
            CH2IN.setMotorSpeed(1);
        } else {
            CH2IN.setMotorSpeed(0);
        }
        if (OutTake_Button == true) {
            EH2OUT.setMotorSpeed(1);
        } else {
            EH2OUT.setMotorSpeed(0);
        }
    }
}
