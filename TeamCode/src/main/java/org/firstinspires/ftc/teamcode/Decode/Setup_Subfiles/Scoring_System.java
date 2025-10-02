package org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Motor_Servo_Setup.Drive_Motor_Setup;
public class Scoring_System {
    Drive_Motor_Setup CH2IN = new Drive_Motor_Setup();
    private boolean Intake_Button;
    public void Score_MotorCal (HardwareMap hardwareMap) {
        CH2IN.init(hardwareMap, "CH2Intake");
        CH2IN.setDirection(DcMotorSimple.Direction.FORWARD);
        // setup
    }

    public void Scoring_Grabber (boolean button) {
        Intake_Button = button;
    }

    public void Scoring_Running () {
        if (Intake_Button == true) {
            CH2IN.setMotorSpeed(1);
        } else {
            CH2IN.setMotorSpeed(0);
        }
    }
}
