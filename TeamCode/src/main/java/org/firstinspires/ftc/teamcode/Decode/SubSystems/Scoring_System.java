package org.firstinspires.ftc.teamcode.Decode.SubSystems;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Drive_Motor_Setup;
import org.firstinspires.ftc.teamcode.Decode.Setup.Motor_Servo_Setup.Cont_Servo_Setup;
public class Scoring_System {
    Cont_Servo_Setup CH0IN = new Cont_Servo_Setup();
    // change object name baseded on port
    Drive_Motor_Setup EH3OUT = new Drive_Motor_Setup();
    Cont_Servo_Setup EHS5P = new Cont_Servo_Setup();
    Cont_Servo_Setup EHS4MP = new Cont_Servo_Setup();
    private double OutTPS;
    private boolean InTake_Button;
    private boolean OutTake_Button;
    private boolean BigPass;
    private boolean MiniPass;
    private boolean Back;
    public void Score_MotorCal (HardwareMap hardwareMap) {
        CH0IN.init(hardwareMap,"CHS0Intake");
        EH3OUT.init(hardwareMap,"EH3OutTake");
        EHS5P.init(hardwareMap,"EHS5Pass");
        EHS4MP.init(hardwareMap,"EHS4MiniPass");
        CH0IN.setDirection(DcMotorSimple.Direction.REVERSE);
        EH3OUT.setDirection(DcMotorSimple.Direction.REVERSE);
        EHS5P.setDirection(DcMotorSimple.Direction.FORWARD);
        EHS4MP.setDirection(DcMotorSimple.Direction.FORWARD);
        // setup
        OutTPS = 2800;
        // 2,800 ticks per second for outtake motor
    }

    public void Scoring_Grabber (boolean Pass, boolean Pass_back) {
        BigPass = Pass;
        Back = Pass_back;
    }

    public void Scoring_Running () {

        if (BigPass == true) {
            EHS5P.setPower(1);
        } else if (Back == true) {
            EHS5P.setPower(-1);
        } else {
            EHS5P.setPower(0);
        }

        EHS4MP.setPower(1);
        // Mini Pass Power to full
        CH0IN.setPower(1);
        // Intake Power to full
        EH3OUT.setMotorVelocity(OutTPS * (0.67));
        //EH3OUT.setMotorVelocity(OutTPS * (2/3));
        // Outake "Power" to two thirds
    }
}
