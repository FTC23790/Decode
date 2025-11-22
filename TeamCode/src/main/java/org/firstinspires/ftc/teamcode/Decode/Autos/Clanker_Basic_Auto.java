package org.firstinspires.ftc.teamcode.Decode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Decode.SubSystems.Driving_System_Field;


@Autonomous
public class Clanker_Basic_Auto extends LinearOpMode {
    Driving_System_Field DS = new Driving_System_Field();
    @Override
    public void runOpMode() {
        DS.Drive_MotorCal(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            DS.Drive_Grabber(0,-1,0,0.7,0);
            DS.Drive_Running();
            sleep(600);
            DS.Drive_Grabber(0,0,0,0,0);
            DS.Drive_Running();
            sleep(300);
            break;
        }
    }
}
