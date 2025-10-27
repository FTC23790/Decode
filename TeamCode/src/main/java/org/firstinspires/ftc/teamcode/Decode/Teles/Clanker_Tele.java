package org.firstinspires.ftc.teamcode.Decode.Teles;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Decode.SubSystems.Driving_System;
//import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Lifting_System;
import org.firstinspires.ftc.teamcode.Decode.SubSystems.Scoring_System;

@TeleOp
public class Clanker_Tele extends OpMode {
    Driving_System DS = new Driving_System();
    // drive system object
    Scoring_System SS = new Scoring_System();
    // Intake/outtake object
    double LX1;
    double LY1;
    double RX1;
    double TL1;
    double TR1;
    double TL2;
    double TR2;
    IMU imu;
    double PowerMod;
    boolean A1;
    boolean X2;
    boolean B2;
    double OutTake_RPM;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.
                        LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));

        DS.Drive_MotorCal(hardwareMap);
        SS.Score_MotorCal(hardwareMap);
        imu.resetYaw();
        //Initilise HardwareMap setup
    }

    @Override
    public void loop() {
        LX1 = -1 * gamepad1.left_stick_x;
        LY1 = +1 * gamepad1.left_stick_y;
        RX1 = +1 * gamepad1.right_stick_x;
        TL1 = gamepad1.left_trigger;
        TR1 = gamepad1.right_trigger;
        TL2 = gamepad2.left_trigger;
        TR2 = gamepad2.right_trigger;
        A1 = gamepad1.a;
        X2 = gamepad2.x;
        B2 = gamepad2.b;
        // gamepad setting

        if ( 0.2 < TL1 ) {
            PowerMod = 1.0;
        } else if (0.2 < TR1) {
            PowerMod = 0.4;
        } else {
            PowerMod = 0.7;
        }
        //power matrix

        if ( 0.2 < TL2 ) {
            OutTake_RPM = 1;
        } else if (0.2 < TR2) {
            OutTake_RPM = 0.5;
        } else {
            OutTake_RPM = 0.75;
        }
        // outtake speed matrix

        if (A1 == true) {imu.resetYaw();}
        // direction reset

        DS.Drive_Grabber(LX1, LY1, RX1, PowerMod, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        DS.Drive_Running();
        // drive system module

        SS.Scoring_Grabber(X2, B2, OutTake_RPM);
        SS.Scoring_Running();
        // Scoring system module

        telemetry.addData("Heading", ( DS.RAPrint() / ( 2 * 3.14159 ) ) * 360 );
        telemetry.addData("Power", PowerMod );
        telemetry.addData("X Input",LX1);
        telemetry.addData("Y Input",LY1);
        telemetry.addData("R Input",RX1);
        //telemetry.addData("Right Slide", LS.RSPrint());
        telemetry.addLine();
        telemetry.addData("Out Take RPM", (OutTake_RPM * 6000));
        //telemetry.addData("button", B2);
        //telemetry.addData("outcurrent", SS.Out_Return());
        //telemetry moduel

    }
}
