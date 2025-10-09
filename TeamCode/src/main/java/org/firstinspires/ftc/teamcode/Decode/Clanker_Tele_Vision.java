package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Driving_System;
//import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Lifting_System;
import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Scoring_System;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp
public class Clanker_Tele_Vision extends OpMode {
    Driving_System DS = new Driving_System();
    // drive system object
    //Lifting_System LS = new Lifting_System();
    //lifting system object
    Scoring_System SS = new Scoring_System();
    // Intake/outtake object
    double LX1;
    double LY1;
    double RX1;
    double TL1;
    double TR1;
    boolean LB2;
    boolean RB2;
    IMU imu;
    double PowerMod;
    boolean A1;
    boolean X2;
    boolean B2;

    // Limelight declaration
    private Limelight3A limelight = null;
    private static final String LIMELIGHT_NAME = "limelight"; // Adjust based on config

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.
                        LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));

        DS.Drive_MotorCal(hardwareMap);
        //LS.Lift_MotorCal(hardwareMap);
        SS.Score_MotorCal(hardwareMap);
        imu.resetYaw();
        //LS.RSTarget(-25);
        //Initilise HardwareMap setup

    try {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(0); // Set to Pipeline 0 (assuming AprilTags)
        limelight.start(); // Start data acquisition
        telemetry.addData("Limelight", "Initialized");
    } catch (Exception e) {
        telemetry.addData("Limelight Error", "Not found: " + LIMELIGHT_NAME);
        limelight = null;
    }
}

    @Override
    public void loop() {
        LX1 = +1 * gamepad1.left_stick_x;
        LY1 = -1 * gamepad1.left_stick_y;
        RX1 = +1 * gamepad1.right_stick_x;
        TL1 = gamepad1.left_trigger;
        TR1 = gamepad1.right_trigger;
        A1 = gamepad1.a;
        X2 = gamepad2.x;
        B2 = gamepad2.b;
        LB2 = gamepad2.left_bumper;
        RB2 = gamepad2.right_bumper;
        // gamepad setting

        if ( 0.2 < TL1 ) {
            PowerMod = 1.0;
        } else if (0.2 < TR1) {
            PowerMod = 0.4;
        } else {
            PowerMod = 0.7;
        }
        //power matrix

        if (A1 == true) {imu.resetYaw();}
        // direction reset

        DS.Drive_Grabber(LX1, LY1, RX1, PowerMod, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        DS.Drive_Running();
        // drive system module

        //LS.Lift_Grabber(LB2, RB2);
        //LS.RightLiftSys();
        // lift system module

        SS.Scoring_Grabber(X2, B2);
        SS.Scoring_Running();
        // Scoring system module

        telemetry.addData("Heading", ( DS.RAPrint() / ( 2 * 3.14159 ) ) * 360 );
        telemetry.addData("Power", PowerMod );
        telemetry.addData("X Input",LX1);
        telemetry.addData("Y Input",LY1);
        telemetry.addData("R Input",RX1);
        //telemetry.addData("Right Slide", LS.RSPrint());
        //telemetry moduel





    @Override
    public void stop() {
        // Stop Limelight polling when opmode stop
        if (limelight != null) {
            limelight.stop();
        }
    }
}
