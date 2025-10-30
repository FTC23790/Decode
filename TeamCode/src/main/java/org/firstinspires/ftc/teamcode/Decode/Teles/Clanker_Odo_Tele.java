package org.firstinspires.ftc.teamcode.Decode.Teles;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Decode.Setup.ImportedStuffs.NO_TOUCH.GoBildaPinpointDriver;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Decode.SubSystems.Driving_System;
import org.firstinspires.ftc.teamcode.Decode.SubSystems.Scoring_System;

@TeleOp
public class Clanker_Odo_Tele extends OpMode {

    GoBildaPinpointDriver odo;
    // makes odo object
    double oldTime = 0;
    // odo bit
    Driving_System DS = new Driving_System();
    // drive system object
    Scoring_System SS = new Scoring_System();
    // Intake/outtake object
    TelemetryPacket Tele = new TelemetryPacket();
    double LX1;
    double LY1;
    double RX1;
    double TL1;
    double TR1;
    double PowerMod;
    boolean A1;
    boolean A2;
    boolean X2;
    boolean B2;
    boolean UP2; // LINE ADDED BY NICK FOR TESTING
    double TL2;
    double TR2;
    boolean DU2;
    boolean DD2;
    double PassThrough;
    int OutTake_RPM;
    double Intake;

    double XposCurrent;
    double YposCurrent;
    double RposCurrent;

    @Override
    public void init() {
        DS.Drive_MotorCal(hardwareMap);
        SS.Score_MotorCal(hardwareMap);
        //Initilise HardwareMap setup

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(0 , 0 , DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED , GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // x dirention then y dierncton
        odo.resetPosAndIMU();
        // odo cal bits
    }

    @Override
    public void loop() {

        Pose2D pos = odo.getPosition();
        XposCurrent = pos.getY(DistanceUnit.INCH);
        YposCurrent = pos.getX(DistanceUnit.INCH);
        RposCurrent = pos.getHeading(AngleUnit.DEGREES);
        //feeding the odo positions into code varibles
        // it calls strafe Y and X is frward backard, so calling Y X and X Y
        // to use X as horoziontal strafe and Y forward Backward

        LX1 = -1 * gamepad1.left_stick_x;
        LY1 = +1 * gamepad1.left_stick_y;
        RX1 = +1 * gamepad1.right_stick_x;
        TL1 = gamepad1.left_trigger;
        TR1 = gamepad1.right_trigger;
        TL2 = gamepad2.left_trigger;
        TR2 = gamepad2.right_trigger;
        A1 = gamepad1.a;
        A2 = gamepad2.a;
        X2 = gamepad2.x;
        B2 = gamepad2.b;
        UP2 = gamepad2.dpad_up;
        DD2 = gamepad2.dpad_down;
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
            OutTake_RPM = 6000;
        } else if (0.2 < TR2) {
            OutTake_RPM = 3000;
        } else {
            OutTake_RPM = 4500;
        }
        // outtake speed matrix

        if (DU2 == true) {
            PassThrough = 1;
        } else if (DD2 == true) {
            PassThrough = -1;
        } else {
            PassThrough = 0;
        }
        // passthrough direction matrix

        if (X2 == true) {
            Intake = 1;
        } else if (B2 == true) {
            Intake = -1;
        } else {
            Intake = 0;
        }
        // Intake direction matrix


        if (A1 == true) {odo.resetPosAndIMU();}
        //odo yaw reset

        DS.Drive_Grabber(LX1, LY1, RX1, PowerMod, odo.getHeading(AngleUnit.RADIANS) );
        DS.Drive_Running();
        // drive system module

        SS.Scoring_Grabber(Intake, A2, OutTake_RPM, PassThrough);
        SS.Scoring_Running();
        // Scoring system module

        odo.update();
        // self explaitry

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        // odo update module

        //telemetry.addData("Heading", ( DS.RAPrint() / ( 2 * 3.14159 ) ) * 360 );
        //telemetry.addData("X Input",LX1);
        //telemetry.addData("Y Input",LY1);
        //telemetry.addData("R Input",RX1);
        // normal telmetry
        //telemetry.addLine();
        //telemetry.addLine("Odo bits");
        telemetry.addData("Odo_X", XposCurrent);
        telemetry.addData("Odo_Y", YposCurrent);
        telemetry.addData("Odo_R", RposCurrent);
        telemetry.addData("Power", PowerMod );
        telemetry.addData("Out Take RPM", OutTake_RPM);
        // odo telemtery
        telemetry.addLine();
        Tele.put("X", XposCurrent);
        Tele.put("Y", YposCurrent);
        Tele.put("R", RposCurrent);
        // FTC Dashboard text test
            //telemetry moduel

    }
}
