package org.firstinspires.ftc.teamcode.Decode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Driving_System;
import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.ImportedStuffs.NO_TOUCH.GoBildaPinpointDriver;

@Autonomous
public class Clanker_Basic_Auto extends OpMode {
    Driving_System DS = new Driving_System();
    GoBildaPinpointDriver odo;
    // makes odo object
    double oldTime = 0;
    double XposCurrent;
    double YposCurrent;
    double RposCurrent;
    @Override
    public void init() {
        DS.Drive_MotorCal(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "Clanker_Odo");
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

        odo.update();
        // self explaitry

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        // odo update module

        telemetry.addLine("Odo bits");
        telemetry.addData("Odo_X", XposCurrent);
        telemetry.addData("Odo_Y", YposCurrent);
        telemetry.addData("Odo_R", RposCurrent);

        if (YposCurrent >= 10) {
            // it says stop at 10 inches, but it has 10 inches of stoping distance so
            // it actuly stops after  20 inches
            DS.Stop();
        } else {
            DS.Move_Forward();
        }

    }
}
