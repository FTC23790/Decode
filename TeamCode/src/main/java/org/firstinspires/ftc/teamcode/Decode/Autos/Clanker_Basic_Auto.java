package org.firstinspires.ftc.teamcode.Decode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Clanker_Basic_Auto extends OpMode{
    @Override
    public void init() {
        telemetry.addData("test", 0);
    }

    @Override
    public void loop() {
        telemetry.addData("test", 1);
    }
}
