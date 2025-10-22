package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Decode.Setup_Subfiles.Scrapbin_DriveTrain;

@TeleOp
public class Scrapbin_Tele extends OpMode {
    Scrapbin_DriveTrain SBDS = new Scrapbin_DriveTrain();
    double PowerMod;
    double LY1;
    double RY1;
    double TL1;
    double TR1;
    @Override
    public void init() {

    }
    @Override
    public void loop() {
        LY1 = -1 * gamepad1.left_stick_y;
        RY1 = -1 * gamepad1.right_stick_y;
        TL1 = gamepad1.left_trigger;
        TR1 = gamepad1.right_trigger;

        if ( 0.2 < TL1 ) {
            PowerMod = 1.0;
        } else if (0.2 < TR1) {
            PowerMod = 0.4;
        } else {
            PowerMod = 0.7;
        }
        //power matrix

        SBDS.Bin_Drive_Grabber(LY1, RY1, PowerMod);
        SBDS.Bin_Drive_Running();
        // drive system module
    }
}
