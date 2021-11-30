package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class TeleOpBlue extends TeleOpRed {
    public TeleOpBlue() {
        super();
        alliance = AllianceColor.BLUE;
    }
}
