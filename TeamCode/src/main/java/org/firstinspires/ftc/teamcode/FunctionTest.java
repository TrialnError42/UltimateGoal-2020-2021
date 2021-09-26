package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FunctionTest extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();

        //drive out to the foundation

//        driveOnHeading(90,1,0);
        moveArmDown(1000);
    }
}