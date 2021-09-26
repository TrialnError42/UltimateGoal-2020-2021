package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class CamTest extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();


        //setup pipeline

        RingPosition ringPosition = WaitForStartRingCount();

        while (opModeIsActive()) {

            telemetry.addData("AfterWiatforSignal", 99);
            telemetry.update();

            if (ringPosition == RingPosition.NONE) {
                telemetry.addData("InIFblock", 0);
                telemetry.update();


            } else if (ringPosition == RingPosition.ONE) {
//            driveOnHeading(80,0.5,0);
                telemetry.addData("InIFblock", 1);
                telemetry.update();

            } else if (ringPosition == RingPosition.FOUR) {
                telemetry.addData("InIFblock", 4);
                telemetry.update();


            }
        }
    }
}
