package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAuto extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();


        int a = 1;
        int b = 4;
        int ringPos = b;

        boolean isAutoRunning = true;

        while (isAutoRunning) {
            robot.motorShooter.setPower(-1);
        }
       // temporary til camera is figured out
        driveOnHeading(75,0.5,0);

//        shootRings(robot.ANGLE_SHOOT_POWER_POS,1);
//
//        strafeOnHeading(-4,0.3,0);
//        shootRings(robot.ANGLE_SHOOT_POWER_POS,1);
//
//        strafeOnHeading(-4,0.3,0);
//        shootRings(robot.ANGLE_SHOOT_POWER_POS,1);

        driveOnHeading(10,0.5,0);

        if(ringPos == b) {
            //driveOnHeading(10,0.5,0);
        } else if (ringPos == a) {
           // driveOnHeading(10,0.5,0);
        } else {
          //  driveOnHeading(10,0.5,0);

        }
        sleep(500);
        
    }
}
