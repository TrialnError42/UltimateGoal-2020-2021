package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class CamRedAuto extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();


        //setup pipeline

        RingPosition ringPosition = WaitForStartRingCount();

        while (opModeIsActive()) {

            telemetry.addData("AfterWiatforSignal", 99);
            telemetry.update();

            robot.servoHolder.setPosition(robot.HOLD_ERECT_POS);
            driveOnHeadingArm(84,0.5,0, 2100);
            // move angle up
            angleHighGoalBeginningPos();
            robot.motorShooter.setPower(-0.95);
            strafeOnHeading(20,0.3,0);
//            driveOnHeading(-1,0.3,0);
            shootRings(3);


            if (ringPosition == RingPosition.NONE) {
                telemetry.addData("InIFblockPOSITION", 0);
                telemetry.update();

                turnToHeading(125,0.8);
                driveOnHeading(-25,0.5,125);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                sleep(300);
                moveAngleDown();
                turnToHeading(0,0.5);
                strafeOnHeading(-1,0.3,0);
                moveArmDown(450);
                driveOnHeading(-62,0.5,0);
                robot.servoClamp.setPosition(robot.CLAMP_CLOSE_POS);
                sleep(300);
                moveArmUp(150);
                strafeOnHeading(-8,0.5,0);
                turnToHeading(160,0.5);
                driveOnHeading(-44,1,160);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                moveArmUp(750);
                robot.servoClamp.setPosition(robot.CLAMP_CLOSE_POS);
                strafeOnHeading(40,1,90);
                turnToHeading(100,0.5);
                moveArmDown(600);
                turnToHeading(0,0.3);

                sleep(20000);

            } else if (ringPosition == RingPosition.ONE) {
//            driveOnHeading(80,0.5,0);
                telemetry.addData("InIFblockPOSITION", 1);
                telemetry.update();

                driveOnHeading(35,0.5,0);
                turnToHeading(180,0.5);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                sleep(300);
                moveAngleDown();
                moveArmUp(500);
                turnToHeading(0,0.5);
                robot.motorIntake.setPower(0.8);
                driveOnHeading(-63,0.8,0);
                driveOnHeading(-5,0.5,0);
                robot.motorIntake.setPower(0);
                turnToHeading(26,0.5);
                moveArmDown(800);
                driveOnHeading(-20,0.5,24);
                robot.servoClamp.setPosition(robot.CLAMP_CLOSE_POS);
                sleep(300);
                moveArmUp(100);
                driveOnHeading(25,0.8,20);
                turnToHeading(0,0.5);
                angleHighGoalPos();
                robot.motorShooter.setPower(-1);
                driveOnHeading(25,0.5,0);
                robot.motorShooter.setPower(-1);
                shootRings(1);
                turnToHeading(180,1);
                driveOnHeading(-30,1,180);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                moveAngleDown();
                driveOnHeading(20,1,180);
                turnToHeading(0,0.3);

                sleep(20000);


            } else if (ringPosition == RingPosition.FOUR) {
                telemetry.addData("InIFblockPOSITION", 4);
                telemetry.update();

                driveOnHeading(57,0.8,0);
                turnToHeading(130,0.5);
                driveOnHeading(-20,0.8,130);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                sleep(300);
                moveAngleDown();
                driveOnHeading(23,0.8,130);
                turnToHeading(0,0.5);
//                strafeOnHeading(8,0.5,0);
//                robot.motorIntake.setPower(0.8);
//                strafeOnHeading(,0.5,0);
                driveOnHeading(-80,0.8,0);
                robot.motorIntake.setPower(0.8);
                driveOnHeading(-12,0.5,0);
                robot.motorShooter.setPower(-0.98);
                driveOnHeading(30,0.5,0);
                angleHighGoalPos();
                shootRings(3);
                moveAngleDown();
                robot.motorShooter.setPower(0);
                driveOnHeading(-45,0.8,0);
                driveOnHeading(10,0.8,0);
                robot.motorShooter.setPower(-1);
                driveOnHeading(18,0.8,0);
                angleHighGoalBeginningPos();
                shootRings(3);
                moveAngleDown();
                driveOnHeading(13,1,0);
                turnToHeading(0,0.3);

                sleep(20000);


            }
        }
    }
}
