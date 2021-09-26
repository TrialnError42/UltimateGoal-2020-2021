package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class CamRedAutoPathChange extends AutoCommon {

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
            moveAngleDown();


            if (ringPosition == RingPosition.NONE) {
                telemetry.addData("InIFblockPOSITION", 0);
                telemetry.update();

                turnToHeading(125,0.8);
                driveOnHeading(-25,0.5,125);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                sleep(300);
                turnToHeading(0,0.5);
                strafeOnHeading(-1,0.3,0);
                moveArmDown(650);
                driveOnHeading(-62,0.5,0);
                robot.servoClamp.setPosition(robot.CLAMP_CLOSE_POS);
                sleep(300);
                moveArmUp(150);
                strafeOnHeading(-8,0.5,0);
                turnToHeading(160,0.5);
                driveOnHeading(-35,1,160);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                moveArmUp(750);
                robot.servoClamp.setPosition(robot.CLAMP_CLOSE_POS);
                strafeOnHeading(40,1,90);
                turnToHeading(100,0.5);
                moveArmDown(600);

                sleep(20000);

            } else if (ringPosition == RingPosition.ONE) {
//            driveOnHeading(80,0.5,0);
                telemetry.addData("InIFblockPOSITION", 1);
                telemetry.update();
                driveOnHeading(35,0.5,0);
                turnToHeading(180,0.5);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                sleep(300);
                moveArmUp(500);
                turnToHeading(0,0.3);
                robot.motorIntake.setPower(0.8);
                driveOnHeading(-63,0.8,0);
                driveOnHeading(-7,0.5,0);
                robot.motorIntake.setPower(0);
                turnToHeading(26,0.5);
                moveArmDown(1100);
                driveOnHeading(-22,0.5,24);
                robot.servoClamp.setPosition(robot.CLAMP_CLOSE_POS);
                sleep(300);
                robot.motorIntake.setPower(0.8);
                moveArmUp(100);
                driveOnHeading(25,0.8,20);
                turnToHeading(0,0.5);
                angleHighGoalPos();
                robot.motorIntake.setPower(0);
                robot.motorShooter.setPower(-1);
                driveOnHeading(19,0.5,0);
                robot.motorShooter.setPower(-1);
                shootRings(1);
                turnToHeading(180,0.5);
                driveOnHeading(-35,1,180);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                moveAngleDown();
                driveOnHeading(20,1,180);

                sleep(20000);


            } else if (ringPosition == RingPosition.FOUR) {
                telemetry.addData("InIFblockPOSITION", 4);
                telemetry.update();

                strafeOnHeading(-2,0.3,0);
                driveOnHeading(-30,0.8,0);
                robot.motorIntake.setPower(0.8);
                driveOnHeading(-10,0.5,0);
                robot.motorShooter.setPower(-0.98);
                driveOnHeading(40,0.8,0);
                angleHighGoalPos();
                shootRings(3);
                moveAngleDown();
                robot.motorShooter.setPower(0);
                driveOnHeading(-45,0.8,0);
                driveOnHeading(40,0.8,0);
                robot.motorIntake.setPower(0);
                robot.motorShooter.setPower(-1);
                angleHighGoalBeginningPos();
                shootRings(3);
                moveAngleDown();
                driveOnHeading(57,0.8,0);
                turnToHeading(130,0.5);
                driveOnHeading(-20,0.8,130);
                robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
                sleep(300);
                driveOnHeading(23,0.8,130);
                turnToHeading(0,0.5);
                driveOnHeading(-40,1,0);

                sleep(20000);


            }
        }
    }
}