package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class HIGH_GOAL_AUTO extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();

//        driveOnHeading(98,1,0);

        // b
        
        robot.servoHolder.setPosition(robot.HOLD_ERECT_POS);
        driveOnHeadingArm(69,0.5,0, 2650);

        // move angle up
        moveAngleUp(10);
        robot.servoHolder.setPosition(robot.HOLD_HIGH_GOAL_POS);
        moveAngleUp(robot.ANGLE_SHOOT_HIGH_POS);
        moveAngleDown1(75);
        sleep(100);


        strafeOnHeading(17,0.3,0);
        shootRings(3);

        driveOnHeading(30,0.5,0);
        turnToHeading(180,0.8);
        robot.servoClamp.setPosition(robot.CLAMP_OPEN_POS);
        sleep(300);
        robot.servoHolder.setPosition(robot.HOLD_ERECT_POS);

        turnToHeading(0,0.3);
        robot.motorIntake.setPower(0.8);
        driveOnHeading(-70,0.8,0);
        driveOnHeading(-5,0.5,0);
        driveOnHeading(40,0.5,0);

        moveAngleUp(10);
        robot.servoHolder.setPosition(robot.HOLD_HIGH_GOAL_POS);
        moveAngleUp(robot.ANGLE_SHOOT_HIGH_POS);
        moveAngleDown1(75);

        shootRings(3);
        robot.servoHolder.setPosition(robot.HOLD_ERECT_POS);

        driveOnHeading(-10,0.8,0);

        driveOnHeading(55,0.8,0);
        moveAngleUp(10);
        robot.servoHolder.setPosition(robot.HOLD_HIGH_GOAL_POS);
        moveAngleUp(robot.ANGLE_SHOOT_HIGH_POS);
        moveAngleDown1(75);
        sleep(100);
        shootRings(1);

        driveOnHeading(5,1,0);




    }
}
