/*
Copyright 2026 FIRST Tech Challenge Team 293

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.MecanumRobotController;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.Segment;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.SegmentedPath;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.Spline;

import java.util.ArrayList;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class SegmentedPathTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    double dt = 0.02;

    //@Override
    public void runOpMode() {


        String[] motorNames = {"DriveLF", "DriveRF", "DriveLB", "DriveRB"};
        boolean[] reverseList = {true, false, true, false};
        double P, I, D, F;
        P = 2.5;
        I = 0;
        D = 0;
        F = 1.5*(435/12.0/2); //load factor * (max rpm / max voltage) * 1/2
        PIDFCoefficients[] PIDList = {
                new PIDFCoefficients(P, I, D, F),
                new PIDFCoefficients(P, I, D, F),
                new PIDFCoefficients(P, I, D, F),
                new PIDFCoefficients(P, I, D, F)};
        dt = 0.02;
        double[] dimensions = {0.15, 0.19125, 0.052}; //length, width, wheel radius, in meters

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, runtime, new VectorF(0, 0, 0), new VectorF(0, 0, 0), 118, 126, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD, new GoBildaPinpointDriver.EncoderDirection[]{GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED});

        MecanumRobotController robot = new MecanumRobotController(hardwareMap, runtime, motorNames, reverseList, PIDList, dt, dimensions, localizer);

        double xt = 0, yt = 0, ht = 0;
        double x = 0, y = 0, h = 0;
        int i = 0; int j = 0;
        double sensitivity = -2.0;
        float kPval = (float) 0.60, kIval = (float) 0, kDval = (float) 0.12;
        double timeNow = 0;

        Segment segment1 = new Spline(
                new double[] {0, 0.25},
                new double[] {0},
                new double[] {0},
                4
        );
        Segment segment2 = new Spline(
                new double[] {1},
                new double[] {0, 0.25},
                new double[] {0},
                4
        );

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(segment1);
        segments.add(segment2);

        SegmentedPath trajectory = new SegmentedPath(segments); //should go in an L shape

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetRuntime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            xt += -0.08*gamepad1.left_stick_y;
            yt += -0.08*gamepad1.left_stick_x;
            ht += -0.08*gamepad1.right_stick_x;


            telemetry.addData("Target Pose", "%4.3f, %4.3f, %4.3f", xt, yt, ht);
            VectorF targetPose = new VectorF((float)xt, (float)yt, (float) ht);
            VectorF targetVel = robot.PID(targetPose, runtime.seconds());
            telemetry.addData("Target Vel", "%4.3f, %4.3f, %4.3f", targetVel.get(0), targetVel.get(1), targetVel.get(2));
            telemetry.addData("runtime.seconds", "%4.3f", runtime.seconds()-timeNow);
            if(gamepad1.dpad_down){
                timeNow = runtime.seconds();
            }
            if(gamepad1.a) {
                double error = targetPose.subtracted(robot.getLocalizer().getPose()).magnitude();
                telemetry.addData("error", "%4.3f", error);

                robot.followPath(trajectory, runtime.seconds(), timeNow);
            }
            else{
                robot.setTargetVelocity(new VectorF(0, 0, 0));
            }

            /*for(int m = 0; m < 4; m++){
                if(robot.motors[i].getPower()>0.75){
                    for(int n = 0; n < 4; n++){
                        robot.motors[j].setPower(robot.motors[j].getPower()*0.75/robot.motors[i].getPower());
                    }
                }
                telemetry.addData("velocity rad/s, power /100", "%4.1f, %4.2f, %4.3f", (float)i, robot.getMotor(i).getVelocity(AngleUnit.RADIANS), robot.getMotor(i).getPower());
            }*/
            robot.getLocalizer().updatePose();
            x = robot.getLocalizer().getPose().get(0);
            y = robot.getLocalizer().getPose().get(1);
            h = robot.getLocalizer().getPose().get(2);
            telemetry.addData("True Pose (x, y, h)", "%4.3f, %4.3f, %4.3f", x, y, h);
            double dx = robot.getLocalizer().getVel().get(0);
            double dy = robot.getLocalizer().getVel().get(1);
            double dh = robot.getLocalizer().getVel().get(2);
            telemetry.addData("Target Derivative", "%4.3f, %4.3f, %4.3f", dx, dy, dh);
            telemetry.update();




        }
    }
}
