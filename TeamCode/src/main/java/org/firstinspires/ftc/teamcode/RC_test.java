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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.MecanumRobotControllerOld;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.PinpointLocalizer;

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

public class RC_test extends LinearOpMode {

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

        MecanumRobotControllerOld robot = new MecanumRobotControllerOld(hardwareMap, runtime, motorNames, reverseList, PIDList, dt, dimensions, localizer);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double angular = -gamepad1.right_stick_x;

            VectorF input = new VectorF((float)(0.15*axial), (float)(0.15*lateral), (float)(0.2*angular));
            VectorF rotatedInput = robot.rotate(input, robot.getLocalizer().getPose().get(2));
            robot.setTargetVelocity(rotatedInput);
            for(int i = 0; i < 4; i++){
                //robot.motors[i].setVelocity(2.5, AngleUnit.RADIANS);
                telemetry.addData("velocity rad/s, power /100", "%4.1f, %4.2f, %4.3f", (float)i, robot.getMotor(i).getVelocity(AngleUnit.RADIANS), robot.getMotor(i).getPower());
            }
            robot.getLocalizer().updatePose();
            telemetry.addData("x", "%4.3f", robot.getLocalizer().getPose().get(0));
            telemetry.addData("y", "%4.3f", robot.getLocalizer().getPose().get(1));
            telemetry.addData("pp heading", "%4.3f", robot.getLocalizer().getPose().get(2));
            telemetry.update();

            //robot.getLocalizer().updateTelemetry();



        }
    }
}
