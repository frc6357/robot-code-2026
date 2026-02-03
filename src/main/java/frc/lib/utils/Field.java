// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.lib.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class Field {
        public static double getFieldLength() {
                return FieldConstants.fieldLength;
        }
        public static double getFieldWidth() {
                return FieldConstants.fieldWidth;
        }

        /** Returns {@code true} if the robot is on the blue alliance. */
        public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
        }

        /** Returns {@code true} if the robot is on the red alliance. */
        public static boolean isRed() {
        return !isBlue();
        }

        public static final Trigger red = new Trigger(() -> isRed());
        public static final Trigger blue = new Trigger(() -> isBlue());

        // Flip the angle if we are blue, as we are setting things for a red driver station angle
        // This flips the left and right side for aiming purposes
        public static double flipAimAngleIfBlue(double redAngleDegs) {
                if (Field.isBlue()) {
                        return 180 - redAngleDegs;
                }
                return redAngleDegs;
        }

        // This flips the true angle of the robot if we are blue
        public static double flipTrueAngleIfBlue(double redAngleDegs) {
                if (Field.isBlue()) {
                        return (180 + redAngleDegs) % 360;
                }
                return redAngleDegs;
        }

        public static double flipTrueAngleIfRed(double blueAngleDegs) {
                if (Field.isRed()) {
                        return (180 + blueAngleDegs) % 360;
                }
                return blueAngleDegs;
        }

        public static Rotation2d flipAngleIfRed(Rotation2d blue) {
                if (Field.isRed()) {
                        return new Rotation2d(-blue.getCos(), blue.getSin());
                } else {
                        return blue;
                }
        }

        public static Pose2d flipXifRed(Pose2d blue) {
                return new Pose2d(
                        flipXifRed(blue.getX()), blue.getTranslation().getY(), blue.getRotation());
        }

        public static Translation2d flipXifRed(Translation2d blue) {
                return new Translation2d(flipXifRed(blue.getX()), blue.getY());
        }

        public static Translation3d flipXifRed(Translation3d blue) {
                return new Translation3d(flipXifRed(blue.getX()), blue.getY(), blue.getZ());
        }

        // If we are red flip the x pose to the other side of the field
        public static double flipXifRed(double xCoordinate) {
                if (Field.isRed()) {
                        return FieldConstants.fieldLength - xCoordinate;
                }
                return xCoordinate;
        }

        // If we are red flip the y pose to the other side of the field
        public static double flipYifRed(double yCoordinate) {
                if (Field.isRed()) {
                        return FieldConstants.fieldWidth - yCoordinate;
                }
                return yCoordinate;
        }

        public static Translation2d flipIfRed(Translation2d point) {
                if (Field.isRed()) {
                        return new Translation2d(
                                flipXifRed(point.getX()),
                                flipYifRed(point.getY()));
                }
                return point;
        }

        public static boolean poseOutOfField(Pose2d pose2D) {
                double x = pose2D.getX();
                double y = pose2D.getY();
                return (x <= 0 || x >= FieldConstants.fieldLength) || (y <= 0 || y >= FieldConstants.fieldWidth);
        }

        public static boolean poseOutOfField(Pose3d pose3D) {
                return poseOutOfField(pose3D.toPose2d());
        }
}