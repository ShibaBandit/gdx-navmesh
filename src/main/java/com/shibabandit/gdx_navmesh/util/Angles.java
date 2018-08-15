package com.shibabandit.gdx_navmesh.util;

import com.badlogic.gdx.math.Vector2;

/**
 * Utility class of angle math. Most angles are degree units [0, 360].
 */
public final class Angles {

    /**
     * Normalize a degrees angle, only works for angles in range of 360 degrees outside of [0, 360]
     * (hence the 'dumb' in the name).
     *
     * @param angleDegrees angle possibly outside the range of [0, 360]
     * @return normalized degrees angle in range [0, 360]
     */
    public static float dumbNormAngle(float angleDegrees) {
        if(angleDegrees > 360f) {
            angleDegrees -= 360f;
        } else if(angleDegrees < 0f) {
            angleDegrees += 360f;
        }

        return angleDegrees;
    }

    /**
     * Calculate angle from source point to destination point in degrees.
     *
     * @param srcX source point x coordinate
     * @param srcY source point y coordinate
     * @param destX destination point x coordinate
     * @param destY destination point y coordinate
     * @return angle between points in degrees
     */
    public static float between(float srcX, float srcY, float destX, float destY) {
        return dumbNormAngle((float) (Math.atan2(destY - srcY, destX - srcX) * 180d / Math.PI));
    }

    /**
     * Calculate angle from source point to destination point in degrees.
     *
     * @param src source point
     * @param dest destination point
     * @return angle between points in degrees
     * @see #between(float, float, float, float)
     */
    public static float between(Vector2 src, Vector2 dest) {
        return between(src.x, src.y, dest.x, dest.y);
    }

    /**
     * True if the shortest angle from srcAng to destAng is CCW (counter-close-wise, +), false if CW (clock-wise, -).
     * Angles are in degrees.
     *
     * @param srcAng source angle in degrees
     * @param destAng destination angle in degrees
     * @return true if the shortest angle from source angle to destination angle is CCW, false if CW
     */
    public static boolean isShortRotCCW(float srcAng, float destAng) {

        // Find angular dist +, -
        float angDistPos;
        if(destAng > srcAng) {
            angDistPos = destAng - srcAng;
        } else {
            angDistPos = 360f - srcAng + destAng;
        }

        float angDistNeg;
        if(destAng > srcAng) {
            angDistNeg = srcAng + (360f - destAng);
        } else {
            angDistNeg = srcAng - destAng;
        }

        return angDistPos <= angDistNeg;
    }
}
