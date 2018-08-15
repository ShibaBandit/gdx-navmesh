package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Pool;
import com.shibabandit.gdx_navmesh.util.Angles;

/**
 * Walkable edge in a navigation mesh.
 */
public class NavMeshPortal implements Pool.Poolable {

    private final Vector2 left, right;

    /** Length of the portal line segment */
    private float length;

    /** {@link #length} * 0.5f */
    private float lengthDiv2;

    /** Angle from left point towards right */
    private float leftIntAng;

    /** Angle from right point towards left */
    private float rightIntAng;

    public NavMeshPortal() {
        left = new Vector2();
        right = new Vector2();
    }

    /**
     * Initialize {@link com.badlogic.gdx.utils.Pool.Poolable} instance.
     *
     * @param left left-most vertex
     * @param right right-most vertex
     * @return {@link com.badlogic.gdx.utils.Pool.Poolable} instance
     */
    public NavMeshPortal init(Vector2 left, Vector2 right) {
        this.left.set(left);
        this.right.set(right);
        this.length = left.dst(right);
        this.lengthDiv2 = length * 0.5f;
        this.leftIntAng = Angles.between(left, right);
        this.rightIntAng = Angles.between(right, left);
        return this;
    }

    @Override
    public void reset() {
        left.setZero();
        right.setZero();
        length = 0f;
        lengthDiv2 = 0f;
        leftIntAng = 0f;
        rightIntAng = 0f;
    }

    /**
     * @return left-most vertex
     */
    public Vector2 getLeft() {
        return left;
    }

    /**
     * @return right-most vertex
     */
    public Vector2 getRight() {
        return right;
    }

    /**
     * @return distance in world units between {@link #left} and {@link #right}
     */
    public float getLength() {
        return length;
    }

    /**
     * @return distance in world units between {@link #left} and {@link #right} / 2
     */
    public float getLengthDiv2() {
        return lengthDiv2;
    }

    /**
     * @return interior angle from left to right
     */
    public float getLeftIntAng() {
        return leftIntAng;
    }

    /**
     * @return interior angle from right to left
     */
    public float getRightIntAng() {
        return rightIntAng;
    }
}
