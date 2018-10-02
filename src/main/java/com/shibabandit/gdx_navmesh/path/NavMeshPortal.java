package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Pool;
import com.shibabandit.gdx_navmesh.util.Angles;

import java.util.Objects;

/**
 * Walkable edge in a navigation mesh.
 */
public class NavMeshPortal implements Pool.Poolable {

    private final Vector2 left, right, midpoint;

    /** Length of the portal line segment */
    private float length;

    /** {@link #length} * 0.5f */
    private float lengthDiv2;

    /** Angle from left point towards right */
    private float leftIntAng;

    /** Angle from right point towards left */
    private float rightIntAng;

    /** Flag for ignoring portal length. Useful for ignoring 'fake' start/end goal portals vs agent radius. */
    private boolean ignorePortalLength;

    public NavMeshPortal() {
        left = new Vector2();
        right = new Vector2();
        midpoint = new Vector2();
    }

    /**
     * Initialize {@link com.badlogic.gdx.utils.Pool.Poolable} instance. {@link #ignorePortalLength} is false.
     *
     * @param left left-most vertex
     * @param right right-most vertex
     * @return {@link com.badlogic.gdx.utils.Pool.Poolable} instance
     */
    public NavMeshPortal init(Vector2 left, Vector2 right) {
        return init(left, right, false);
    }

    /**
     * Initialize {@link com.badlogic.gdx.utils.Pool.Poolable} instance.
     *
     * @param left left-most vertex
     * @param right right-most vertex
     * @param ignorePortalLength flag for ignoring portal length. Useful for ignoring 'fake' start/end goal portals vs agent radius.
     * @return {@link com.badlogic.gdx.utils.Pool.Poolable} instance
     */
    public NavMeshPortal init(Vector2 left, Vector2 right, boolean ignorePortalLength) {
        this.left.set(left);
        this.right.set(right);
        this.midpoint.set(left).add(right).scl(.5f);
        this.length = left.dst(right);
        this.lengthDiv2 = length * 0.5f;
        this.leftIntAng = Angles.between(left, right);
        this.rightIntAng = Angles.between(right, left);
        this.ignorePortalLength = ignorePortalLength;
        return this;
    }

    @Override
    public void reset() {
        left.setZero();
        right.setZero();
        midpoint.setZero();
        length = 0f;
        lengthDiv2 = 0f;
        leftIntAng = 0f;
        rightIntAng = 0f;
        ignorePortalLength = false;
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
     * @return midpoint between left and right
     */
    public Vector2 getMidpoint() {
        return midpoint;
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

    /**
     * @return Flag for ignoring portal length. Useful for ignoring 'fake' start/end goal portals vs agent radius.
     */
    public boolean isIgnorePortalLength() {
        return ignorePortalLength;
    }

    /**
     * @param ignorePortalLength flag for ignoring portal length. Useful for ignoring 'fake' start/end goal portals vs agent radius.
     */
    public void setIgnorePortalLength(boolean ignorePortalLength) {
        this.ignorePortalLength = ignorePortalLength;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        NavMeshPortal that = (NavMeshPortal) o;
        return Objects.equals(left, that.left) &&
                Objects.equals(right, that.right);
    }

    @Override
    public int hashCode() {
        return Objects.hash(left, right);
    }
}
