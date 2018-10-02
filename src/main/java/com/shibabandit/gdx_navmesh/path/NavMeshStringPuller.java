package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.DefaultGraphPath;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;

/**
 * Adapted from http://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html
 * "Simple Stupid Funnel Algorithm" by Mikko Mononen.
 */
public final class NavMeshStringPuller {

    /** Default distance in world units allowed for distance squared equality */
    private static final float DEFAULT_V_EQ_2 = 0.001f * 0.001f;

    private static float triArea2(Vector2 a, Vector2 b, Vector2 c) {
        final float ax = b.x - a.x;
        final float ay = b.y - a.y;
        final float bx = c.x - a.x;
        final float by = c.y - a.y;
        return bx * ay - ax * by;
    }

    /**
     * Convert a graph path to a list of portals to pass through. The first and last portals are the starting
     * and ending positions. Repeated path points that occur directly one after the other will be removed.
     *
     * @param path graph solution
     * @return list of portals to pass through, where the first and last portal are start and end points respectively
     */
    public NavMeshPortal[] pathToPortals(DefaultGraphPath<NavMeshPathNode> path) {

        int portalCount = 0;
        for(int i = 0; i < path.getCount(); ++i) {

            if(i > 0) {

                // Ignore duplicate points
                if(!vEqual(path.get(i).getPortal().getMidpoint(), path.get(i - 1).getPortal().getMidpoint())) {
                    ++portalCount;
                }

            } else {
                ++portalCount;
            }
        }

        final NavMeshPortal[] portals = new NavMeshPortal[portalCount];

        // Copy portal list
        int portalIndex = 0;
        for(int i = 0; i < path.getCount(); ++i) {

            if(portalIndex > 0) {

                // Ignore duplicate points
                if(!vEqual(path.get(i).getPortal().getMidpoint(), path.get(i - 1).getPortal().getMidpoint())) {
                    portals[portalIndex++] = path.get(i).getPortal();
                }

            } else {
                portals[portalIndex++] = path.get(i).getPortal();
            }
        }

        return portals;
    }


    /** Max allowed value for distance squared equality */
    private final float eq;

    //
    // Reuse vectors
    //
    private final Vector2 portalApex, portalLeft, portalRight, offset, left, right;
    private int apexIndex, leftIndex, rightIndex;

    /**
     * Create string puller with default vertex equal squared distance {@link #DEFAULT_V_EQ_2}.
     */
    public NavMeshStringPuller() {
        this(DEFAULT_V_EQ_2);
    }

    /**
     * Create string puller with the provided vertex equal squared distance.
     *
     * @param vEqualDist2 maximum distance squared between two vertices to be considered equal
     */
    public NavMeshStringPuller(float vEqualDist2) {
        this.eq = vEqualDist2;
        this.portalApex = new Vector2();
        this.portalLeft = new Vector2();
        this.portalRight = new Vector2();
        this.offset = new Vector2();
        this.left = new Vector2();
        this.right = new Vector2();
    }

    /**
     * @param portal0 first portal
     */
    private void initScanState(NavMeshPortal portal0) {
        portalApex.set(portal0.getLeft());
        portalLeft.set(portal0.getLeft());
        portalRight.set(portal0.getRight());
        apexIndex = 0;
        leftIndex = 0;
        rightIndex = 0;
    }

    private void resetPortal() {
        portalLeft.set(portalApex);
        portalRight.set(portalApex);
        leftIndex = apexIndex;
        rightIndex = apexIndex;
    }

    /**
     * Determine if vectors are equal based on threshold {@link #eq}
     *
     * @param a first position vector
     * @param b second position vector
     * @return true if distance less than threshold {@link #eq}
     */
    private boolean vEqual(Vector2 a, Vector2 b) {
        return a.dst2(b) < eq;
    }

    /**
     * String pull from start position to end position using a list of portals to pass through and an agent radius.
     * The String pulling will offset distance using the agent radius within each portal.
     *
     * @param startPos starting search position
     * @param endPos ending search position
     * @param portals calculated using pathToPortals()
     * @param agentRadius agent collision radius in world units
     * @return list of string-pulled waypoints from starting position to ending position
     */
    public Array<Vector2> stringPull(Vector2 startPos, Vector2 endPos, NavMeshPortal[] portals, float agentRadius) {
        final Array<Vector2> pathPts = new Array<>();

        // Init scan state
        initScanState(portals[0]);

        // Initial case: add start point
        pathPts.add(new Vector2(startPos)); // TODO: Remove allocation

        // 1...N case:
        float triAreaRight, triAreaLeft;
        boolean isFinalPortal;

        for(int i = 1; i < portals.length; ++i) {
            isFinalPortal = (i == portals.length - 1);

            if(isFinalPortal) {
                offset.setZero();
            } else {
                offset.set(agentRadius, 0f).setAngle(portals[i].getLeftIntAng());
            }
            left.set(portals[i].getLeft()).add(offset);

            if(isFinalPortal) {
                offset.setZero();
            } else {
                offset.set(agentRadius, 0f).setAngle(portals[i].getRightIntAng());
            }
            right.set(portals[i].getRight()).add(offset);


            triAreaRight = triArea2(portalApex, portalRight, right);
            triAreaLeft = triArea2(portalApex, portalLeft, left);

            // Update right vertex
            if(triAreaRight <= 0.0f) {

                if(vEqual(portalApex, portalRight) || triArea2(portalApex, portalLeft, right) > 0.0f) {

                    // Tighten the funnel
                    portalRight.set(right);
                    rightIndex = i;

                } else {

                    // Right over left, insert left to path and restart scan from portal left point
                    pathPts.add(new Vector2(portalLeft)); // TODO: Remove Vector Allocation

                    // Make current left the new apex
                    portalApex.set(portalLeft);
                    apexIndex = leftIndex;

                    // Reset portal
                    resetPortal();

                    // Restart scan
                    i = apexIndex;
                    continue;
                }
            }


            // Update left vertex
            if(triAreaLeft >= 0.0f) {
                if (vEqual(portalApex, portalLeft) || triArea2(portalApex, portalRight, left) < 0.0f) {

                    // Tighten the funnel
                    portalLeft.set(left);
                    leftIndex = i;

                } else {

                    // Left over right, insert right to path and restart scan from portal right point
                    pathPts.add(new Vector2(portalRight)); // TODO: Remove Vector Allocation

                    // Make current right the new apex
                    portalApex.set(portalRight);
                    apexIndex = rightIndex;

                    // Reset portal
                    resetPortal();

                    // Restart scan
                    i = apexIndex;
                    continue;
                }
            }
        }


        // Append last point to path if it's not a duplicate
        if(!endPos.equals(pathPts.get(pathPts.size-1))) {
            pathPts.add(new Vector2(endPos)); // TODO: Remove allocation
        }

        return pathPts;
    }
}
