package com.shibabandit.gdx_navmesh.coll;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Pool;
import org.locationtech.jts.algorithm.LineIntersector;
import org.locationtech.jts.algorithm.RobustLineIntersector;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

/**
 * Find intersections between lines, from points.
 */
public final class IntersectionFinder {

    /**
     * Results from intersection test.
     */
    public static class InterResults implements Pool.Poolable {

        private static final InterResults FAILURE;
        static {
            FAILURE = new InterResults();
            FAILURE.interFound = false;
            FAILURE.dist = -1d;
            FAILURE.interPt.setZero();
        }

        public InterResults() {
            interPt = new Vector2();
            reset();
        }

        /** True if an intersection was found */
        public boolean interFound;

        /** Distance to the intersection */
        public double dist;

        /** Point of intersection */
        public final Vector2 interPt;

        @Override
        public void reset() {
            interFound = false;
            dist = -1d;
            interPt.setZero();
        }

        /**
         * Set values in this object to match input.
         *
         * @param interResults values to apply
         */
        public void set(InterResults interResults) {
            interFound = interResults.interFound;
            dist = interResults.dist;
            interPt.set(interResults.interPt);
        }

        /**
         * Set the results to a failure state.
         */
        public void setFailed() {
            set(FAILURE);
        }
    }

    private Vector2 ve2v2;
    private Coordinate ce1v1, ce1v2, ce2v1, ce2v2;
    private LineIntersector li;
    private InterResults nextResult;

    public IntersectionFinder() {
        ce1v1 = new Coordinate();
        ce1v2 = new Coordinate();
        ce2v1 = new Coordinate();
        ce2v2 = new Coordinate();
        li = new RobustLineIntersector();
        ve2v2 = new Vector2();
        nextResult = new InterResults();
    }

    public void findClosestInter(Coordinate c, float angle, float dist, Polygon polygon, InterResults results) {
        findClosestInter(
                (float)c.getOrdinate(0),
                (float)c.getOrdinate(1),
                angle, dist, polygon, results);
    }

    public void findClosestInter(float x, float y, float angle, float dist, Polygon polygon, InterResults results) {

        setOrds(x, y, ce2v1);
        setOrds(ve2v2.set(dist, 0f).setAngle(angle).add(x, y), ce2v2);

        results.reset();

        // Define rotating line segment
        final LineString shell = polygon.getExteriorRing();
        final Coordinate[] coords = shell.getCoordinates();
        for(int i = 0; i < coords.length; ++i) {
            ce1v1.setCoordinate(coords[i]);
            ce1v2.setCoordinate(coords[(i + 1) % coords.length]);

            findInter(nextResult);
            if(nextResult.interFound && (nextResult.dist < results.dist || !results.interFound)) {
                results.set(nextResult);
            }
        }
    }

    public void findInter(float e1v1x, float e1v1y, float e1v2x, float e1v2y,
                          float e2v1x, float e2v1y, float e2v2x, float e2v2y,
                          InterResults results) {
        setOrds(e1v1x, e1v1y, ce1v1);
        setOrds(e1v2x, e1v2y, ce1v2);
        setOrds(e2v1x, e2v1y, ce2v1);
        setOrds(e2v2x, e2v2y, ce2v2);

        results.reset();

        findInter(results);
    }

    public void findInter(InterResults results) {
        results.interFound = false;

        li.computeIntersection(ce1v1, ce1v2, ce2v1, ce2v2);

        if(li.hasIntersection()) {
            results.interFound = true;

            final Coordinate inter = li.getIntersection(0);
            results.interPt.set((float)inter.x, (float)inter.y);

            results.dist = li.getEdgeDistance(1, 0); // TODO: Do we want to do dist every time..?
        }
    }

    /**
     * Set ordinate values in coordinate c from vector v. Return modified
     * coordinate c for chaining.
     *
     * @param v contains values to apply to c
     * @param c coordinate to modify and return
     * @return modified coordinate c, with values from v
     */
    public Coordinate setOrds(Vector2 v, Coordinate c) {
        c.setOrdinate(0, v.x);
        c.setOrdinate(1, v.y);
        return c;
    }

    /**
     * Set ordinate values in coordinate c from provided x and y. Return modified
     * coordinate c for chaining.
     *
     * @param x x coordinate value to apply
     * @param y y coordiante value to apply
     * @param c coordinate to modify and return
     * @return modified coordinate c, with values from v
     */
    public Coordinate setOrds(float x, float y, Coordinate c) {
        c.setOrdinate(0, x);
        c.setOrdinate(1, y);
        return c;
    }
}
