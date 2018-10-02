package com.shibabandit.gdx_navmesh.util;

import com.badlogic.gdx.math.Vector2;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

/**
 * Utility class for poly2tri DelaunayTriangle.
 */
public final class DelaunayTriangleUtil {

    /**
     * Modifies input vector to contain the centroid of '{@code dt}' and returns the value for chaining.
     *
     * @param dt the triangle to calculate a centroid for
     * @param v result container
     * @return input vector v with result for chaining
     */
    public static Vector2 dtCentroid(DelaunayTriangle dt, Vector2 v) {
        float cx = ( dt.points[0].getXf() + dt.points[1].getXf() + dt.points[2].getXf() ) / 3f;
        float cy = ( dt.points[0].getYf() + dt.points[1].getYf() + dt.points[2].getYf() ) / 3f;

        return v.set(cx, cy);
    }

    /**
     * Returns whether an x, y pair is contained within the polygon of '{@code dt}'.
     *
     * @param dt the triangle to check if (x,y) is contained
     * @param x world x coordinate
     * @param y world y coordinate
     * @return true if point (x,y) is within the triangle's area
     */
    public static boolean dtContains(DelaunayTriangle dt, float x, float y) {
        int intersects = 0;

        for(int i = 0; i < dt.points.length; ++i) {

            float x1 = dt.points[i].getXf();
            float y1 = dt.points[i].getYf();

            float x2 = dt.points[(i + 1) % dt.points.length].getXf();
            float y2 = dt.points[(i + 1) % dt.points.length].getYf();

            if (((y1 <= y && y < y2) || (y2 <= y && y < y1)) && x < ((x2 - x1) / (y2 - y1) * (y - y1) + x1)) intersects++;
        }

        return (intersects & 1) == 1;
    }

    public static boolean dtGetEdge(DelaunayTriangle dt, int index, Vector2 ptA, Vector2 ptB) {
        boolean success = false;

        // Edges:
        //
        // edge[0] = Edge(points[1], points[2])
        // edge[1] = Edge(points[0], points[2])
        // edge[2] = Edge(points[0], points[1])

        switch (index) {
            case 0:
                if(dt.points[1] != null && dt.points[2] != null) {
                    ptA.set(dt.points[1].getXf(), dt.points[1].getYf());
                    ptB.set(dt.points[2].getXf(), dt.points[2].getYf());
                    success = true;
                }
                break;
            case 1:
                if(dt.points[0] != null && dt.points[2] != null) {
                    ptA.set(dt.points[0].getXf(), dt.points[0].getYf());
                    ptB.set(dt.points[2].getXf(), dt.points[2].getYf());
                    success = true;
                }
                break;
            case 2:
                if(dt.points[0] != null && dt.points[1] != null) {
                    ptA.set(dt.points[0].getXf(), dt.points[0].getYf());
                    ptB.set(dt.points[1].getXf(), dt.points[1].getYf());
                    success = true;
                }
                break;
        }

        return success;
    }

    public static boolean dtGetEdgeIfDelaunay(DelaunayTriangle dt, int index, Vector2 ptA, Vector2 ptB) {
        boolean success = false;

        // Edges:
        //
        // edge[0] = Edge(points[1], points[2])
        // edge[1] = Edge(points[0], points[2])
        // edge[2] = Edge(points[0], points[1])

        switch (index) {
            case 0:
                if(dt.points[1] != null && dt.points[2] != null && dt.dEdge[0]) {
                    ptA.set(dt.points[1].getXf(), dt.points[1].getYf());
                    ptB.set(dt.points[2].getXf(), dt.points[2].getYf());
                    success = true;
                }
                break;
            case 1:
                if(dt.points[0] != null && dt.points[2] != null && dt.dEdge[1]) {
                    ptA.set(dt.points[0].getXf(), dt.points[0].getYf());
                    ptB.set(dt.points[2].getXf(), dt.points[2].getYf());
                    success = true;
                }
                break;
            case 2:
                if(dt.points[0] != null && dt.points[1] != null && dt.dEdge[2]) {
                    ptA.set(dt.points[0].getXf(), dt.points[0].getYf());
                    ptB.set(dt.points[1].getXf(), dt.points[1].getYf());
                    success = true;
                }
                break;
        }

        return success;
    }

    public static boolean dtGetNeighborEdge(DelaunayTriangle dt, int index, Vector2 ptA, Vector2 ptB) {
        boolean success = false;

        if(dt.neighbors[index] != null) {

            // Neighbor edges:
            //
            // neighbors[0] = Edge(points[1], points[2])
            // neighbors[1] = Edge(points[0], points[2])
            // neighbors[2] = Edge(points[0], points[1])

            switch (index) {
                case 0:
                    if(dt.points[1] != null && dt.points[2] != null) {
                        ptA.set(dt.points[1].getXf(), dt.points[1].getYf());
                        ptB.set(dt.points[2].getXf(), dt.points[2].getYf());
                        success = true;
                    }
                    break;
                case 1:
                    if(dt.points[0] != null && dt.points[2] != null) {
                        ptA.set(dt.points[0].getXf(), dt.points[0].getYf());
                        ptB.set(dt.points[2].getXf(), dt.points[2].getYf());
                        success = true;
                    }
                    break;
                case 2:
                    if(dt.points[0] != null && dt.points[1] != null) {
                        ptA.set(dt.points[0].getXf(), dt.points[0].getYf());
                        ptB.set(dt.points[1].getXf(), dt.points[1].getYf());
                        success = true;
                    }
                    break;
            }
        }

        return success;
    }

    public static boolean dtGetNeighborEdgeIfDelaunay(DelaunayTriangle dt, int index, Vector2 ptA, Vector2 ptB) {
        boolean success = false;

        if(dt.neighbors[index] != null && dt.neighbors[index].isInterior()) {

            // Neighbor edges:
            //
            // neighbors[0] = Edge(points[1], points[2])
            // neighbors[1] = Edge(points[0], points[2])
            // neighbors[2] = Edge(points[0], points[1])

            switch (index) {
                case 0:
                    if(dt.points[1] != null && dt.points[2] != null && !dt.cEdge[0]) {
                        ptA.set(dt.points[1].getXf(), dt.points[1].getYf());
                        ptB.set(dt.points[2].getXf(), dt.points[2].getYf());
                        success = true;
                    }
                    break;
                case 1:
                    if(dt.points[0] != null && dt.points[2] != null && !dt.cEdge[1]) {
                        ptA.set(dt.points[0].getXf(), dt.points[0].getYf());
                        ptB.set(dt.points[2].getXf(), dt.points[2].getYf());
                        success = true;
                    }
                    break;
                case 2:
                    if(dt.points[0] != null && dt.points[1] != null && !dt.cEdge[2]) {
                        ptA.set(dt.points[0].getXf(), dt.points[0].getYf());
                        ptB.set(dt.points[1].getXf(), dt.points[1].getYf());
                        success = true;
                    }
                    break;
            }
        }

        return success;
    }
}
