package com.shibabandit.gdx_navmesh.util;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.poly2tri.geometry.polygon.Polygon;
import org.poly2tri.geometry.polygon.PolygonPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Convenience class for converting to Poly2Tri {@link org.poly2tri.geometry.polygon.Polygon} instances.
 */
public final class Poly2TriPolygonFactory {

    /**
     * Convert JTS geometry potentially containing holes to a poly2tri Polygon.
     *
     * @param jtsGeom JTS polygon geometry to convert
     * @return poly2tri polygon for triangulation
     */
    public static Polygon fromJtsPoly(Geometry jtsGeom) {
        Polygon poly;

        if(jtsGeom instanceof org.locationtech.jts.geom.Polygon) {
            org.locationtech.jts.geom.Polygon jtsPoly = (org.locationtech.jts.geom.Polygon)jtsGeom;

            poly = fromJtsGeomPts(jtsPoly.getExteriorRing());

            for(int i = 0; i < jtsPoly.getNumInteriorRing(); ++i) {
                LineString ringN = jtsPoly.getInteriorRingN(i);
                poly.addHole(fromJtsGeomPts(ringN));
            }

        } else {
            poly = fromJtsGeomPts(jtsGeom);
        }

        return poly;
    }

    /**
     * Convert a JTS polygon ring to a poly2tri Polygon.
     *
     * @param jtsGeom JTS polygon geometry to convert
     * @return poly2tri polygon for triangulation
     */
    private static Polygon fromJtsGeomPts(Geometry jtsGeom) {
        final Coordinate[] jtsCoords = jtsGeom.getCoordinates();
        final List<PolygonPoint> polyPts = new ArrayList<>(jtsCoords.length - 1);

        for(int i = 0; i < jtsCoords.length - 1; ++i) {
            polyPts.add(new PolygonPoint(jtsCoords[i].x, jtsCoords[i].y));
        }

        return new Polygon(polyPts);
    }

}
