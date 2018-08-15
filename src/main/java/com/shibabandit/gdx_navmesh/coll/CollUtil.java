package com.shibabandit.gdx_navmesh.coll;

import com.shibabandit.gdx_navmesh.path.NavMeshPathNode;
import org.locationtech.jts.geom.*;
import org.poly2tri.triangulation.TriangulationPoint;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

import java.util.List;

public final class CollUtil {
    private static final GeometryFactory GF = new GeometryFactory();

    public static Polygon toJtsPoly(org.poly2tri.geometry.polygon.Polygon polygon) {
        final List<TriangulationPoint> points = polygon.getPoints();

        final Coordinate[] jtsCoords = new Coordinate[points.size() + 1]; // JTS likes to repeat the last point
        for(int i = 0; i < points.size(); ++i){
            jtsCoords[i] = new Coordinate(points.get(i).getX(), points.get(i).getY());
        }

        // Link the first and last points
        jtsCoords[jtsCoords.length - 1] = new Coordinate(points.get(0).getX(), points.get(0).getY());

        final LinearRing jtsLinearRing = GF.createLinearRing(jtsCoords);
        return new Polygon(jtsLinearRing, null, GF);
    }

    public static Envelope setIndexEnvelope(NavMeshPathNode navMeshPathNode, Envelope envelope) {
        final DelaunayTriangle dt = navMeshPathNode.getDelaunayTriangle();
        TriangulationPoint p;

        envelope.init();

        for(int pi = 0; pi < dt.points.length; ++pi) {
            p = dt.points[pi];
            envelope.expandToInclude(p.getX(), p.getY());
        }

        return envelope;
    }
}
