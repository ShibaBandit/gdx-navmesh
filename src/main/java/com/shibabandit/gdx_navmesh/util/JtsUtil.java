package com.shibabandit.gdx_navmesh.util;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.utils.Array;
import com.shibabandit.gdx_navmesh.path.NavMeshClipper;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryCollection;

import java.util.Stack;

/**
 * JTS utility functions.
 */
public final class JtsUtil {

    /**
     * Convert a JTS such as a {@link GeometryCollection} to a flat list of {@link Geometry} Polygons.
     * Non-polygon geometry are ignored and an error message is printed.
     *
     * @param geom JTS Geometry to flatten
     * @return flat list of Polygons
     */
    public static Array<Geometry> flatList(Geometry geom) {
        final Array<Geometry> simpleJtsPolys = new Array<>();
        final Stack<Geometry> geomStack = new Stack<>();

        Geometry nextGeom;
        int nextGeomCount;

        geomStack.push(geom);
        while(!geomStack.isEmpty()) {
            nextGeom = geomStack.pop();

            if(nextGeom instanceof GeometryCollection) {

                // Push all child geometries
                nextGeomCount = geom.getNumGeometries();
                for(int i = 0; i < nextGeomCount; ++i) {
                    geomStack.push(nextGeom.getGeometryN(i));
                }

            } else if(nextGeom instanceof org.locationtech.jts.geom.Polygon) {
                org.locationtech.jts.geom.Polygon jtsPoly = (org.locationtech.jts.geom.Polygon) nextGeom;
                simpleJtsPolys.add(jtsPoly);

            } else {
                Gdx.app.error(NavMeshClipper.class.getName(), "Unhandled geometry type: " + nextGeom.getClass().getCanonicalName());
            }
        }

        return simpleJtsPolys;
    }
}
