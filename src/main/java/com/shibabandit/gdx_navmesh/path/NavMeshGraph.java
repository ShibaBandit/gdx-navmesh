package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pools;
import com.shibabandit.gdx_navmesh.coll.QtSearchIndex;
import com.shibabandit.gdx_navmesh.util.Angles;
import org.locationtech.jts.index.quadtree.Quadtree;
import org.poly2tri.geometry.polygon.Polygon;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static com.shibabandit.gdx_navmesh.util.DelaunayTriangleUtil.centroid;
import static com.shibabandit.gdx_navmesh.util.DelaunayTriangleUtil.dtGetNeighborEdge;

/**
 * Indexed navmesh graph. Supports fast lookup using quadtree index.
 */
public class NavMeshGraph implements INavMeshGraph<NavMeshPathNode> {

    /** Indexed list of graph nodes */
    protected Array<NavMeshPathNode> nodes;

    /** Stores the next index to use for an indexed node */
    protected int nextIndex;

    /** Lookup for the navmesh node given a {@link DelaunayTriangle} */
    protected HashMap<DelaunayTriangle, NavMeshPathNode> triToNode;

    /** Stores navmesh nodes for indexed lookup */
    protected final QtSearchIndex<NavMeshPathNode> nodesQt;

    /**
     * Build the navigation graph using walkable triangles from delaunay triangulation. This pre-initializes
     * all navmesh portals and connections. After the graph is built, a quadtree index is created for quickly
     * searching for graph nodes.
     *
     * @param walkablePolys flat list of walkable polygons
     */
    public NavMeshGraph(Array<Polygon> walkablePolys) {
        buildGraph(walkablePolys);

        // Build spatial index for node lookup
        nodesQt = buildQtIndex();
    }

    /**
     * Build the navigation graph using walkable triangles from delaunay triangulation. This pre-initializes
     * all navmesh portals and connections.
     *
     * @param walkablePolys flat list of walkable polygons
     */
    protected void buildGraph(Array<Polygon> walkablePolys) {

        // Aggregate walkable polygon triangles into a flat list
        int triangleCount = 0;
        for(Polygon p : walkablePolys) {
            if(p.getTriangles() != null) {
                for(DelaunayTriangle dt : p.getTriangles()) {
                    if(dt.isInterior()) {
                        ++triangleCount;
                    }
                }
            }
        }
        final List<DelaunayTriangle> triangles = new ArrayList<>(triangleCount);
        for(Polygon p : walkablePolys) {
            if(p.getTriangles() != null) {
                for(DelaunayTriangle dt : p.getTriangles()) {
                    if(dt.isInterior()) {
                        triangles.add(dt);
                    }
                }
            }
        }

        // Guard: no triangle case
        if(triangles.size() < 1) {
            nodes = new Array<>(0);
            nextIndex = 0;
            triToNode = new HashMap<>(0);
            return;
        }


        nodes = new Array<>(triangles.size());
        nextIndex = 0;
        triToNode = new HashMap<>(triangles.size());

        NavMeshPathNode nextNode;
        NavMeshPortal nextPortal;
        Vector2 pvec1 = new Vector2(), pvec2 = new Vector2(), centroid = new Vector2();
        float v1AngDeg, v2AngDeg;

        // Nodes and connections for all triangles
        for(DelaunayTriangle t : triangles) {
            nextNode = getNodeAndCreate(t);

            // connections
            for(int i = 0; i < t.neighbors.length; ++i) {
                if(t.neighbors[i] == null || !t.neighbors[i].isInterior()) {
                    continue;
                }

                if(dtGetNeighborEdge(t, i, pvec1, pvec2)) {

                    nextPortal = Pools.get(NavMeshPortal.class).obtain();

                    // Left and right portals can be determined by looking at the angles relative
                    // to the centroid. If the rotation angle from pvec1 to pvec2 is positive (CCW), pvec2
                    // is to the right of pvec1 from the standpoint of the centroid.
                    centroid(t, centroid);
                    v1AngDeg = Angles.between(centroid, pvec1);
                    v2AngDeg = Angles.between(centroid, pvec2);

                    if(Angles.isShortRotCCW(v1AngDeg, v2AngDeg)) {
                        nextPortal.init(pvec2, pvec1);
                    } else {
                        nextPortal.init(pvec1, pvec2);
                    }

                    nextNode.getConnections().add(new NavMeshPathConn(nextNode, getNodeAndCreate(t.neighbors[i]), nextPortal));

                } else {
                    Gdx.app.error(NavMeshGraph.class.getName(),
                            "Failed to get neighbor portal during graph creation, dropping edge");
                }
            }
        }
    }

    /**
     * @return quadtree initialized with all graph nodes using their triangle area
     */
    protected QtSearchIndex<NavMeshPathNode> buildQtIndex() {
        QtSearchIndex<NavMeshPathNode> qtIndex = new QtSearchIndex<>();
        final Quadtree qt = qtIndex.getQt();
        NavMeshPathNode nextNode;

        for(int ni = 0; ni < nodes.size; ++ ni) {
            nextNode = nodes.get(ni);
            qt.insert(nextNode.getEnvelope(), nextNode);
        }
        return qtIndex;
    }

    @Override
    public NavMeshPathNode getNode(DelaunayTriangle triangle) {
        return triToNode.get(triangle);
    }

    @Override
    public int getIndex(NavMeshPathNode node) {
        return node.getIndex();
    }

    @Override
    public int getNodeCount() {
        return nodes.size;
    }

    @Override
    public Array<Connection<NavMeshPathNode>> getConnections(NavMeshPathNode fromNode) {
        return nodes.get(fromNode.getIndex()).getConnections();
    }

    /**
     * Convenience method for creating a {@link NavMeshPathNode}.
     *
     * @param triangle
     * @return
     */
    protected NavMeshPathNode getNodeAndCreate(DelaunayTriangle triangle) {
        NavMeshPathNode node = getNode(triangle);

        if(node == null) {

            // Add tile to graph
            node = new NavMeshPathNode(nextIndex++, triangle);
            nodes.add(node);
            triToNode.put(triangle, node);
        }

        return node;
    }

    public Array<NavMeshPathNode> getNodes() {
        return nodes;
    }
}
