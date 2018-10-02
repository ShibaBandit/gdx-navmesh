package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.shibabandit.gdx_navmesh.coll.CollUtil;
import com.shibabandit.gdx_navmesh.coll.QtItem;
import com.shibabandit.gdx_navmesh.coll.QtSearchIndex;
import com.shibabandit.gdx_navmesh.util.Angles;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.index.quadtree.Quadtree;
import org.poly2tri.geometry.polygon.Polygon;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static com.shibabandit.gdx_navmesh.util.DelaunayTriangleUtil.*;

/**
 * Indexed navmesh graph. Supports fast lookup using quadtree index.
 */
public class NavMeshGraph implements INavMeshGraph<NavMeshPathNode> {


    public static final class QtTriNode implements QtItem {
        protected final DelaunayTriangle dt;

        /** Navmesh nodes associated with {@link #dt} */
        protected final Array<NavMeshPathNode> nodes;

        /** Bounding envelope for {@link #dt} */
        protected final Envelope envelope;

        public QtTriNode(DelaunayTriangle dt, Array<NavMeshPathNode> nodes) {
            this.dt = dt;
            this.nodes = nodes;
            this.envelope = new Envelope();
            CollUtil.setIndexEnvelope(dt, envelope);
        }

        public DelaunayTriangle getDt() {
            return dt;
        }

        public Array<NavMeshPathNode> getNodes() {
            return nodes;
        }

        @Override
        public Envelope getEnvelope() {
            return envelope;
        }

        public NavMeshPathNode closestPathNodeTo(Vector2 pos) {
            float leastDstSqd = Float.MAX_VALUE;
            float nextDstSqd;
            NavMeshPathNode bestNode = null;

            for(NavMeshPathNode nextNode : nodes) {
                nextDstSqd = pos.dst2(nextNode.getPortal().getMidpoint());
                if(nextDstSqd < leastDstSqd) {
                    bestNode = nextNode;
                    leastDstSqd = nextDstSqd;
                }
            }

            return bestNode;
        }

        @Override
        public String toString() {
            return "QtTriNode{" +
                    "dt=" + dt +
                    ", nodes=" + nodes +
                    ", envelope=" + envelope +
                    '}';
        }
    }


    /** Maximum nodes are related to the 3 edges of a triangle */
    private static final int MAX_NODES_PER_TRI = 3;

    /** Up to 4 connections per node (the edges of 2 bordering triangles, minus their shared edge) */
    private static final int MAX_CONNS_PER_NODE = 4;



    /** Indexed list of graph nodes */
    protected Array<NavMeshPathNode> nodes;

    /** Stores the next index to use for an indexed node */
    protected int nextIndex;

    /** Lookup for the navmesh node given a {@link NavMeshPortal} */
    protected HashMap<NavMeshPortal, NavMeshPathNode> portalToNode;

    /** Stores navmesh nodes for spatially indexed lookup */
    protected final QtSearchIndex<QtTriNode> nodesQt;

    /**
     * Build the navigation graph using walkable triangles from Delaunay triangulation. This pre-initializes
     * all navmesh portals and connections. After the graph is built, a quadtree index is created for quickly
     * searching for graph nodes.
     *
     * @param walkablePolys flat list of walkable polygons
     */
    public NavMeshGraph(Array<Polygon> walkablePolys) {
        this.nodesQt = new QtSearchIndex<>();
        buildGraph(walkablePolys);
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
            portalToNode = new HashMap<>(0);
            return;
        }


        portalToNode = new HashMap<>(triangles.size() * MAX_NODES_PER_TRI);
        nodes = new Array<>(triangles.size() * MAX_NODES_PER_TRI);

        DelaunayTriangle tNeighborI;
        NavMeshPortal nextPortal, nextNeighborPortal;
        NavMeshPathNode nextNode;
        int tNeighborIndices[] = new int[2];
        NavMeshPathNode nextNeighborNode;

        QtTriNode nextQtTriNode;
        final Quadtree qt = nodesQt.getQt();


        for(DelaunayTriangle t : triangles) {

            // Add triangle to spatial index
            nextQtTriNode = new QtTriNode(t, new Array<>(MAX_CONNS_PER_NODE));
            qt.insert(nextQtTriNode.getEnvelope(), nextQtTriNode);


            for(int i = 0; i < t.neighbors.length; ++i) {

                nextPortal = getPortal(t, i);
                if(nextPortal != null) {
                    tNeighborI = t.neighbors[i];

                    nextNode = getNodeAndCreate(nextPortal);
                    nextNode.setDtA(t);
                    nextNode.setDtB(tNeighborI);

                    // Spatial index reference to nextPortal
                    nextQtTriNode.getNodes().add(nextNode);

                    // Other portal connections in t
                    tNeighborIndices[0] = (i + 1) % 3;
                    tNeighborIndices[1] = (i + 2) % 3;
                    for(int j = 0; j < tNeighborIndices.length; ++j) {
                        if((nextNeighborPortal = getPortal(t, j)) != null) {
                            nextNeighborNode = getNodeAndCreate(nextNeighborPortal);
                            nextNode.connections.add(new NavMeshPathConn(nextNode, nextNeighborNode));

                            // Spatial index reference to neighbor portal
                            nextQtTriNode.getNodes().add(nextNeighborNode);
                        }
                    }

                    // Portal connections in tNeighborI, must check equality to avoid self-connection (shared edge)
                    for(int j = 0; j < 3; ++j) {
                        if((nextNeighborPortal = getPortal(tNeighborI, j)) != null
                                && !nextNeighborPortal.equals(nextPortal)) {
                            nextNeighborNode = getNodeAndCreate(nextNeighborPortal);
                            nextNode.connections.add(new NavMeshPathConn(nextNode, nextNeighborNode));
                        }
                    }
                }
            }

            // Check for triangle islands
            nextNode = getNodeAndCreate(getIslandPortal(t));
            nextNode.setDtA(t);
            nextNode.setDtB(null);
            if(nextQtTriNode.getNodes().size < 1) {
                nextQtTriNode.getNodes().add(nextNode);
            }
        }
    }


    private static final Vector2 PVEC_1 = new Vector2(),
            PVEC_2 = new Vector2(),
            CENTROID = new Vector2();

    /**
     * Gets navmesh portal for the given neighbor index, or null if it is not a valid delaunay edge.
     *
     * @param dt reference delaunay triangle
     * @param neighborIndex the edge/neighbor index to get a portal for
     * @return valid navmesh portal to another neighbor on a delaunay edge, or null if not valid
     */
    private static NavMeshPortal getPortal(DelaunayTriangle dt, int neighborIndex) {
        NavMeshPortal result = null;
        float v1AngDeg, v2AngDeg;

        if(dtGetNeighborEdgeIfDelaunay(dt, neighborIndex, PVEC_1, PVEC_2)) {

            result = new NavMeshPortal();

            // Calculate CENTROID of t
            dtCentroid(dt, CENTROID);

            // Left and right portals can be determined by looking at the angles relative
            // to the CENTROID. If the rotation angle from PVEC_1 to PVEC_2 is positive (CCW), PVEC_2
            // is to the right of PVEC_1 from the standpoint of the CENTROID.
            v1AngDeg = Angles.between(CENTROID, PVEC_1);
            v2AngDeg = Angles.between(CENTROID, PVEC_2);

            if(Angles.isShortRotCCW(v1AngDeg, v2AngDeg)) {
                result.init(PVEC_2, PVEC_1);
            } else {
                result.init(PVEC_1, PVEC_2);
            }
        }

        return result;
    }

    /**
     * Gets a fake island portal in the center of the triangle.
     *
     * @param dt reference delaunay triangle
     * @return fake navmesh portal at triangle centroid
     */
    public static NavMeshPortal getIslandPortal(DelaunayTriangle dt) {
        NavMeshPortal result = new NavMeshPortal();

        // Calculate CENTROID of t
        dtCentroid(dt, CENTROID);

        result.init(CENTROID, CENTROID);

        return result;
    }

    @Override
    public NavMeshPathNode getNode(NavMeshPortal portal) {
        return portalToNode.get(portal);
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
     * Convenience method for creating a {@link NavMeshPathNode}. Avoids creating
     * new path node if it already exists.
     *
     * @param portal
     * @return
     */
    protected NavMeshPathNode getNodeAndCreate(NavMeshPortal portal) {
        NavMeshPathNode node = getNode(portal);

        if(node == null) {
            node = new NavMeshPathNode(nextIndex++, portal);
            nodes.add(node);
            portalToNode.put(portal, node);
        }

        return node;
    }

    /**
     * @return all path nodes in the graph
     */
    public Array<NavMeshPathNode> getNodes() {
        return nodes;
    }

    /**
     * @return quadtree index for spatially indexed lookup of path nodes based on triangles
     */
    public QtSearchIndex<QtTriNode> getNodesQt() {
        return nodesQt;
    }


}
