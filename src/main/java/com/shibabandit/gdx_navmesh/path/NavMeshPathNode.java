package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.utils.Array;
import com.shibabandit.gdx_navmesh.coll.CollUtil;
import com.shibabandit.gdx_navmesh.coll.QtItem;
import org.locationtech.jts.geom.Envelope;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

/**
 * Navmesh path node that supports {@link org.locationtech.jts.index.quadtree.Quadtree} search and contains a
 * Delaunay Triangle. Fields left protected to support inheritance.
 *
 * @see IndexedNavMeshAStarPathFinder
 * @see org.locationtech.jts.index.quadtree.Quadtree
 * @see QtItem
 * @see DelaunayTriangle
 */
public class NavMeshPathNode implements QtItem {

    /** Connections to neighboring navmesh nodes that share a walkable edge */
    protected final Array<Connection<NavMeshPathNode>> connections;

    /** Global graph index for fast lookup */
    protected final int index;

    /** The triangle geometry represented by this path node */
    protected final DelaunayTriangle delaunayTriangle;

    /** Bounding envelope for {@link #delaunayTriangle} */
    protected final Envelope envelope;

    /**
     * @param index global graph index for fast lookup
     * @param delaunayTriangle the triangle geometry represented by this path node
     */
    public NavMeshPathNode(int index, DelaunayTriangle delaunayTriangle) {

        // Triangles will have a maximum of 3 shared edges
        this(new Array<>(3), index, delaunayTriangle);
    }

    /**
     * @param connections connections to neighboring navmesh nodes that share a walkable edge
     * @param index global graph index for fast lookup
     * @param delaunayTriangle the triangle geometry represented by this path node
     */
    public NavMeshPathNode(Array<Connection<NavMeshPathNode>> connections, int index, DelaunayTriangle delaunayTriangle) {
        this.connections = connections;
        this.index = index;
        this.delaunayTriangle = delaunayTriangle;
        this.envelope = new Envelope();
        CollUtil.setIndexEnvelope(this, envelope);
    }

    /**
     * @return global graph index for fast lookup
     * @see IndexedNavMeshAStarPathFinder
     */
    public int getIndex() {
        return index;
    }

    /**
     * @return connections to neighboring navmesh nodes that share a walkable edge
     */
    public Array<Connection<NavMeshPathNode>> getConnections() {
        return connections;
    }

    /**
     * @return the triangle geometry represented by this path node
     */
    public DelaunayTriangle getDelaunayTriangle() {
        return delaunayTriangle;
    }

    @Override
    public Envelope getEnvelope() {
        return envelope;
    }

    @Override
    public String toString() {
        return "NavMeshPathNode{" +
                "index=" + index +
                ", delaunayTriangle=" + delaunayTriangle +
                '}';
    }

    /**
     * Get the shared portal between two directly connected path finding nodes. The result is null if there
     * is not a direct connection.
     *
     * @param dest directly neighboring path node
     * @return the walkable portal between this node and dest node, or null if there is not a shared portal edge
     */
    public NavMeshPortal portalTo(NavMeshPathNode dest) {
        NavMeshPortal navMeshPortal = null;

        // Have to reconstruct the edges due to API limitations from gdx-ai extension
        for(Connection<NavMeshPathNode> nextConn : connections) {
            if(nextConn.getToNode() == dest) {
                navMeshPortal = ((NavMeshPathConn) nextConn).getPortal();
                break;
            }
        }

        return navMeshPortal;
    }
}
