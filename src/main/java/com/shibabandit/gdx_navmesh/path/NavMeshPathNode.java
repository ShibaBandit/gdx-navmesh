package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.utils.Array;
import com.shibabandit.gdx_navmesh.coll.QtItem;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

/**
 * Navmesh path node that supports {@link org.locationtech.jts.index.quadtree.Quadtree} search and lies on a
 * Delaunay Triangle walkable portal. Fields left protected to support inheritance.
 *
 * @see IndexedNavMeshAStarPathFinder
 * @see org.locationtech.jts.index.quadtree.Quadtree
 * @see QtItem
 * @see DelaunayTriangle
 */
public class NavMeshPathNode {

    /** Connections to neighboring navmesh nodes: any portals open to the two triangles that this node lies on. */
    protected final Array<Connection<NavMeshPathNode>> connections;

    /** Global graph index for fast lookup */
    protected final int index;

    /** The first triangle geometry shared by this portal */
    protected DelaunayTriangle dtA;

    /** The second triangle geometry shared by this portal */
    protected DelaunayTriangle dtB;

    /** The actual spatial data for the path node. The midpoint should be used as the node location. */
    protected final NavMeshPortal portal;

    /**
     * @param index global graph index for fast lookup
     * @param portal the actual spatial data for the path node
     */
    public NavMeshPathNode(int index,
                           NavMeshPortal portal) {

        // Portal nodes will have a maximum of 4 shared edges
        this(new Array<>(4), index, portal);
    }

    /**
     * @param connections connections to neighboring navmesh nodes
     * @param index global graph index for fast lookup
     * @param portal the actual spatial data for the path node
     */
    public NavMeshPathNode(Array<Connection<NavMeshPathNode>> connections,
                           int index,
                           NavMeshPortal portal) {
        this.connections = connections;
        this.index = index;
        this.portal = portal;
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
     * @return the first triangle connected to this portal node
     */
    public DelaunayTriangle getDtA() {
        return dtA;
    }

    /**
     * @param dtA the first triangle connected to this portal node
     */
    public void setDtA(DelaunayTriangle dtA) {
        this.dtA = dtA;
    }

    /**
     * @return the second triangle connected to this portal node
     */
    public DelaunayTriangle getDtB() {
        return dtB;
    }

    /**
     * @param dtB the second triangle connected to this portal node
     */
    public void setDtB(DelaunayTriangle dtB) {
        this.dtB = dtB;
    }

    /**
     * @return spatial data for the path node
     */
    public NavMeshPortal getPortal() {
        return portal;
    }

    @Override
    public String toString() {
        return "NavMeshPathNode{" +
                "connections=" + connections +
                ", index=" + index +
                ", dtA=" + dtA +
                ", dtB=" + dtB +
                ", portal=" + portal +
                '}';
    }
}
