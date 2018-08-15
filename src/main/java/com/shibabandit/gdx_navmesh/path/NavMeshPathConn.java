package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.DefaultConnection;

/**
 * A connection between two graph nodes, where the {@link #getPortal()} defines the edge between
 * two walkable polygons.
 */
public class NavMeshPathConn extends DefaultConnection<NavMeshPathNode> {
    private final NavMeshPortal portal;

    public NavMeshPathConn(NavMeshPathNode fromNode, NavMeshPathNode toNode, NavMeshPortal portal) {
        super(fromNode, toNode);
        this.portal = portal;
    }

    /**
     * @return get walkable edge between two navigation mesh nodes
     */
    public NavMeshPortal getPortal() {
        return portal;
    }
}