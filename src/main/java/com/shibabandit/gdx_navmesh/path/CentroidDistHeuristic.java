package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.Heuristic;
import com.badlogic.gdx.math.Vector2;

import static com.shibabandit.gdx_navmesh.util.DelaunayTriangleUtil.dtCentroid;

/**
 * A* heuristic using the distance between the delaunay triangle centroid of each path node.
 */
public final class CentroidDistHeuristic implements Heuristic<NavMeshPathNode> {
    private static final Vector2 NODE_CENTROID = new Vector2(),
            END_NODE_CENTROID = new Vector2();

    @Override
    public float estimate(NavMeshPathNode node, NavMeshPathNode endNode) {
        dtCentroid(node.getDelaunayTriangle(), NODE_CENTROID);
        dtCentroid(endNode.getDelaunayTriangle(), END_NODE_CENTROID);
        return NODE_CENTROID.dst(END_NODE_CENTROID);
    }
}
