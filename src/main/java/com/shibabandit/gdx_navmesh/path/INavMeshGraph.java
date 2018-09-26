package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.indexed.IndexedGraph;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

/**
 * @param <N> 
 */
public interface INavMeshGraph<N extends NavMeshPathNode> extends IndexedGraph<NavMeshPathNode> {

    /**
     * @param triangle
     * @return the navigation mesh node associated with the provided triangle
     */
    N getNode(DelaunayTriangle triangle);
}
