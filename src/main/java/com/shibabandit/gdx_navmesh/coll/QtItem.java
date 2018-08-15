package com.shibabandit.gdx_navmesh.coll;

import org.locationtech.jts.geom.Envelope;

/**
 * Intended for use with {@link org.locationtech.jts.index.quadtree.Quadtree} and provides its bounding
 * {@link Envelope}.
 */
public interface QtItem {
    Envelope getEnvelope();
}
