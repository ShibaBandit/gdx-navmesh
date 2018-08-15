package com.shibabandit.gdx_navmesh.coll;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.index.quadtree.Quadtree;

/**
 * Wrapper around {@link Quadtree} common boilerplate.
 *
 * @param <T> item type stored in quad tree
 */
public final class QtSearchIndex<T extends QtItem> {

    /** Track item envelopes */
    private final Quadtree qt;

    /** Query a spatial index */
    private final Envelope qtEnv;

    /** Query result container */
    private final ArrayItemVisitor<T> visitor;

    public QtSearchIndex() {
        qt = new Quadtree();
        qtEnv = new Envelope();
        visitor = new ArrayItemVisitor<>();
    }

    public void itemsInRange(Vector2 targetPt, float range, Array<T> resultsInRange) {
        //noinspection unchecked
        itemsInRange(targetPt, range, resultsInRange, ArrayItemVisitorFilters.AcceptAnyFilter.INSTANCE);
    }

    public void itemsInRange(Vector2 targetPt, float range, Array<T> resultsInRange, ArrayItemVisitor.Filter<T> filter) {
//        final long t0 = System.nanoTime();
        final float range2 = range * 2f;
        visitor.getQueryBounds().set(targetPt.x - range, targetPt.y - range, range2, range2);
        visitor.setItems(resultsInRange);
        visitor.setFilter(filter);
        visitor.resetStats();
        qtEnv.init(targetPt.x - range, targetPt.x + range, targetPt.y - range, targetPt.y + range);
        qt.query(qtEnv, visitor);
//        final long t1 = System.nanoTime();
//        Gdx.app.log("QtSearchIndex", "Elapsed nanos for search: " + (t0 - t1));
    }

    public void itemsInRange(Vector2 lowerLeft,
                             float width,
                             float height,
                             Array<T> resultsInRange,
                             ArrayItemVisitor.Filter<T> filter) {

        visitor.getQueryBounds().set(lowerLeft.x, lowerLeft.y, width, height);
        visitor.setItems(resultsInRange);
        visitor.setFilter(filter);
        visitor.resetStats();
        qtEnv.init(lowerLeft.x, lowerLeft.x + width, lowerLeft.y, lowerLeft.y + height);
        qt.query(qtEnv, visitor);
    }

    public Quadtree getQt() {
        return qt;
    }
}
