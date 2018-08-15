package com.shibabandit.gdx_navmesh.coll;

import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.utils.Array;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.index.ItemVisitor;

/**
 * Support bounding box queries for {@link org.locationtech.jts.index.quadtree.Quadtree}.
 *
 * @param <T>
 */
public final class ArrayItemVisitor<T extends QtItem> implements ItemVisitor {

    public interface Filter<T> {
        boolean accept(T item);
    }

    private Array<T> items;
    private Rectangle queryBounds;
    private Rectangle nextBounds;
    private int itemsVisited;
    private Filter<T> filter;

    public ArrayItemVisitor() {
        this.items = null;
        this.queryBounds = new Rectangle();
        this.nextBounds = new Rectangle();
        this.itemsVisited = 0;
        //noinspection unchecked
        this.filter = ArrayItemVisitorFilters.AcceptAnyFilter.INSTANCE;
    }

    @Override
    public void visitItem(Object item) {
        ++itemsVisited;

        QtItem spObj = (QtItem) item;
        final Envelope spObjEnv = spObj.getEnvelope();

        // TODO: Why can't this use envelope?
        nextBounds.set((float) spObjEnv.getMinX(),
                (float) spObjEnv.getMinY(),
                (float) (spObjEnv.getMaxX() - spObjEnv.getMinX()),
                (float) (spObjEnv.getMaxY() - spObjEnv.getMinY()));

        if(nextBounds.overlaps(queryBounds) && filter.accept((T) item)) {
            items.add((T) item);
        }
    }

    public void clear() {
        if(items != null) {
            items.clear();
        }
        itemsVisited = 0;
    }

    public Array<T> getItems() {
        return items;
    }

    public void setItems(Array<T> items) {
        this.items = items;
    }

    public int getItemsVisited() {
        return itemsVisited;
    }

    public void resetStats() {
        itemsVisited = 0;
    }

    public Rectangle getQueryBounds() {
        return queryBounds;
    }

    public void setFilter(Filter<T> filter) {
        this.filter = filter;
    }
}
