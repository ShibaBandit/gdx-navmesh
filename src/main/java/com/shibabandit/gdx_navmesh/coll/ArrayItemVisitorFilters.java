package com.shibabandit.gdx_navmesh.coll;

/**
 * Common filter types for {@link ArrayItemVisitor}.
 */
public final class ArrayItemVisitorFilters {

    /**
     * Accept any input. Use {@link #INSTANCE} instead of constructor.
     *
     * @param <T>
     */
    public static final class AcceptAnyFilter<T> implements ArrayItemVisitor.Filter<T> {

        public static final AcceptAnyFilter INSTANCE = new AcceptAnyFilter();

        private AcceptAnyFilter() {}

        @Override
        public boolean accept(T item) {
            return true;
        }
    }
}
