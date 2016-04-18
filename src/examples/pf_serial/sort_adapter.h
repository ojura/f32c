#ifndef SORT_ADAPTER_H
#define SORT_ADAPTER_H

#include <stdlib.h>

template <typename ConstPointer, typename Compare>
inline int qsort_adapter (const void* p1, const void* p2)
{
    ConstPointer i1 = (ConstPointer) p1;
    ConstPointer i2 = (ConstPointer) p2;
    Compare comp = Compare();
    return comp (*i1, *i2) ? -1 : (comp (*i2, *i1) ? 1 : 0);
}



template <typename RandomAccessIterator, typename Compare>
inline void sort (RandomAccessIterator first, RandomAccessIterator last, Compare)
{    qsort (first, (char) (last - first), sizeof(*first),
        qsort_adapter<RandomAccessIterator, Compare>);
}

#endif
