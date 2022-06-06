
#ifndef __STATIC_DATA_H__
#define __STATIC_DATA_H__

#ifdef WITH_USD
#include <pxr/base/tf/staticData.h>
#endif

CCL_NAMESPACE_BEGIN

#ifdef WITH_USD
template<class T, class Factory= pxr::Tf_StaticDataDefaultFactory<T>>
using Static_Data = pxr::TfStaticData<T, Factory>;
#else
template <class T>
struct StaticDataDefaultFactory {
    static T *New() { return new T; }
};

template <class T, class Factory = StaticDataDefaultFactory<T> >
class Static_Data {
public:
    /// Return a pointer to the underlying data object. It is created and
    /// initialized if necessary.
    inline T* operator-> () const { return Get(); }

    /// Member lookup. The underlying data object is created and initialized
    /// if necessary.
    inline T& operator* () const { return *Get(); }

    /// Return a pointer to the underlying object, creating and initializing
    /// it if necessary.
    inline T* Get() const {
        T *p = _data;
        return ARCH_LIKELY(p) ? p : _TryToCreateData();
    }
    
    /// Return true if the underlying data object is created and initialized.
    /// Return false otherwise.
    inline bool IsInitialized() const { return _data.load() != nullptr; }

private:
    T *_TryToCreateData() const {
        // Allocate an instance.
        T *tmp = Factory::New();

        // Try to atomically set the pointer from null to tmp.
        T *n = nullptr;
        if (ARCH_LIKELY(_data.compare_exchange_strong(n, tmp)))
            return tmp;

        // Another thread won the initialization race.
        delete tmp;
        return _data;
    }

    mutable std::atomic<T *> _data;
};
#endif

CCL_NAMESPACE_END
#endif  //__STATIC_DATA_H__
