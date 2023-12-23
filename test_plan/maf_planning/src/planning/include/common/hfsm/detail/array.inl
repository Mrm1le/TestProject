namespace hfsm {
namespace detail {

////////////////////////////////////////////////////////////////////////////////

template <typename T, unsigned TCapacity>
T&
StaticArray<T, TCapacity>::operator[] (const unsigned i) {
	mph_assert(0 <= i && i < CAPACITY);

	return _items[i];
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <typename T, unsigned TCapacity>
const T&
StaticArray<T, TCapacity>::operator[] (const unsigned i) const {
	mph_assert(0 <= i && i < CAPACITY);

	return _items[i];
}

////////////////////////////////////////////////////////////////////////////////

}
}
