#include <cstdint>

template<typename T, int N>
class RingArray {
public:
    RingArray() = default;
    T& operator[](uint64_t index) {
        return elements_[index % N];
    }
    const T& operator[](uint64_t index) const {
        return elements_[index % N];
    }

private:
    T elements_[N];
};
