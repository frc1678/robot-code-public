// vim: set filetype=cpp:

//TODO(Wesley) Don't do this every tick
//TODO(Wesley or Kyle) Clean up this code (remove structs)
namespace ConvertArgs {
// These operators are in structs because C++ doesn't support
// partial function overloading, which is needed to specialize
// on Units<U1, U2, U3, U4>
template<typename T>
struct Convert {
  T operator()(void* arg) {
    return *static_cast<T*>(arg);
  }
};

template<typename T>
struct Convert<T*>{
  T* operator()(void* arg) {
    return static_cast<T*>(arg);
  }
};
}
