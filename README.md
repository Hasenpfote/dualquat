[![Build Status](https://travis-ci.com/Hasenpfote/dualquat.svg?branch=master)](https://travis-ci.com/Hasenpfote/dualquat)

# dualquat

---------------------

## About

- Class template for dual quaternions using Eigen.

- Header-only.



## Compatibility

Supports C++ 11 or higher.

| Compiler | Version           | Remarks |
| -------- | ----------------- | ------- |
| gcc      | 5.5.0 or higher.  |         |
| clang    | 7.0.0 or higher.  |         |
| msvc     | 16.5.4 or higher. |         |

**Older versions of compilers might work as well but they are not tested.**



## usage

```c++
#include <dualquat/dualquat.h>

int main()
{
    using value_type = double;
    using Quat = Eigen::Quaternion<value_type>;
    using Vec3 = Quat::Vector3;
    using AngleAxis = Quat::AngleAxisType;
    using DualQuat = dualquat::DualQuaternion<value_type>;

    Quat r = AngleAxis(
        std::acos(-1.0),
        Vec3(1.0, 0.0, 0.0)
    );
    auto t = Vec3(1.0, 2.0, 3.0);

    auto src = Vec3(0.0, 0.0, 0.0);
    auto dst = transform(dualquat::transformation(r, t), src);

    return 0;
}
```



## References

- [Eigen](http://eigen.tuxfamily.org)
- [Dual quaternion](https://en.wikipedia.org/wiki/Dual_quaternion)



## License

This software is released under the CC0 License, see LICENSE.
