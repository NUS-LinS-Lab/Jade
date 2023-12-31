/*
 * Basic vector. Maybe happy templated crazyness later.
 *
 */

#ifndef AMC_VECTOR_HPP
#define AMC_VECTOR_HPP

#include <cmath>

template <typename NUM, int size>
class Vector
{
public:
  NUM c[size];
  NUM& operator[](int const& i)
  {
    return c[i];
  }
  template <typename NUM2>
  inline Vector<NUM, size>& operator=(Vector<NUM2, size> const& b)
  {
    for (unsigned int i = 0; i < size; ++i)
    {
      c[i] = (NUM)b.c[i];
    }
    return *this;
  }
};

template <typename NUM>
class Vector<NUM, 0>
{
public:
  NUM* c; // sure, why not.
  // empty.
};

template <typename NUM>
class Vector<NUM, 2>
{
public:
#ifndef NOUNION
  union
  {
#endif
    NUM c[2];
#ifndef NOUNION
    struct
    {
      NUM x;
      NUM y;
    };
    struct
    {
      NUM u;
      NUM v;
    };
  };
#endif
  NUM& operator[](int const& i)
  {
    return c[i];
  }
  const NUM& operator[](int const& i) const
  {
    return c[i];
  }
  template <typename NUM2>
  inline Vector<NUM, 2>& operator=(Vector<NUM2, 2> const& b)
  {
    for (unsigned int i = 0; i < 2; ++i)
    {
      c[i] = (NUM)b.c[i];
    }
    return *this;
  }
};

template <typename NUM>
class Vector<NUM, 3>
{
public:
#ifndef NOUNION
  union
  {
#endif
    NUM c[3];
#ifndef NOUNION
    struct
    {
      NUM x;
      NUM y;
      NUM z;
    };
    struct
    {
      Vector<NUM, 2> xy;
      NUM pad1;
    };
    struct
    {
      NUM pad2;
      Vector<NUM, 2> yz;
    };
    struct
    {
      NUM r;
      NUM g;
      NUM b;
    };
  };
#endif
  NUM& operator[](int const& i)
  {
    return c[i];
  }
  const NUM& operator[](int const& i) const
  {
    return c[i];
  }
  template <typename NUM2>
  inline Vector<NUM, 3>& operator=(Vector<NUM2, 3> const& b)
  {
    for (unsigned int i = 0; i < 3; ++i)
    {
      c[i] = (NUM)b.c[i];
    }
    return *this;
  }

  void set(NUM const& v1, NUM const& v2, NUM const& v3)
  {
    c[0] = v1;
    c[1] = v2;
    c[2] = v3;
  }
};

template <typename NUM>
class Vector<NUM, 4>
{
public:
#ifndef NOUNION
  union
  {
#endif
    NUM c[4];
#ifndef NOUNION
    struct
    {
      NUM x;
      NUM y;
      NUM z;
      NUM w;
    };
    struct
    {
      NUM r;
      NUM g;
      NUM b;
      NUM a;
    };
    struct
    {
      NUM pad1;
      Vector<NUM, 2> yz;
      NUM pad2;
    };
    struct
    {
      Vector<NUM, 2> xy;
      Vector<NUM, 2> zw;
    };
    struct
    {
      Vector<NUM, 3> xyz;
      NUM pad3;
    };
    struct
    {
      NUM pad4;
      Vector<NUM, 3> yzw;
    };
  };
#endif
  NUM& operator[](int const& i)
  {
    return c[i];
  }
  const NUM& operator[](int const& i) const
  {
    return c[i];
  }

  template <typename NUM2>
  inline Vector<NUM, 4>& operator=(Vector<NUM2, 4> const& b)
  {
    for (unsigned int i = 0; i < 4; ++i)
    {
      c[i] = (NUM)b.c[i];
    }
    return *this;
  }
};

typedef Vector<double, 2> Vector2s;
typedef Vector<float, 2> Vector2f;
typedef Vector<int, 2> Vector2i;

typedef Vector<double, 3> Vector3s;
typedef Vector<float, 3> Vector3f;
typedef Vector<int, 3> Vector3i;

typedef Vector<double, 4> Vector4s;
typedef Vector<float, 4> Vector4f;
typedef Vector<int, 4> Vector4i;

// these should be cunningly optimized.
template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> operator+(
    Vector<NUM, SIZE> const& a, Vector<NUM, SIZE> const& b)
{
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = a.c[i] + b.c[i];
  }
  return ret;
}

template <typename NUM, int SIZE>
inline void operator+=(Vector<NUM, SIZE>& a, Vector<NUM, SIZE> const& b)
{
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    a.c[i] += b.c[i];
  }
}

template <typename NUM, int SIZE>
inline void operator-=(Vector<NUM, SIZE>& a, Vector<NUM, SIZE> const& b)
{
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    a.c[i] -= b.c[i];
  }
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> operator-(Vector<NUM, SIZE> const& a)
{
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = -a.c[i];
  }
  return ret;
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> operator-(
    Vector<NUM, SIZE> const& a, Vector<NUM, SIZE> const& b)
{
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = a.c[i] - b.c[i];
  }
  return ret;
}

template <typename NUM, int SIZE>
inline NUM operator*(Vector<NUM, SIZE> const& a, Vector<NUM, SIZE> const& b)
{
  NUM ret;
  ret = 0;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret += a.c[i] * b.c[i];
  }
  return ret;
}

template <typename NUM, int SIZE, typename NUM2>
inline Vector<NUM, SIZE> operator*(Vector<NUM, SIZE> const& a, NUM2 const& b)
{
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = a.c[i] * (NUM)b;
  }
  return ret;
}

template <typename NUM, int SIZE, typename NUM2>
inline Vector<NUM, SIZE> operator*(NUM2 const& b, Vector<NUM, SIZE> const& a)
{
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = a.c[i] * b;
  }
  return ret;
}

template <typename NUM, int SIZE, typename NUM2>
inline void operator*=(Vector<NUM, SIZE>& a, NUM2 const& b)
{
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    a.c[i] *= (NUM)b;
  }
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> product(
    Vector<NUM, SIZE> const& a, Vector<NUM, SIZE> const& b)
{
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = a.c[i] * b.c[i];
  }
  return ret;
}

template <typename NUM, int SIZE, typename NUM2>
inline Vector<NUM, SIZE> operator/(Vector<NUM, SIZE> const& a, NUM2 const& b)
{
  Vector<NUM, SIZE> ret;
  NUM one;
  one = 1.0;
  NUM temp2 = one / b;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = a.c[i] * temp2;
  }
  return ret;
}

template <typename NUM, int SIZE, typename NUM2>
inline bool operator==(Vector<NUM, SIZE> const& a, Vector<NUM2, SIZE> const& b)
{
  bool equal = true;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    equal = equal && (a.c[i] == b.c[i]);
  }
  return equal;
}

template <typename NUM, int SIZE, typename NUM2>
inline bool operator!=(Vector<NUM, SIZE> const& a, Vector<NUM2, SIZE> const& b)
{
  bool nequal = false;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    nequal = nequal || (a.c[i] != b.c[i]);
  }
  return nequal;
}

template <typename NUM, int SIZE, typename NUM2>
inline void operator/=(Vector<NUM, SIZE>& a, NUM2 const& b)
{
  NUM one;
  one = 1.0;
  NUM temp2 = one / b;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    a.c[i] *= temp2;
  }
}

template <typename NUM, int SIZE>
inline NUM length_squared(Vector<NUM, SIZE> const& a)
{
  return a * a;
}

template <typename NUM, int SIZE>
inline NUM length(Vector<NUM, SIZE> const& a)
{
  NUM ret;
  ret = sqrt(a * a);
  return ret;
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> normalize(Vector<NUM, SIZE> a)
{
  NUM len;
  len = length(a);
  if (len == 0)
  {
    a.c[0] = 1;
  }
  else
  {
    NUM one, tmp;
    one = 1.0;
    tmp = one / len;
    a *= tmp;
  }
  return a;
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> perpendicular(Vector<NUM, SIZE> const& a)
{
  return make_vector(-a.c[1], a.c[0]);
}

// this does a 3d cross product -- may not perform well on other types.
// will do very bad things on size==2
template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> cross_product(
    Vector<NUM, SIZE> const& a, Vector<NUM, SIZE> const& b)
{
  Vector<NUM, SIZE> ret;
  ret.c[0] = a.c[1] * b.c[2] - a.c[2] * b.c[1];
  ret.c[1] = a.c[2] * b.c[0] - a.c[0] * b.c[2];
  ret.c[2] = a.c[0] * b.c[1] - a.c[1] * b.c[0];
  return ret;
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> lerp(
    Vector<NUM, SIZE> a, Vector<NUM, SIZE> const& b, NUM const& amt)
{
  for (int i = 0; i < SIZE; ++i)
  {
    a.c[i] = (a.c[i] * (NUM(1) - amt)) + (b.c[i] * amt);
  }
  return a;
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> min(
    Vector<NUM, SIZE> const& a, Vector<NUM, SIZE> const& b)
{
  Vector<NUM, SIZE> ret = a;
  for (int i = 0; i < SIZE; ++i)
  {
    if (b.c[i] < ret.c[i])
      ret.c[i] = b.c[i];
  }
  return ret;
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> max(
    Vector<NUM, SIZE> const& a, Vector<NUM, SIZE> const& b)
{
  Vector<NUM, SIZE> ret = a;
  for (int i = 0; i < SIZE; ++i)
  {
    if (b.c[i] > ret.c[i])
      ret.c[i] = b.c[i];
  }
  return ret;
}

template <int SIZE>
inline Vector<float, SIZE> abs(Vector<float, SIZE> const& a)
{
  Vector<float, SIZE> ret;
  for (int i = 0; i < SIZE; ++i)
  {
    ret[i] = fabsf(a[i]);
  }
  return ret;
}

template <int SIZE>
inline Vector<double, SIZE> abs(Vector<double, SIZE> const& a)
{
  Vector<double, SIZE> ret;
  for (int i = 0; i < SIZE; ++i)
  {
    ret[i] = fabs(a[i]);
  }
  return ret;
}

#include <iostream>

using std::istream;
using std::ostream;

template <typename NUM, int SIZE>
ostream& operator<<(ostream& o, Vector<NUM, SIZE> const& vec)
{
  o << "( ";
  for (int i = 0; i < SIZE; ++i)
  {
    if (i != 0)
      o << ", ";
    o << vec.c[i];
  }
  o << " )";
  return o;
}

template <typename NUM, int SIZE>
istream& operator>>(istream& in, Vector<NUM, SIZE>& vec)
{
  char c;
  if (!(in >> c) || c != '(')
  {
    in.setstate(std::ios::failbit);
    return in;
  }
  for (int i = 0; i < SIZE; ++i)
  {
    if (i != 0)
    {
      if (!(in >> c) || c != ',')
      {
        in.setstate(std::ios::failbit);
        return in;
      }
    }
    if (!(in >> vec.c[i]))
    {
      in.setstate(std::ios::failbit);
      return in;
    }
  }
  if (!(in >> c) || c != ')')
  {
    in.setstate(std::ios::failbit);
    return in;
  }
  return in;
}

template <typename NUM, typename NUM2, int SIZE>
inline Vector<NUM, SIZE> make_vector(Vector<NUM2, SIZE> const& in)
{
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = (NUM)in.c[i];
  }
  return ret;
}

template <typename NUM>
inline Vector<NUM, 2> make_vector(NUM x, NUM y)
{
  Vector<NUM, 2> ret;
  ret.c[0] = x;
  ret.c[1] = y;
  return ret;
}

template <typename NUM>
inline Vector<NUM, 3> make_vector(NUM x, NUM y, NUM z)
{
  Vector<NUM, 3> ret;
  ret.c[0] = x;
  ret.c[1] = y;
  ret.c[2] = z;
  return ret;
}

template <typename NUM>
inline Vector<NUM, 4> make_vector(NUM x, NUM y, NUM z, NUM w)
{
  Vector<NUM, 4> ret;
  ret.c[0] = x;
  ret.c[1] = y;
  ret.c[2] = z;
  ret.c[3] = w;
  return ret;
}

template <typename NUM>
inline Vector<NUM, 3> make_vector(Vector<NUM, 2> v, NUM z)
{
  Vector<NUM, 3> ret;
  ret.c[0] = v.c[0];
  ret.c[1] = v.c[1];
  ret.c[2] = z;
  return ret;
}

template <typename NUM>
inline Vector<NUM, 3> make_vector(NUM x, Vector<NUM, 2> v)
{
  Vector<NUM, 3> ret;
  ret.c[0] = x;
  ret.c[1] = v.c[0];
  ret.c[2] = v.c[1];
  return ret;
}

template <typename NUM>
inline Vector<NUM, 4> make_vector(Vector<NUM, 3> v, NUM w)
{
  Vector<NUM, 4> ret;
  ret.c[0] = v.c[0];
  ret.c[1] = v.c[1];
  ret.c[2] = v.c[2];
  ret.c[3] = w;
  return ret;
}

template <typename NUM>
inline Vector<NUM, 4> make_vector(NUM x, Vector<NUM, 3> v)
{
  Vector<NUM, 4> ret;
  ret.c[0] = x;
  ret.c[1] = v.c[0];
  ret.c[2] = v.c[1];
  ret.c[3] = v.c[2];
  return ret;
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> make_vector(NUM x)
{
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = x;
  }
  return ret;
}

template <typename NUM, int SIZE1, int SIZE2>
inline Vector<NUM, SIZE1 + SIZE2> make_vector(
    Vector<NUM, SIZE1> const& a, Vector<NUM, SIZE2> const& b)
{
  Vector<NUM, SIZE1 + SIZE2> ret;
  for (unsigned int i = 0; i < SIZE1; ++i)
  {
    ret.c[i] = a.c[i];
  }
  for (unsigned int i = 0; i < SIZE2; ++i)
  {
    ret.c[SIZE1 + i] = b.c[i];
  }
  return ret;
}

template <typename NUM, int SIZE1>
inline Vector<NUM, SIZE1> make_vector(
    Vector<NUM, SIZE1> const& a, Vector<NUM, 0> const& /* b */)
{
  return a;
}

template <typename NUM, int SIZE>
inline Vector<NUM, SIZE> make_vector_p(NUM const* in)
{ // hack!
  Vector<NUM, SIZE> ret;
  for (unsigned int i = 0; i < SIZE; ++i)
  {
    ret.c[i] = in[i];
  }
  return ret;
}

#endif
