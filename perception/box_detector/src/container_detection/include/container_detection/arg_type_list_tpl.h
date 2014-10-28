#pragma once
//~ From: http://stackoverflow.com/a/21607170
template<typename T, int N> class mytype;
template<typename T, int N> std::istream& operator>> (std::istream& is, mytype<T,N>& rhs);
template<typename T, int N> std::ostream& operator<< (std::ostream& os, const mytype<T,N>& rhs);
template < typename T, int N >
struct mytype
{
  T values[N];
  friend std::istream& operator>> <>(std::istream &is, mytype<T,N> &val);
  friend std::ostream& operator<< <>(std::ostream &os, const mytype<T,N> &val);
};
template<typename T, int N>
inline std::istream& operator>>(std::istream &is, mytype<T,N> &val)
{
  for( int i = 0; i < N; ++i )
    {
    if( i )
      if (is.peek() == ',')
        is.ignore();
    is >> val.values[i];
    }
  return is;
}
template<typename T, int N>
inline std::ostream& operator<<(std::ostream &os, const mytype<T,N> &val)
{
  for( int i = 0; i < N; ++i )
    {
    if( i ) os << ',';
    os << val.values[i];
    }
  return os;
}
