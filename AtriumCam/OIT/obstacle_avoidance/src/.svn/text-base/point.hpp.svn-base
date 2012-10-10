#ifndef POINT_HPP_
#define POINT_HPP_

#include <iosfwd>
#include <cmath>

class Point;

float dot( const Point &a, const Point &b );
Point cross( const Point &a, const Point &b );

class Point 
{
public:
	float m_P[3];

	Point(){}
	virtual ~Point(){}
	Point(const Point &p)
	{
		x()= p.x();
		y()= p.y();
		z()= p.z();
	}
	Point & operator=( const Point &p )
	{
		x()=p.x();
		y()=p.y();
		z()=p.z();
		return *this;
	}
	bool operator==( const Point &p ) const
	{
		return p.x()==x() && p.y()==y() && p.z() == z();
	}
	Point operator+=( const Point &p )
	{
		m_P[0]+=p.m_P[0];
		m_P[1]+=p.m_P[1];
		m_P[2]+=p.m_P[2];
		return *this;
	}
	Point operator-=( const Point &p )
	{
		m_P[0]-=p.m_P[0];
		m_P[1]-=p.m_P[1];
		m_P[2]-=p.m_P[2];
		return *this;
	}
	Point operator*=( const float p )
	{
		m_P[0]*=p;
		m_P[1]*=p;
		m_P[2]*=p;
		return *this;
	}
	Point operator-()
	{
		Point p;
		p.m_P[0]= -m_P[0];
		p.m_P[1]= -m_P[1];
		p.m_P[2]= -m_P[2];
		return p;
	}
    
	Point(const float x, const float y, const float z=0.0)
		{m_P[0]=x; m_P[1]=y; m_P[2]=z;}

	float x() const { return m_P[0];}
	float y() const { return m_P[1];}
	float z() const { return m_P[2];}

	float& x() { return m_P[0];}
	float& y() { return m_P[1];}
	float& z() { return m_P[2];}
    std::ostream& put(std::ostream& s) const;
};

std::ostream& operator<<(std::ostream &s, const Point& a);

inline float magnitude(const Point &p) 
{
	return std::sqrt( p.m_P[0]*p.m_P[0] + p.m_P[1]*p.m_P[1] + p.m_P[2]*p.m_P[2] );
}

inline Point unit(const Point &p) 
{
	return Point( p.x()/magnitude(p), p.y()/magnitude(p), p.z()/magnitude(p) );
}

#endif

