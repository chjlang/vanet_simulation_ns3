#ifndef _PUBLIC_H_
#define _PUBLIC_H_

struct Position
{
	int m_x, m_y;	//coordinate of a point 
	Position(int x, int y) : m_x(x), m_y(y) {}

	//operator overload
	Position& operator = (const Position& p)
	{
		m_x = p.m_x;
		m_y = p.m_y;
		return *this;
	}
};

#endif