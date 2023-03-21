//#include<bits/stdc++.h>
#include<stdio.h>
#include<set>

#define eps 1e-5
using namespace std;

struct Point
{
	double x, y, ag;
	Point(double x = 0, double y = 0) :x(x), y(y) {}
	Point operator +(const Point& B)const { return Point(x + B.x, y + B.y); }
	Point operator -(const Point& B)const { return Point(x - B.x, y - B.y); }
	double operator *(const Point& B)const { return x * B.y - y * B.x; }
}P[3], O;

struct node
{
	bool operator ()(const Point& A, const Point& B)
	{
		return A.ag < B.ag;
	}
};
set<Point, node>st;
set<Point, node>::iterator pre, nxt, ppre, nnxt;

int main()
{
	int q, op, x, y;
	scanf("%d", &q);
	q -= 3;
	scanf("%d%lf%lf", &op, &P[0].x, &P[0].y);
	scanf("%d%lf%lf", &op, &P[1].x, &P[1].y);
	scanf("%d%lf%lf", &op, &P[2].x, &P[2].y);
	O.x = (P[0].x + P[1].x + P[2].x) / 3;
	O.y = (P[0].y + P[1].y + P[2].y) / 3;
	for (int i = 0; i < 3; i++)
		P[i] = P[i] - O, P[i].ag = atan2(P[i].y, P[i].x);
	st.insert(P[0]), st.insert(P[1]), st.insert(P[2]);
	for (; q--;)
	{
		scanf("%d%lf%lf", &op, &P[0].x, &P[0].y);
		P[0] = P[0] - O;
		P[0].ag = atan2(P[0].y, P[0].x);
		if (op == 1)
		{
			nxt = st.lower_bound(P[0]), pre = nxt;
			if (pre == st.begin()) pre = st.end();
			pre--;
			if (nxt == st.end()) nxt = st.begin();
			if (((*nxt) - (*pre)) * ((*nxt) - P[0]) > eps)
			{
				for (; st.size() >= 2; st.erase(*nxt), nxt = nnxt)
				{
					nnxt = nxt, nnxt++;
					if (nnxt == st.end()) nnxt = st.begin();
					if (((*nnxt) - (*nxt)) * ((*nxt) - P[0]) < eps) break;
				}

				for (; st.size() >= 2; st.erase(*pre), pre = ppre)
				{
					ppre = pre;
					if (ppre == st.begin()) ppre = st.end();
					ppre--;
					if ((P[0] - (*pre)) * ((*pre) - (*ppre)) < eps) break;
				}
				st.insert(P[0]);
			}
		}
		else
		{
			nxt = st.lower_bound(P[0]), pre = nxt;
			if (pre == st.begin()) pre = st.end();
			pre--;
			if (nxt == st.end()) nxt = st.begin();
			if (((*nxt) - (*pre)) * ((*nxt) - P[0]) > eps)
				puts("NO");
			else
				puts("YES");
		}
	}
}


//一些 计算几何 经典算法

//https://www.cnblogs.com/YSFAC/p/13152847.html
/*
求凸包：Graham扫描法。
找到左下角的点，以该点为原点建系，将其他点极角排序。用队列维护构成当前凸包的点。
依次加入点，如果新加入的点与队列右端点构成的向量在上一条凸包边的顺时针方向，那么需要弹出上一个点。
最终将队列中所有点顺次连接构成凸包。
*/


//bool cmp1(node a, node b) { return a.x == b.x ? a.y < b.y : a.x < b.x; }
//bool cmp2(node a, node b) { return a * b > 0 || (a * b == 0 && dis(a) < dis(b)); }
//
//void Graham() {
//	sort(A + 1, A + 1 + n, cmp1);
//	for (int i = n; i >= 1; i--) A[i] = A[i] - A[1];
//	sort(A + 2, A + 1 + n, cmp2);
//	node sta[maxn]; int top = 0;
//	for (int i = 1; i <= n; sta[++top] = A[i++])
//		while (top >= 2 && (A[i] - sta[top]) * (sta[top] - sta[top - 1]) >= 0) top--;
//	n = top; for (int i = 1; i <= top; i++) A[i] = sta[i];
//}


