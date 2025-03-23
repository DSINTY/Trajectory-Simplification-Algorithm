#ifndef SIM_HPP
#define SIM_HPP
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/Circular_kernel_intersections.h>
#include "line.hpp"
#include "point.hpp"
#include "algorithm.hpp"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;


typedef CGAL::Exact_circular_kernel_2 Circular_kernel;
typedef Circular_kernel::Line_2 Line_2;
typedef Circular_kernel::Circle_2 Circle_2;

using namespace std;

class SIM :public Algorithm {
public:
    SIM(double bound_) :Algorithm{ bound_ } {

    }

	Point_2 point_cast(Point pt) {
		return Point_2(pt.x, pt.y);
	}

	Line line_from_pt_2(Point_2 p, Point_2 q) {
		return Line(Point(p.x(), p.y()), Point(q.x(), q.y()));

	}

	Line line_from_seg(Segment_2 seg) {
		return line_from_pt_2(seg.source(), seg.target());
	}

	Point_2 getIntersect(Line_2 line, Circle_2 c, Point_2 p) {
		std::vector<pair<CGAL::Circular_arc_point_2<Circular_kernel>, unsigned>> res;
		CGAL::intersection(line, c, std::back_inserter(res));
		cout << "res size: " << res.size() << endl;
		if (res.size() == 0) {
			throw "ERROR: No intersection found";
		}
		CGAL::Circular_arc_point_2<Circular_kernel> resPt = res[0].first;
		Point_2 interPt(CGAL::to_double(resPt.x()), CGAL::to_double(resPt.y()));
		for (vector<pair<CGAL::Circular_arc_point_2<Circular_kernel>, unsigned>>::iterator it = res.begin() + 1; it != res.end(); it++) {
			CGAL::Circular_arc_point_2<Circular_kernel> resTmpPt = (*it).first;
			Point_2 tmpPt = Point_2(CGAL::to_double(resTmpPt.x()), CGAL::to_double(resTmpPt.y()));
			if (CGAL::has_larger_distance_to_point(p, interPt, tmpPt)) {
				interPt = tmpPt;
			}
		}
		return interPt;
	}

    Trajectory<Line>* compress(const Trajectory<Point>* traj) {
		Point_2 v = point_cast((*traj)[0]);
		bool init = true;
		bool first = true;
		Trajectory<Line>* res = new Trajectory<Line>();
		Segment_2 seg(v, v);
		unordered_set<Point_2> P;
		unordered_map<Point_2, unordered_set<Point_2>> S_i;
		unordered_map<Point_2, pair<Point_2, Point_2>> tangents;
		unordered_map<Point_2, unordered_set<Point_2>> arc_centers;
		//Circle_2 B_v;
		for (int i = 0; i < traj->size();i++)
		{

			if (init) {
				init = false;
				if (first) first = false;
				else res->push(line_from_seg(seg));
				seg = { v,v };
				P = { v };
				for (Point_2 p : P) {
					S_i[p] = { p };
					tangents[p] = { p, p };
				}
			}
			else {
				for (Point_2 p : P) {
					if (!S_i[p].empty()) {
						seg = { p, *(S_i[p].begin()) };
						break;
					}

				}
			}
			if (i == traj->size() - 1) {
				res->push(line_from_seg(seg));
				break;
			}
			v = point_cast((*traj)[i]);
			//B_v = Circle_2(v, delta);
			//cout << "v: " << v << endl;
			//cout << "v:" << v << endl;
			for (Point_2 p : P) {
				if (p == v) continue;
				cout << "v-p: " << v-p << endl;

				double dx = v.x() - p.x();
				double dy = v.y() - p.y();
				double distance = std::sqrt(dx * dx + dy * dy);

				// Calculate the angle to the center of the circle
				double angle_to_center = std::atan2(dy, dx);

				//cout << "p: " << p << endl;
				//cout << "angle_to_center: " << angle_to_center << endl;

				// Calculate the angle for the tangent lines using the radius
				double angle_offset = std::atan(bound / distance);
				//cout << "delta: " << delta << endl;
				//cout << "distance: " << distance << endl;
				//cout << "angle_offset: " << angle_offset << endl;

				// Calculate the angles of the tangent lines
				double angle1 = angle_to_center - angle_offset;
				double angle2 = angle_to_center + angle_offset;
				double tangent_length = std::sqrt(distance * distance + bound * bound);
				// Calculate the tangent points
				Point_2 tangent1(p.x() + tangent_length * std::cos(angle1), p.y() + tangent_length * std::sin(angle1));
				Point_2 tangent2(p.x() + tangent_length * std::cos(angle2), p.y() + tangent_length * std::sin(angle2));
				//Point_2 tangent1(p.x() + delta * std::cos(angle1), p.y() + delta * std::sin(angle1));
				//Point_2 tangent2(p.x() + delta * std::cos(angle2), p.y() + delta * std::sin(angle2));

				// swap tangent1 and tangent2 if tangent1 is on the right of tangent2
				if (CGAL::orientation(p, tangent1, tangent2) == CGAL::RIGHT_TURN) {
					Point_2 tmp = tangent1;
					tangent1 = tangent2;
					tangent2 = tmp;
				}

				//cout << CGAL::orientation(v, p, tangents[p].first) << endl;
				//cout << CGAL::orientation(v, p, tangents[p].second) << endl;
				bool in_arc = true;
				if (tangents[p].first == p && tangents[p].second == p) in_arc = true;
				else {
					for (Point_2 q : arc_centers[p]) {
						if (CGAL::angle(p, q, v) == CGAL::ACUTE && CGAL::squared_distance(q,v)>(bound*bound)) {
							in_arc = false;
							break;
						}
					}
				}



				if ((tangents[p].first == p && tangents[p].second == p) || (CGAL::angle(v, p, tangents[p].first) == CGAL::ACUTE && CGAL::angle(v, p, tangents[p].second) == CGAL::ACUTE && (CGAL::orientation(v, p, tangents[p].first) != CGAL::orientation(v, p, tangents[p].second)))) {
					bool first_pt = false;
					if (tangents[p].first == p && tangents[p].second == p) {
						first_pt = true;
					}
					S_i[p] = { v };
					

					// update the tangent points to be intersection of two tangent cones
					//cout << tangents[p].first << endl;
					//cout << tangents[p].second << endl;
					bool new_tangent_1 = false;
					bool new_tangent_2 = false;
					if (CGAL::orientation(tangents[p].first, p, tangent1) == CGAL::LEFT_TURN || tangents[p].first == p) {
						
						cout << "updating t1" << endl;
						cout << "original t1-p " << tangents[p].first - p << endl;
						cout << "new t1-p" << tangent1 - p << endl;
						new_tangent_1 = true;
						tangents[p].first = tangent1;
					}

					if (CGAL::orientation(tangents[p].second, p, tangent2) == CGAL::RIGHT_TURN || tangents[p].second == p) {
						new_tangent_2 = true;
						tangents[p].second = tangent2;
					}

					if (CGAL::orientation(tangents[p].first, p, tangents[p].second) == CGAL::RIGHT_TURN) {
						Point_2 tmp = tangents[p].first;
						tangents[p].first = tangents[p].second;
						tangents[p].second = tmp;
					}

					//cout << tangents[p].first << endl;
					//cout << tangents[p].second << endl;

					// update the arc centers
					if (first_pt)
						arc_centers[p] = { v };
					else {
						Circular_kernel::Point_2 p1 = { p.x(), p.y() };
						Circular_kernel::Point_2 p2 = { tangents[p].first.x(), tangents[p].first.y() };
						Circular_kernel::Point_2 p3 = { tangents[p].second.x(), tangents[p].second.y() };

						//construct the tangent line from p to tangents[p].first
						Line_2 ptangent1 = { p1, p2 };
						//construct the tangent line from p to tangents[p].second
						Line_2 ptangent2 = { p1, p3 };



						// construct the circle centered at v with radius bound
						Circle_2 B_v = { {v.x(), v.y()}, bound * bound };

						//typedef typename CGAL::CK2_Intersection_traits<Circular_kernel, Line_2, Circle_2>::type Intersection_result;

						//cout << "v-p: " << (v-p) << endl;
						cout << "tangent 1-p: " << tangents[p].first -p<< endl;
						cout << "tangent 2-p: " << tangents[p].second -p<< endl;
						cout << "v intersection:" << endl;
		
						Point_2 interPt1;
						if (new_tangent_1) {
							interPt1 = tangents[p].first;
						}
						else {
							interPt1 = getIntersect(ptangent1, B_v, p);
						}
						Point_2 interPt2;
						if (new_tangent_2) {
							interPt2 = tangents[p].second;
						}
						else {
							interPt2 = getIntersect(ptangent2, B_v, p);
						}
						




						// find the intersection of the circle centered at v with radius bound with the two tangent lines


						bool add_v = true;

						
						unordered_set<Point_2> to_remove;
						for (Point_2 q : arc_centers[p]) {
							cout << "q-p: " << q - p << endl;
							Circle_2 B_q = { {q.x(), q.y()}, bound * bound };
							cout << "q intersection:" << endl;
							Point_2 qinterPt1 = getIntersect(ptangent1, B_q, p);
							Point_2 qinterPt2 = getIntersect(ptangent2, B_q, p);
							cout << "q intersection 1-p: " << qinterPt1 - p << endl;
							cout << "q intersection 2-p: " << qinterPt2 - p << endl;

							if (CGAL::has_larger_distance_to_point(p, qinterPt1, interPt1) && CGAL::has_larger_distance_to_point(p, qinterPt2, interPt2)) {
								cout << "don't add v" << endl;
								add_v = false;

							}
							else if (CGAL::has_larger_distance_to_point(p, interPt1, qinterPt1) && CGAL::has_larger_distance_to_point(p, interPt2, qinterPt2)) {

								to_remove.insert(q);
							}

							
						}
						for (Point_2 q : to_remove) {
							arc_centers[p].erase(q);
						}

							if (add_v) {
								arc_centers[p].insert(v);
							}




							cout << "arc_centers[p]: ";
							for (Point_2 q : arc_centers[p]) {
								cout << q << " ";
							}
							cout << endl;

					}


				}
				else {
					init = true;
				}
				cout << "end" << endl;
				cout << "tangent 1-p: " << tangents[p].first - p << endl;
				cout << "tangent 2-p: " << tangents[p].second - p << endl;


			}
		}

		return res;

    }



};









#endif