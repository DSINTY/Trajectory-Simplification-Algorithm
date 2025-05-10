#ifndef SIM_HPP
#define SIM_HPP
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/Circular_kernel_intersections.h>
#include <CGAL/Object.h>
#include "line.hpp"
#include "point.hpp"
#include "algorithm.hpp"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;


typedef CGAL::Exact_circular_kernel_2 Circular_kernel;
typedef Circular_kernel::Line_2 Line_2;
typedef Circular_kernel::Circle_2 Circle_2;
typedef Circular_kernel::Circular_arc_2 Circular_arc_2;
typedef Circular_kernel::Circular_arc_point_2 Circular_arc_point_2;
typedef Circular_kernel::Point_2 Circular_point_2;

typedef CGAL::CK2_Intersection_traits<Circular_kernel, Circular_arc_2, Circular_arc_2>::type Intersection_result;

static bool DUMMY_BOOL = false;
static Circular_arc_point_2 DUMMY_ARC_POINT = Circular_arc_point_2();


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

	bool greater_distance(Point_2 p, Circular_arc_point_2 q, Circular_arc_point_2 r) {
		double px, py, qx, qy, rx, ry;
		px = double(p.x());
		py = double(p.y());
		qx = CGAL::to_double(q.x());
		qy = CGAL::to_double(q.y());
		rx = CGAL::to_double(r.x());
		ry = CGAL::to_double(r.y());

		double pq = (px - qx) * (px - qx) + (py - qy) * (py - qy);
		double pr = (px - rx) * (px - rx) + (py - ry) * (py - ry);
		return pq > pr;
	}

	Circular_arc_point_2 getIntersect(Line_2 line, Circle_2 c, Point_2 p, Point_2 t, bool near) {
		std::vector<pair<Circular_arc_point_2, unsigned>> res;
		CGAL::intersection(line, c, std::back_inserter(res));
		// itereate through all intersection points in reverse order
		vector<size_t> to_remove;
		for (auto it = res.begin(); it != res.end(); it++) {
			if (CGAL::angle(Point_2(CGAL::to_double(it->first.x()), CGAL::to_double(it->first.y())), p, t) != CGAL::ACUTE) {
				to_remove.push_back(it - res.begin());
			}
		}
		for (auto it = to_remove.rbegin(); it != to_remove.rend(); it++) {
			res.erase(res.begin() + *it);
		}
		if (res.size() == 0) {
			throw "ERROR: No intersection found";
		}

		Circular_arc_point_2 resPt = res[0].first;
		for (vector<pair<Circular_arc_point_2, unsigned>>::iterator it = res.begin() + 1; it != res.end(); it++) {
			Circular_arc_point_2 resTmpPt = (*it).first;
			if (near) {
				if (greater_distance(p, resPt, resTmpPt)) {
					resPt = resTmpPt;
				}
			}
			else {
				if (!greater_distance(p, resPt, resTmpPt)) {
					resPt = resTmpPt;
				}
			}
		}
		return resPt;
	}

	

	double get_sqaured_distance_point_and_arc_point(Point_2 p, Circular_arc_point_2 q) {
		double px, py, qx, qy;
		px = double(p.x());
		py = double(p.y());
		qx = CGAL::to_double(q.x());
		qy = CGAL::to_double(q.y());
		return (px - qx) * (px - qx) + (py - qy) * (py - qy);
	}

	Circular_arc_point_2 getIntersect(Line_2 line, Circular_arc_2 arc, bool& found){
		std::vector<pair<Circular_arc_point_2, unsigned>> res;
		CGAL::intersection(line, arc, std::back_inserter(res));
		//cout << "res size: " << res.size() << endl;
		if (res.size() == 0) {
			found = false;
			return Circular_arc_point_2();
		}
		else
		{
			found = true;
			return res[0].first;
		}
	}
	
	Circular_arc_point_2 getIntersect(Circular_arc_2 arc_1, Circular_arc_2 arc_2, bool& found,bool getSecond=false, bool& foundSecond = DUMMY_BOOL, Circular_arc_point_2& point_2_return = DUMMY_ARC_POINT) {
		std::vector<Intersection_result> res;
		CGAL::intersection(arc_1, arc_2, std::back_inserter(res));
		if (res.size() == 0) {
			found = false;
			return Circular_arc_point_2();
		}
		else
		{
			if ((!getSecond) && res.size() > 1) {
				cout << "Multiple intersection points found." << endl;
				// raise error
				//throw "ERROR: Multiple intersection points found";
			}
			found = true;
			 std::pair<Circular_arc_point_2, unsigned> point;
			try {
				point = CGAL::object_cast<std::pair<Circular_arc_point_2, unsigned>>((res[0]));
			}
			catch (...) {
				const Circular_arc_2 arc = CGAL::object_cast<Circular_arc_2>((res[0]));
				point_2_return = arc.target();
				
				return arc.source();
			}
				
			if (getSecond) {
				if (res.size() > 1) {
					point_2_return = CGAL::object_cast<std::pair<Circular_arc_point_2, unsigned>>((res[1])).first;
					foundSecond = true;
				}
				else {
					foundSecond = false;
				}
			}
				return point.first;
		}
		
	}

	Circular_arc_point_2 getSecondIntersect(Circular_arc_2 arc_1, Circular_arc_2 arc_2, bool& found) {
		std::vector<Intersection_result> res;
		CGAL::intersection(arc_1, arc_2, std::back_inserter(res));
		if (res.size() < 2) {
			found = false;
			return Circular_arc_point_2();
		}
		else
		{
			found = true;
			const std::pair<Circular_arc_point_2, unsigned> point = CGAL::object_cast<std::pair<Circular_arc_point_2, unsigned>>((res[1]));

			// Single intersection point found
			return point.first;
			
		}
		return Circular_arc_point_2();

	}


    Trajectory<Line>* compress(const Trajectory<Point>* traj) {

		Point_2 v = point_cast((*traj)[0]);
		bool init = false;
		bool first = true;
		Trajectory<Line>* res = new Trajectory<Line>();
		Segment_2 seg(v, v);
		unordered_set<Point_2> P;
		unordered_map<Point_2, unordered_set<Point_2>> S_i;
		unordered_map<Point_2, pair<Point_2, Point_2>> tangents;
		unordered_map<Point_2, vector<Circular_arc_2>> current_arcs;
		//Circle_2 B_v;
		P = { v };
		tangents[v] = { v, v };
		for (Point_2 p : P) {
			S_i[p] = { p };
			tangents[p] = { p, p };
			current_arcs[p] = {};
		}
		for (int i = 1; i < traj->size();i++)
		{

			if (init) {
				//Start of new segment
				init = false;
				res->push(line_from_seg(seg));
				seg = { v,v };
				P = { v };
				for (Point_2 p : P) {
					S_i[p] = { p };
					tangents[p] = { p, p };
					current_arcs[p] = {};
				}
			}
			else {
				for (Point_2 p : P) {
					if (!S_i[p].empty()) {
						seg = { p, v };
						break;
					}

				}
			}
			Point_2 old_v = v;
			v = point_cast((*traj)[i]);
			if (v == old_v) continue;
			init = true;
			Circle_2 B_v = { {v.x(), v.y()}, bound * bound };
			for (Point_2 p : P) {
				

				if ((tangents[p].first==p) && (tangents[p].second == p) &&(CGAL::squared_distance(v, p) <= bound * bound)) {
					
					init = false;
					continue;
				}
				Point_2 tangent1 = tangents[p].first;
				Point_2 tangent2 = tangents[p].second;
				if (CGAL::squared_distance(v, p) > bound * bound) {

					double dx = v.x() - p.x();
					double dy = v.y() - p.y();
					double distance = std::sqrt(dx * dx + dy * dy);

					// Calculate the angle to the center of the circle
					double angle_to_center = std::atan2(dy, dx);


					// Calculate the angle for the tangent lines using the radius
					double angle_offset = std::asin(bound / distance);

					// Calculate the angles of the tangent lines
					double angle1 = angle_to_center - angle_offset;
					double angle2 = angle_to_center + angle_offset;
					double tangent_length = std::sqrt(distance * distance - bound * bound);
					// Calculate the tangent points
					tangent1 = Point_2(p.x() + tangent_length * std::cos(angle1), p.y() + tangent_length * std::sin(angle1));
					tangent2 = Point_2(p.x() + tangent_length * std::cos(angle2), p.y() + tangent_length * std::sin(angle2));


					// swap tangent1 and tangent2 if tangent1 is on the right of tangent2
					if (CGAL::orientation(tangent1, p, tangent2) == CGAL::RIGHT_TURN) {
						Point_2 tmp = tangent1;
						tangent1 = tangent2;
						tangent2 = tmp;
					}


				}
				

				if ((tangents[p].first == p && tangents[p].second == p) || (CGAL::angle(v, p, tangents[p].first) == CGAL::ACUTE && CGAL::angle(v, p, tangents[p].second) == CGAL::ACUTE && (CGAL::orientation(v, p, tangents[p].first) != CGAL::orientation(v, p, tangents[p].second)))) {
					
					// loop through all current arcs and check whether the line from p to v cross any arc
					bool v_inside=false;
					for (auto it = current_arcs[p].begin(); it != current_arcs[p].end(); it++) {
						Circular_kernel::Point_2 p_c = { p.x(), p.y() };
						Circular_kernel::Point_2 v_c = { v.x(), v.y() };
						Line_2 pv_line(p_c, v_c );
						Circular_arc_point_2 interPt = getIntersect(pv_line, *it, v_inside);
						if (v_inside) break;
						
					}
					if (!v_inside)continue;

					bool first_pt = false;
					if (tangents[p].first == p && tangents[p].second == p) {
						first_pt = true;
					}
					

					// update the tangent points to be intersection of two tangent cones

					bool new_tangent_1 = false;
					bool new_tangent_2 = false;
					if (CGAL::orientation(tangents[p].first, p, tangent1) == CGAL::LEFT_TURN || tangents[p].first == p) {
						
						new_tangent_1 = true;
						tangents[p].first = tangent1;
					}

					
					if (CGAL::orientation(tangents[p].second, p, tangent2) == CGAL::RIGHT_TURN || tangents[p].second == p) {
						new_tangent_2 = true;
						tangents[p].second = tangent2;
					}

					
					
					Circular_kernel::Point_2 p1 = { p.x(), p.y() };
					Circular_kernel::Point_2 p2 = { tangents[p].first.x(), tangents[p].first.y() };
					Circular_kernel::Point_2 p3 = { tangents[p].second.x(), tangents[p].second.y() };

					//construct the tangent line from p to tangents[p].first
					Line_2 ptangent1 = { p1, p2 };
					//construct the tangent line from p to tangents[p].second
					Line_2 ptangent2 = { p1, p3 };
					Circular_arc_point_2 newInter1_near;
					Circular_arc_point_2 newInter2_near;
					Circular_arc_point_2 newInter1_far;
					Circular_arc_point_2 newInter2_far;


					if (CGAL::squared_distance(Circular_point_2(tangent1.x(), tangent1.y()), ptangent1) == 0) {
						
						new_tangent_1 = true;
					}
					if (CGAL::squared_distance(Circular_point_2(tangent2.x(), tangent2.y()), ptangent2) == 0) {
						new_tangent_2 = true;
					}

					if (CGAL::squared_distance(v, p) <= bound * bound) {
						new_tangent_1 = false;
						new_tangent_2 = false;
					}
					if (new_tangent_1 )
					{
						
						newInter1_near = Circular_arc_point_2(Circular_point_2(tangent1.x(),tangent1.y()));
						newInter1_far = Circular_arc_point_2(Circular_point_2(tangent1.x(), tangent1.y()));
					}
					else {
						newInter1_near = getIntersect(ptangent1, B_v, p, tangents[p].first, true);
						newInter1_far = getIntersect(ptangent1, B_v, p, tangents[p].first, false);
					}
					if (new_tangent_2)
					{
						newInter2_near = Circular_arc_point_2(Circular_point_2(tangent2.x(), tangent2.y()));
						newInter2_far = Circular_arc_point_2(Circular_point_2(tangent2.x(), tangent2.y()));
					}
					else {
						newInter2_near = getIntersect(ptangent2, B_v, p, tangents[p].second, true);
						newInter2_far = getIntersect(ptangent2, B_v, p, tangents[p].second, false);
						
					}
				
					Circular_arc_2 newArc = { B_v, newInter1_near, newInter2_near };
					Circular_arc_2 newArc_far = { B_v, newInter2_far, newInter1_far  };
					if (current_arcs[p].empty()) {
						init = false;
						
						S_i[p] = { v };
						current_arcs[p].push_back(newArc);
					}
					else {
						
						if (current_arcs[p][0].source() == current_arcs[p][0].target()) {
							// current tangent is a line
							if (!greater_distance(p, current_arcs[p][0].source(), newInter1_far)) {
								init = false;
								if (greater_distance(p, newInter1_near, current_arcs[p][0].source())) {
									current_arcs[p][0] = Circular_arc_2(current_arcs[p][0].supporting_circle(), newInter1_near, newInter1_near);
								}
							}
							continue;
						}
						Circular_arc_point_2 first_inter_old_arcs;
						bool found=false;
						// store all the arcs to be removed
						vector<size_t> arcs_to_remove;
						size_t index = 0;
						bool touch_tangent = false;
						for (vector<Circular_arc_2>::iterator it = current_arcs[p].begin(); it != current_arcs[p].end(); it++, index++) {
							
							first_inter_old_arcs = getIntersect(ptangent1, *it, found);
							

							if (found) break;
							else {
								if (it == current_arcs[p].begin()) {
									Point_2 first_target = Point_2(CGAL::to_double((*it).target().x()), CGAL::to_double((*it).target().y()));
									if (CGAL::orientation(tangents[p].first, p, first_target) == CGAL::LEFT_TURN) {
										touch_tangent = true;
										found = true;
										first_inter_old_arcs = (*it).source();
										break;
									}
								}
								arcs_to_remove.push_back(index);
							}
						}
						for (auto it = arcs_to_remove.rbegin(); it != arcs_to_remove.rend(); ++it) {
							current_arcs[p].erase(current_arcs[p].begin() + *it);
						}
						if (!found) {

							
							

							cout << "ERROR: No intersection found on old arc" << endl;
							throw "ERROR: No intersection found on old arc";
							
						}
						if (!touch_tangent)current_arcs[p][0] = Circular_arc_2(current_arcs[p][0].supporting_circle(), first_inter_old_arcs, current_arcs[p][0].target());
						
						Circular_arc_point_2 last_inter_old_arcs;
						for (vector<Circular_arc_2>::iterator it = current_arcs[p].begin(); it != current_arcs[p].end(); it++) {
							last_inter_old_arcs = getIntersect(ptangent2, *it, found);
							if (found) {
								
								(*it) = Circular_arc_2((*it).supporting_circle(), (*it).source(), last_inter_old_arcs);
								current_arcs[p].erase(it + 1, current_arcs[p].end());
								break;
							}

						}
						

						if (greater_distance(p, first_inter_old_arcs, newInter1_far)) {
							// new point "in front" of old arc, need to shrink tangent
								bool haveIntersect=false;
								Circular_arc_point_2 old_new_inter;
								vector<size_t> to_remove;
								size_t index = 0;
								bool foundSecond = false;
								Circular_arc_point_2 second_inter;
		
								for (vector<Circular_arc_2>::iterator it = current_arcs[p].begin(); it != current_arcs[p].end(); it++) {
									
									old_new_inter = getIntersect(*it, newArc_far, found, true, foundSecond, second_inter);
									if (found) {
										haveIntersect = true;
										break;
									}
									else {
										to_remove.push_back(index);
									}
								}
								for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
									current_arcs[p].erase(current_arcs[p].begin() + *it);
								}
								if (haveIntersect) {
									init = false;
								}
								else continue;
								// set new tangent point 1 to be the intersection of the old arc and the new arc
								if (foundSecond) {
									// check orientation of the two intersection points
									if (CGAL::orientation(Point_2(CGAL::to_double(old_new_inter.x()), CGAL::to_double(old_new_inter.y())), p, Point_2(CGAL::to_double(second_inter.x()), CGAL::to_double(second_inter.y()))) == CGAL::RIGHT_TURN) {
										// swap intersection points
										Circular_arc_point_2 tmp = old_new_inter;
										old_new_inter = second_inter;
										second_inter = tmp;
									}
								}

								tangents[p].first = Point_2(CGAL::to_double(old_new_inter.x()), CGAL::to_double(old_new_inter.y()));
								current_arcs[p][0] = Circular_arc_2(current_arcs[p][0].supporting_circle(), old_new_inter, current_arcs[p][0].target());

								//Circular_arc_point_2 second_inter = getSecondIntersect(current_arcs[p][0], newArc_far, foundSecond);
								if (foundSecond) {
									// set new tangent point 1 to be the intersection of the old arc and the new arc
									tangents[p].second = Point_2(CGAL::to_double(second_inter.x()), CGAL::to_double(second_inter.y()));
									current_arcs[p][0] = Circular_arc_2(current_arcs[p][0].supporting_circle(), current_arcs[p][0].source(), second_inter);
									current_arcs[p].erase(current_arcs[p].begin() + 1, current_arcs[p].end());
									v = old_v;
									
									continue;
								}
									
								for (vector<Circular_arc_2>::iterator it = current_arcs[p].begin()+1 ; it != current_arcs[p].end(); it++) {
									second_inter = getIntersect(*it, newArc_far, foundSecond);
									if (foundSecond) {
										tangents[p].second = Point_2(CGAL::to_double(second_inter.x()), CGAL::to_double(second_inter.y()));
										(*it) = Circular_arc_2((*it).supporting_circle(), (*it).source(), second_inter);
										current_arcs[p].erase(it + 1, current_arcs[p].end());
										break;
									}
									
								}
								v = old_v;
								





							}
							else {
							
								init = false;
								S_i[p] = { v };
								bool onNewArc = false;
								size_t currentArcIndex=0;
								Circular_arc_point_2 current_start;
								Circular_arc_point_2 current_end;
								if (greater_distance(p, first_inter_old_arcs, newInter1_near)|| (CGAL::squared_distance(v, p) <= bound * bound)) {
									onNewArc = false;
									
									
								}
								else {
									onNewArc = true;
									current_start = newInter1_near;
									
								}
								bool reached_end = false;
								while (!reached_end){
									
									if (onNewArc) {
										int remove_count = 0;
										reached_end = true;
										for (vector<Circular_arc_2>::iterator it = current_arcs[p].begin() + currentArcIndex; it != current_arcs[p].end(); it++) {
											
											Circular_arc_point_2 inter = getIntersect(*it, newArc, found);
											if (found) {
												(*it) = Circular_arc_2((*it).supporting_circle(), inter, (*it).target());
												current_end = inter;
												onNewArc = false;
												reached_end = false;
												break;
											}
											else {
												remove_count++;
											}
											
										}
										if (remove_count > 0) {
											current_arcs[p].erase(current_arcs[p].begin() + currentArcIndex, current_arcs[p].begin() + currentArcIndex + remove_count);
										}
										if (reached_end) {
											current_end = newInter2_near;
											
										}
										
										current_arcs[p].insert(current_arcs[p].begin() + currentArcIndex, Circular_arc_2(newArc.supporting_circle(),current_start,current_end));
										if (!reached_end) {
											currentArcIndex++;
										}
									}
									else {
										reached_end = true;
										found = false;
										for (vector<Circular_arc_2>::iterator it = current_arcs[p].begin()+currentArcIndex; it != current_arcs[p].end(); it++, currentArcIndex++) {
											
											Circular_arc_point_2 inter = getIntersect(*it, newArc, found);
										
											if (found) {
												
												(*it) = Circular_arc_2((*it).supporting_circle(), (*it).source(), inter);
												current_start = inter;
												onNewArc = true;
												currentArcIndex++;
												reached_end = false;
												break;
											}
										}
										
									}

									

							}
							
						}
						

					}


				}
				


			}

		}
		

		
		if (!init) {
			for (Point_2 p : P) {
				if (!S_i[p].empty()) {
					seg = { p, v };
					break;
				}

			}
		}
		res->push(line_from_seg(seg));

		if (init) res->push(line_from_seg({ v,v }));

		return res;

    }



};









#endif