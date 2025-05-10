#ifndef HELPER_HPP
#define HELPER_HPP
#include "trajectory.hpp"
#include "point.hpp"
#include "line.hpp"

Trajectory<Line>* point_to_line(const Trajectory<Point>* traj) {
	Trajectory<Line>* res = new Trajectory<Line>();
	for (int i = 0; i < traj->size() - 1; i++) {
		res->push(Line{ (*traj)[i],(*traj)[i + 1] });
	}
	return res;
}

Trajectory<Point>* line_to_point(const Trajectory<Line>* traj) {
	Trajectory<Point>* res = new Trajectory<Point>();
	res->push((*traj)[0].start_point());
	for (int i = 0; i < traj->size(); i++) {
		if (((*traj)[i].start_point()).distance(((*res)[res->size()-1]))==0) {
			res->push((*traj)[i].start_point());
		}
		if (((*traj)[i].end_point()).distance(((*res)[res->size() - 1])) == 0) {
			res->push((*traj)[i].end_point());
		}
	}
	
	return res;
}


//double frechet_distance(Trajectory<Point>* traj1, Trajectory<Point>* traj2) {
//	//frechet distance
//
//}





#endif