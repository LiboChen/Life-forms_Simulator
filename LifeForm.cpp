/* main test simulator */
#include <iostream>
#include "CraigUtils.h"
#include "Window.h"
#include "tokens.h"
#include "ObjInfo.h"
#include "QuadTree.h"
#include "Params.h"
#include "LifeForm.h"
#include "Event.h"

using namespace std;

template <typename T>
void bound(T& x, const T& min, const T& max) {
	assert(min < max);
	if (x > max) { x = max; }
	if (x < min) { x = min; }
}



ObjInfo LifeForm::info_about_them(SmartPointer<LifeForm> neighbor) {
	ObjInfo info;

	info.species = neighbor->species_name();
	info.health = neighbor->health();
	info.distance = pos.distance(neighbor->position());
	info.bearing = pos.bearing(neighbor->position());
	info.their_speed = neighbor->speed;
	info.their_course = neighbor->course;
	return info;
}

/************************  move around  *****************************************************/
void LifeForm::update_position(void){
	//std::cout << "clb::update position " << std::endl;
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
	SimTime time_passed = Event::now() - update_time;
	energy = energy - movement_cost(speed, time_passed); //energy consume
	//case 1: run out of energy   case 2: go out of space
	if(energy < min_energy || space.is_out_of_bounds(pos)) this->die();

	if(!is_alive ) return;    //dead lifeform call update_position will cause problem!!
							  //it will find a leaf with a new lifeform's pos, different from this pos
//	std::cout << "time passed is " << time_passed << std::endl;
	if(time_passed > 0.001){
//		if(id == 603) std::cout << "603 lifeform update postion with old position" << pos.xpos << " " << pos.ypos << std::endl;
//		if(id == 603 && pos.xpos == 160.355 && pos.ypos == 186.467) std::cout << "I found you!" << pos.xpos << " " << pos.ypos << std::endl;
//		std::cout << id << " lifeform is updating position" << std::endl;
//		if(id == 603) std::cout << "603 lifeform life status is" << this->is_alive << std::endl;
		Point new_pos;
		new_pos.xpos = pos.xpos + speed * time_passed * cos(course);
		new_pos.ypos = pos.ypos + speed * time_passed * sin(course);
		if(space.is_out_of_bounds(new_pos)){
			this->die();
			return;
		}
		update_time = Event::now();  //must be before space.update_position!!!
		space.update_position(pos, new_pos);
//		std::cout << "clb::update position end1" << std::endl;
		pos = new_pos;

	}
//	std::cout << "clb::update position end" << std::endl;

}


void LifeForm::border_cross(void){
//	std::cout << "clb::border cross " << std::endl;
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
//	if(id == 603) std::cout << "603 lifeform is crossing border "<< pos.xpos << " " << pos.ypos << std::endl;
//	std::cout << id << " lifeform is crossing border "<< pos.xpos << " " << pos.ypos << std::endl;
	if(!is_alive) return;
	update_position();
	//schedule the next movement event
	compute_next_move();

	//check_encounter?  schedule an event or directly invoke the function
	check_encounter();
}



void LifeForm::region_resize(void){
	//std::cout << "clb::region region_resize " << std::endl;
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
	if(!is_alive) return;
//	if(id == 603) std::cout << "603 lifeform is region_resize in position " << pos.xpos << " "<< pos.ypos << std::endl;
//	std::cout << id <<" lifeform is region_resize" << std::endl;
	border_cross_event->cancel();
	update_position();
	compute_next_move();

}


void LifeForm::set_course(double _course){
//	std::cout << "clb::set course " << std::endl;
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
	if(!is_alive) return;
	//if(id == 603) std::cout << "603 lifeform is set course" << std::endl;
//	std::cout << id <<  " lifeform is set_course with life status " << is_alive << std::endl;
	border_cross_event->cancel();
	update_position();
//	std::cout << id <<  " lifeform is still set_course with life status " << is_alive << std::endl;
	course = _course;
	compute_next_move();

}

void LifeForm::set_speed(double _speed){
//	std::cout << "clb::set speed " << std::endl;
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
	if(!is_alive) return;
//	if(id == 603) std::cout << "603 lifeform is set_speed" << std::endl;
//	std::cout << "other lifeform is set_speed" << std::endl;
	border_cross_event->cancel();
	update_position();
	speed = (_speed > max_speed)? max_speed: _speed;
	compute_next_move();
}


void LifeForm::compute_next_move(void){
	//std::cout << "clb::compute next move " << std::endl;
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
	if(!is_alive ) return;
	//if pos is out of bound, when distance_to_edge will call find_leaf, assert in bound will fail!!
	if(speed > 0){
		SimTime cross_time = (Point::tolerance + space.distance_to_edge(pos, course)) / speed;
		SmartPointer<LifeForm> self = SmartPointer<LifeForm>(this);
		border_cross_event = new Event(cross_time, [self](void){	self->border_cross(); });
	}
}



/************************************** Encounters ***************************************************/
void LifeForm::check_encounter(void){
//	std::cout << "clb::check encounter " << std::endl;
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
	if(!is_alive ) return;
	SmartPointer<LifeForm> y;
	do{
		y = space.closest(pos);
	}while(&*y == this);

	y->update_position();
	if(y && y->position().distance(pos) <= encounter_distance){
		resolve_encounter(y);
	}
}


void LifeForm::resolve_encounter(SmartPointer<LifeForm> y){
//	if(id == 603) std::cout << "603 lifeform is resolve encounter" << std::endl;
	//apply penalty first
	energy = energy - encounter_penalty;
	(*y).energy = (*y).energy - encounter_penalty;

	SmartPointer<LifeForm> x(this);


	//x->update_position(); //in update_position, it will check energy and postion
	//y->update_position(); //if low than min or out of bounds, it will d ie

	//check alive?? if both are alive
	if((*x).is_alive && (*y).is_alive){
		Action x_act = x->encounter(x->info_about_them(y));
		Action y_act = y->encounter(y->info_about_them(x));

		if(x_act == LIFEFORM_EAT && y_act == LIFEFORM_IGNORE){
			if(drand48() < eat_success_chance((*x).energy, (*y).energy))	x->eat(y);
		}
		else if(x_act == LIFEFORM_IGNORE && y_act == LIFEFORM_EAT){
			if(drand48() < eat_success_chance((*y).energy, (*x).energy))	y->eat(x);
		}
		else if(x_act == LIFEFORM_EAT && y_act == LIFEFORM_EAT){
			bool x_can_eat = drand48() < eat_success_chance((*x).energy, (*y).energy);
			bool y_can_eat = drand48() < eat_success_chance((*y).energy, (*x).energy);

			if(x_can_eat && y_can_eat){  //solve tie situation
				switch(encounter_strategy){
				case EVEN_MONEY:
					if(drand48()<0.5) x->eat(y);
					else y->eat(x);
					break;
				case BIG_GUY_WINS:
					if((*x).energy >= (*y).energy) x->eat(y);
					else y->eat(x);
					break;
				case UNDERDOG_IS_HERE:
					if((*x).energy <= (*y).energy) x->eat(y);
					else y->eat(x);
					break;
				case FASTER_GUY_WINS:
					if((*x).speed >= (*y).speed) x->eat(y);
					else y->eat(x);
					break;
				case SLOWER_GUY_WINS:
					if((*x).speed <= (*y).speed) x->eat(y);
					else y->eat(x);
					break;
				default:
					break;
				}
			}
			else if(x_can_eat && !y_can_eat)	x->eat(y);
			else if(!x_can_eat && y_can_eat)	y->eat(x);
			else {}
		}
		else{}
	}

	else if((*x).is_alive && !(*y).is_alive)	Action x_act = x->encounter(x->info_about_them(y));
	else if(!(*x).is_alive && (*y).is_alive) 	Action y_act = y->encounter(y->info_about_them(x));
	else {}
}


/************************************* Eat *****************************************************/
void LifeForm::eat(SmartPointer<LifeForm> that){
	that->die();
	SmartPointer<LifeForm> self = SmartPointer<LifeForm>(this); // can be detelete??
	this->energy -= eat_cost_function();
	if(energy < min_energy) {this->die(); return; }
	new Event(digestion_time , [self, that](void){ self->gain_energy((*that).energy * eat_efficiency);} );
	//here has some problem
}

void LifeForm::gain_energy(double gained_energy){
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
	if(!is_alive ) return;
	(*this).energy += gained_energy;
}


/************************************** Age ***************************************************/
void LifeForm::age(void){
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
		return;
	}
	if(!is_alive ) return;
	(*this).energy -= age_penalty;
	if((*this).energy < min_energy) this->die();
	if((*this).is_alive){
		SmartPointer<LifeForm> self = SmartPointer<LifeForm>(this);
		new Event(age_frequency, [self](void) { self->age(); });
	}
}


/************************************ Reproduce ***************************************************/
void LifeForm::reproduce(SmartPointer<LifeForm> child){
	if((Event::now() - reproduce_time) < min_reproduce_time) { child = nullptr; return; }

	update_position();  //to get most recent energy and check if still alive
	if(energy < min_energy || space.is_out_of_bounds(pos))	{ this->die(); child = nullptr; return; }

	if(!is_alive) {child = nullptr; return; }
	SmartPointer<LifeForm> nearest;
	//begin reproduce
	//std::cout << "start!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	int count = 0;
	do{ //how to generate random position???
		do{
			double distance = drand48() * (reproduce_dist - encounter_distance) + encounter_distance + 0.001;
			double angle = drand48() * 2.0 * M_PI;
			child->pos.xpos = cos(angle) * distance + this->pos.xpos;
			child->pos.ypos = sin(angle) * distance + this->pos.ypos;
			if(++count > 5) {child = nullptr; return;}
		}while(space.is_out_of_bounds(child->pos));
		if(++count > 5) {child = nullptr; return; }
		nearest = space.closest(child->pos);
	}while(nearest && nearest->position().distance(child->position()) <= encounter_distance);
	// nearest???


	if(space.is_out_of_bounds(child->position())) {
		child=nullptr;
		return;
	}

	double new_energy = this->energy / 2 * (1 - reproduce_cost);
	if(new_energy < min_energy) {	child = nullptr; this->die(); return; }
	this->energy = new_energy;
	child->energy = new_energy;

	//insert into quadtree
	if(!is_alive) return;
	child->start_point = child->pos;
	reproduce_time = Event::now();
	space.insert(child, child->pos, [child]() { child->region_resize(); });
	(void) new Event(age_frequency, [child](void) { child->age(); });
	child->is_alive = true;
	//std::cout << "end!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
}

/************************************ Perceive ***************************************************/
ObjList LifeForm::perceive(double radius){
	if(radius > max_perceive_range) radius = max_perceive_range;
	if(radius < min_perceive_range) radius = min_perceive_range;

	//perceive
	ObjList objs_info;
	std::vector<SmartPointer<LifeForm>> objs;   //pointer to nearby objects
	this->update_position();
	if(space.is_out_of_bounds(pos) || energy<min_energy) {
		this->die();
	}
	if(!is_alive ) return objs_info;
	objs = space.nearby(this->pos, radius);
	//for(auto it = objs.begin(); it != objs.end(); it++){
	//	objs_info.push_back(this->info_about_them(*it));
	//}

	for(auto it:objs){
		if(&*it!=this){
		SmartPointer<LifeForm> a(it);
	    objs_info.push_back(info_about_them(a));
	    }
	 }

	this->energy -= perceive_cost(radius);
	if(this->energy < min_energy){
		this->die();
		border_cross_event->cancel();
	}
	return objs_info;
}


