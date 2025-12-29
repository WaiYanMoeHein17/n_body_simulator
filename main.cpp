#include "NBodySimulation.h"
#include "thread_safety.h"
#include "debug.h"
#include "IO.h"

#include <iostream>
#include <cstddef>
#include <vector>
#include <omp.h>
#include <concepts>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <type_traits>
#include <utility>
#include <array>
#include <chrono>
#include <span>

constexpr double gravity{ 9.81 };       

template <typename T> 
concept Numeric = std::is_arithmetic_v<T>; 

template <typename C> 
concept Class = std::is_class_v<C>; 

template <typename T, typename...Args>
constexpr T constexpr_passer(Args...argument, T(*function)(...)) {
    return function(argument); 
}

static constexpr int factorial(const int number) {
#ifdef ENABLE_DEBUG
    std::cout << "factorial called" << "\n"; 
#endif
    if (number <= 1) {
        return 1; 
    }
    int answer{ 1 }; 
    for (int i{ 2 }; i <= number; ++i) {
        answer *= i; 
    }
    return answer; 
}

template <Numeric T>
T calculate_magnitude(const std::span<T>& first, const std::span<T>& second) {
#ifdef ENABLE_DEBUG
    std::cout << "caculate_magnitude called" << "\n"; 
#endif
	return std::sqrt((second[0] - first[0]) * (second[0] - first[0]) +
		(second[1] - first[0]) * (second[1] - first[0]) +
		(second[2] - first[2]) * (second[2] - first[2]));
}

template <Numeric T> 
T compute_force(const Body& body_one, const Body& body_two) {
#ifdef ENABLE_DEBUG
    std::cout << "compute_force called" << "\n"; 
#endif
    T force{ gravity * (body_one.mass * body_two.mass) 
        / calculate_magnitude(body_one.x, body_two.x) * (body_two.x - body_one.x) };
	return force;
}

template <Numeric T, Class C> 
T sum_forces(C body, std::vector<C> bodies) {
    T total_force{}; 
    size_t bodies_size{ bodies.size() };
    for (size_t i{}; i < bodies_size; ++i) {
        total_force += compute_force(body, bodies[i]); 
    }
    return total_force; 
}

template<Numeric T, Class C> 
void explicit_euler_single_timestep(std::vector<C> bodies, T timestep) {
    size_t bodies_size{ bodies.size() }; 
    for (size_t i{}; i < bodies_size; ++i) {
        T acceleration = sum_forces(bodies[j]) / bodies[j].mass; 
        for (int j{}; j < 3; ++j) {
            bodies[i].x[j] += bodies[i].v[j] * timestep; 
            bodies[i].v[j] += acceleration * timestep; 
        }
        collision_handler(bodies);
    }
}

template<Numeric T> 
void explicit_euler_soa() {

}

template<Numeric T, Class C> 
void explicit_euler(std::vector<C> bodies, std::vector<C> lagging_bodies, T timestep, T endtime) {
    // @ params
    // timestep         = delta t, dt
    // 
    // v_i = v_i-1 + F_i/m_i delta(t), r_i
    std::array<double, 3> initial_v{ 0,0,0 };                                   // std::array, stack access instead of heap (faster)
    std::array<double, 3> initial_p{ 0,0,0 };      
    std::vector<T> initial_velocity{ 0,0,0 };                                   // std::vector container, heap access (slower) 
    std::vector<T> initial_position{ 0,0,0 };

    // for collision handling 
    std::set<std::unordered_map<std::vector<T>,std::vector<T>>> positions;      // unique unordered map where key = position, value = velocities at position
    // #pragma omp parallel
    for (int i{}; i < endtime; i += timestep) {
        for (size_t j{}; j < bodies.size(); ++j) {
            T acceleration = sum_forces(bodies[j]) / bodies[j].mass; 
            for (int k{}; k < 3; ++k) {
                lagging_bodies[j].x[k] = bodies[j].x[k];                        // update lagging bodies position   
                lagging_bodies[j].v[k] = bodies[j].v[k];                        // update lagging bodies velocity
                bodies[j].x[k] += bodies[j].v[k] * timestep;                    // update current bodies position
                bodies[j].v[k] += acceleration * timestep;                      // update current bodies velocity
            }

        }
        collision_handler(bodies, lagging_bodies); 
    //    for (auto& body : bodies) {
    //        T acceleration = sum_forces(body)/body.mass;
    //        
    //        // can definitely vectorize this part
    //        for (int j{}; j < 3; ++j) {
    //            lagging_bodies.
    //        }
    //        body.x[0] = body.x[0] + (body.v[0] * timestep);
    //        body.x[1] = body.x[1] + (body.v[1] * timestep);
    //        body.x[2] = body.x[2] + (body.v[2] * timestep);
    //        body.v[0] = body.v[0] + acceleration * timestep;
    //        body.v[1] = body.v[1] + acceleration * timestep;
    //        body.v[2] = body.v[2] + acceleration * timestep;
    //        
			
    //        // real time collision handling
    //        /*if (positions.count(body.x)) {
				//// collision
    //            //collision_handler(); 
    //        }
    //        else { 
    //            // no collision
    //            positions.insert(body.x); 
    //            positions[body.x] = body.v; 
    //        }*/ 

    //    }
        // check collisions here after one timestep  
    }
}

//template <Numeric T> 
//void update_velocities_and_positions(std::function<T>(), T timestep) {
//    // @params: 
//    // function         = the time stepping scheme
//    // 
//
//}

//template <Numeric T> 
//void collision_handler(Body& body_one, Body& body_two) {
//    // this function checks in real time if two bodies have collided
//    std::cout << "Working" << "\n"; 
//}
//
//template <Numeric T> 
//void collision_handler(std::vector<Body>& bodies, std::vector<LaggingBody>& lagging_bodies) {
//    // this function checks after each timestep
//    // might need a pixel buffer +1 to combat floating point precision
//    // want to check position of all bodies and check for collisions
//    // need to keep track of the position of every body
//    // collision check 
//    for (size_t i{0}; i < bodies.size() - 1; ++i) {             // O(n) 
//        for (size_t j{i+1}; j < bodies.size(); ++j) {       // O(n)
//            //  && bodies[i].x == bodies[j].x
//            T total_mass = bodies[i].mass + bodies[j].mass; 
//            if (calculate_magnitude<T>(bodies[i].x,bodies[j].x) <= C*(total_mass)) {
//                // collision
//                // formula: 
//                // r = m_i * r_i + m_j * r_j / m_i + m_j
//                // v = m_i * v_i + m_h * v_j / m_i + m_j
//                // need the positions and velocity a timestep before the collision
//                // when they collide, we merge into body[i]
//
//                bodies[i].x[0] = bodies[i].mass * lagging_bodies[i].x[0] + bodies[j].mass * lagging_bodies[j].x[0] / total_mass; 
//                bodies[i].x[1] = bodies[i].mass * lagging_bodies[i].x[1] + bodies[j].mass * lagging_bodies[j].x[1] / total_mass; 
//                bodies[i].x[2] = bodies[i].mass * lagging_bodies[i].x[2] + bodies[j].mass * lagging_bodies[j].x[2] / total_mass;
//				bodies[i].v[0] = bodies[i].mass * lagging_bodies[i].v[0] + bodies[j].mass * lagging_bodies[j].x[0] / total_mass;
//				bodies[i].v[1] = bodies[i].mass * lagging_bodies[i].v[1] + bodies[j].mass * lagging_bodies[j].x[1] / total_mass;
//				bodies[i].v[2] = bodies[i].mass * lagging_bodies[i].v[2] + bodies[j].mass * lagging_bodies[j].x[2] / total_mass;
//                bodies[i].mass += bodies[j].mass; 
//
//                
//            }
//        }
//    }
//}

template <Numeric T, Class C> 
void collision_handler(std::vector<C>& bodies, const double C) {
    size_t bodies_size{ bodies.size() }; 
    for (size_t i{}; i < bodies_size; ++i) {
        for (size_t j{i+1}; j < bodies_size; ++j) {
            if (calculate_magnitude<T>(bodies[i].x, bodies[j].x) <= C * (bodies[i].mass + bodies[j].mass)) {
                double total_mass{ bodies[i].mass + bodies[j].mass }; 
                for (int k{ 0 }; k < 3; ++k) {
					bodies[i].x[k] = (bodies[i].mass * bodies[i].x[k]
						+ bodies[j].mass * bodies[j].x[k])
						/ total_mass;

					bodies[i].v[k] = (bodies[i].mass * bodies[i].v[k]
						+ bodies[j].mass * bodies[j].v[k])
						/ total_mass;
                }
                bodies[i].mass = total_mass; 
            }
        }
    }
}

template <Numeric T, Class C>
void collision_handler(std::vector<C>& bodies, std::vector<C>& lagging_bodies, const double C) {
    // might need to deal with floating point precision when colliding. 
	std::vector<bool> merged(bodies.size(), false);

	for (size_t i = 0; i < bodies.size(); ++i) {
		if (merged[i]) continue;

		for (size_t j = i + 1; j < bodies.size(); ++j) {
			if (merged[j]) continue;

			if (calculate_magnitude<T>(bodies[i].x, bodies[j].x) <= C * (bodies[i].mass + bodies[j].mass)) {
				double total_mass = bodies[i].mass + bodies[j].mass;

				for (int k = 0; k < 3; ++k) {
					bodies[i].x[k] = (bodies[i].mass * lagging_bodies[i].x[k]
						+ bodies[j].mass * lagging_bodies[j].x[k])
						/ total_mass;

					bodies[i].v[k] = (bodies[i].mass * lagging_bodies[i].v[k]
						+ bodies[j].mass * lagging_bodies[j].v[k])
						/ total_mass;
				}

				bodies[i].mass = total_mass;
				merged[j] = true;
			}
		}
	}
	for (int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i) {
		if (merged[i]) {
			bodies.erase(bodies.begin() + i);
			lagging_bodies.erase(lagging_bodies.begin() + i);
		}
	}
}

int main(int argc, char** argv) {
    auto start = std::chrono::steady_clock::now();      // monotonic, best for testing
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " input_file.txt\n";
        return 1;
    }
    NBodySimulation nbs(argv[1]);                       // initialization of object, setup 3 different data structures
    
    // Write initial snapshot
    openPVDFile();
    int snapshotCounter = 0;
    writeVTKSnapshot(nbs, snapshotCounter);
    addSnapshotToPVD(snapshotCounter);

    int N = (int)nbs.number_of_bodies;  
    double C{ 0.01 / N };   

    for (double t{}; t < nbs.endtime; t += nbs.dt) {
        if (t >= nbs.snapshot_interval) {
            snapshotCounter += 1; 
			writeVTKSnapshot(nbs, snapshotCounter);
			addSnapshotToPVD(snapshotCounter);
			nbs.snapshot_interval += nbs.snapshot_interval;
			std::cout << "Plot next snapshot"
				<< ",\t t=" << t
				<< ",\t dt=" << nbs.dt
				<< ",\t N=" << nbs.bodies.size()
				<< std::endl;
			// In addition to the above quantities you may want to track maximum velocity 
			// and smallest distance between masses. This is particularly useful when implementing collisions.
        }
    }

   closePVDFile();
   auto simulation_time{ std::chrono::steady_clock::now() };
   std::chrono::duration<double> time_elapsed{ simulation_time - start };
   std::cout << "Simulation completed in " << time_elapsed.count() << " seconds.\n";
   
   return 0;
}

