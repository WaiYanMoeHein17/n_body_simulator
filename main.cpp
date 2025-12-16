#include <iostream>
#include <cmath>
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
#include "NBodySimulation.h"
#include "IO.h"
hello
constexpr double gravity{ 9.81 };       // gravity

template <typename T> 
concept Numeric = std::is_arithmetic_v<T>; 

template <Numeric T> 
T factorial(T number) {
    if (number <= 1) {
        return 1; 
    }
    T answer{ 1 }; 
    for (T i{ 2 }; i <= number; ++i) {
        answer *= i; 
    }
    return answer; 
}

template <Numeric T> 
T sqrt(T num) {
    return; 
}

template <Numeric T>
T calculate_magnitude(const T* first, const T* second) {
	T dx = second[0] - first[0];
	T dy = second[1] - first[1];
	T dz = second[2] - first[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

template <Numeric T> 
T calculate_magnitude(std::vector<T> first, std::vector<T> second) {
    // perhaps be vectorized
    T magnitude = std::sqrt((second[0]-first[0])*(second[0]-first[0]) + 
                (second[1]-first[0])*(second[1]-first[0]) +
                (second[2]-first[2])*(second[2]-first[2])); 
    return magnitude; 
}

template <Numeric T> 
T compute_forces(const std::pair<Body, Body>& body_pair) {
    // for each pair of bodies i, j, return gravitational force
    // formula: 
    // F_ij = G * ( mass of i * mass of j) / abs( pos of j - pos of i )^3 * (pos of j - pos of i)
    // subtracting position and powering it is vector subtraction and multiplication
    // however it is just for one iteration with each vector limited at size 3. 
    T mass_i{ body_pair.first.mass };
    T mass_j{ body_pair.second.mass };
    T pos_i{ body_pair.first.x };
    T pos_j{ body_pair.second.x };
    T force{ gravity * (mass_i * mass_j) / calculate_magnitude(pos_i, pos_j) * (pos_j - pos_i) };
    return force; 
}

template <Numeric T> 
T compute_forces(const Body& body_one, const Body& body_two) {
    // for each pair of bodies i, j, return gravitational force
    // formula: 
    // F_ij = G * ( mass of i * mass of j) / abs( pos of j - pos of i )^3 * (pos of j - pos of i)
    // subtracting position and powering it is vector subtraction and multiplication
    // however it is just for one iteration with each vector limited at size 3. 
    T mass_i{ body_one.mass };
    T mass_j{ body_two.mass };
    T pos_i{ body_one.x };
    T pos_j{ body_two.x };
    T force{ gravity * (mass_i * mass_j) / calculate_magnitude(pos_i, pos_j) * (pos_j - pos_i) };
	return force;
}

template<Numeric T> 
std::unordered_map<Body, T> sum_forces(std::vector<Body> bodies) {
    std::unordered_map<Body, T> forces; 
    for (size_t i = 0; i < bodies.size(); ++i) {        // O(n)
        for (size_t j = 0; j < bodies.size(); ++j) {    // O(n) 
            forces[bodies[i]] += compute_forces(std::make_pair(bodies[i], bodies[j])); 
        }
    }
    return forces; 
}

template <Numeric T> 
T sum_forces(Body body, std::vector<Body> bodies) {
    T total_force{}; 
    for (size_t i{}; i < bodies.size(); ++i) {
        total_force += compute_forces(body, bodies[i]); 
    }
    return total_force; 
}

template<Numeric T> 
void ader_timestepping(T initial, T order, T timestep, T endtime, Function<T>& function) {
    T curr_val = initial;
    for (T t = 0; t < end_time; t += timestep) {
        T increment = function.evaluate(t);
        // Compute Taylor series terms
        for (T k = 1; k < M; ++k) {
            auto temp = function.clone(); // clone
            // Compute k-th derivative
            for (int j = 0; j < k; ++j) {
                temp->derive();
            }
            T derivative_val = temp->evaluate(t);
            T term = derivative_val * std::pow(timestep, k) / factorial(k);
            increment += term;
        }
        curr_val += timestep * increment;
        std::cout << "Time Step: " << t << " | Value: " << curr_val << std::endl;
    }
}

template <Numeric T> 
void runge_kutta(std::vector<Body> bodies) {
	/*k1 = f(t, y)
		k2 = f(t + Δt / 2, y + k1·Δt / 2)
		k3 = f(t + Δt / 2, y + k2·Δt / 2)
		k4 = f(t + Δt, y + k3·Δt)
		y(t + Δt) = y(t) + Δt / 6·(k1 + 2k2 + 2k3 + k4)*/


}

template<Numeric T> 
void explicit_euler_single_timestep(std::vector<Body> bodies, T timestep) {
    for (size_t i{}; i < bodies.size(); ++i) {
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

template<Numeric T> 
void explicit_euler(std::vector<Body> bodies, std::vector<Body> lagging_bodies, T timestep, T endtime) {
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

template <Numeric T> 
void collision_handler(std::vector<Body>& bodies, const double C) {
    
    for (size_t i{}; i < bodies.size(); ++i) {
        for (size_t j{i+1}; j < bodies.size(); ++j) {
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

template <Numeric T>
void collision_handler(std::vector<Body>& bodies, std::vector<Body>& lagging_bodies, const double C) {
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
    writeVTKSnapshot(nbs, AOS, snapshotCounter);
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
   auto simulation_time = std::chrono::steady_clock::now(); 
   std::chrono::duration<double> time_elapsed = simulation_time - start; 
   std::cout << "Simulation completed in " << time_elapsed.count() << " seconds.\n";
   
   return 0;
}

