#ifndef NBODY_SIMULATION_H
#define NBODY_SIMULATION_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <array>
#include <memory>
#include <utility>

constexpr size_t block_size{ 4 };

struct Body {
	// AoS vs SoA vs AoSoA
    double x[3];						// position in x, y, z
    double v[3];						// velocity in x, y, z
    double mass;						// mass of body
};

// Body's position and velocity one time step behind
struct BodyAoS {
	std::array<double, 3> x; 
	std::array<double, 3> v; 
	double mass; 
};

struct BodySoA {
	// I don't want to use vectors because they are not contiguous
	// affecting cache locality and they take a longer time to 
	// allocate to heap compared to stack (faster allocation)

	std::unique_ptr<double[]> x; 
	std::unique_ptr<double[]> y; 
	std::unique_ptr<double[]> z; 
	std::unique_ptr<double[]> velocity_x; 
	std::unique_ptr<double[]> velocity_y; 
	std::unique_ptr<double[]> velocity_z; 
	std::unique_ptr<double[]> mass;
	size_t number_of_bodies; 

	BodySoA(long long number_of_bodies) : number_of_bodies(number_of_bodies) {
		x = std::make_unique<double[]>(number_of_bodies);			
		y = std::make_unique<double[]>(number_of_bodies);
		z = std::make_unique<double[]>(number_of_bodies);
		velocity_x = std::make_unique<double[]>(number_of_bodies);
		velocity_y = std::make_unique<double[]>(number_of_bodies);
		velocity_z = std::make_unique<double[]>(number_of_bodies);
		mass = std::make_unique<double[]>(number_of_bodies);
	}

	BodySoA() : number_of_bodies(0) {}
};

struct BodyAoSoA {
	// experiment with different block sizes
	// mainly to preserve cache locality 

	std::array<double, block_size> x; 
	std::array<double, block_size> y; 
	std::array<double, block_size> z; 
	std::array<double, block_size> velocity_x; 
	std::array<double, block_size> velocity_y; 
	std::array<double, block_size> velocity_z; 
	std::array<double, block_size> mass;  

};

class NBodySimulation {
public:
    double snapshot_interval;
    double endtime;
    double dt;
	long long number_of_bodies{};

    // If you exchange this data structure, make sure to update the constructor as well
    // If need be add a destructor to the class

	BodySoA bodies_soa;								// bodies Structure of Arrays
	std::vector<BodyAoS> bodies;					// bodies Original
    std::vector<BodyAoS> bodies_aos;				// bodies at actual time
	std::vector<BodyAoS> bodies_aos_l; 				// bodies at one less timestep (lagging behind)								
	std::vector<BodyAoSoA> bodies_aosoa; 			// bodies Array of Structure of Arrays

    NBodySimulation() 
	    : snapshot_interval(0.01), endtime(1.0), dt(0.001), bodies() {
		    // If no input is given use a simple 3 body problem
		    Body b;
		    b.x[0] = 0.0; b.x[1] = 0.0; b.x[2] = 0.0;
		    b.v[0] = 1.0; b.v[1] = 0.0; b.v[2] = 0.0;
		    b.mass = 1.0;
		    bodies.push_back(b);

		    b.x[0] = 1.0; b.x[1] = 0.0; b.x[2] = 0.0;
		    b.v[0] = 0.0; b.v[1] = 1.0; b.v[2] = 0.0;
		    b.mass = 1.0;
		    bodies.push_back(b);

		    b.x[0] = -1.0; b.x[1] = 0.0; b.x[2] = 0.0;
		    b.v[0] = 0.0; b.v[1] = -1.0; b.v[2] = 0.0;
		    b.mass = 1.0;
		    bodies.push_back(b);
    }

	void aos_setup(const std::string& filename) {
		std::ifstream infile(filename);
		if (!infile) {
			std::cerr << "File not found" << "\n";
		}

		infile >> snapshot_interval >> endtime >> dt; 

		BodyAoS aos;
		while (infile >> aos.x[0] >> aos.x[1] >> aos.x[2]
			>> aos.v[0] >> aos.v[1] >> aos.v[2]
			>> aos.mass) {
			if (aos.mass <= 0.0) {
				std::cerr << "Invalid mass encountered.\n";
			}
			bodies_aos.push_back(aos);
			number_of_bodies += 1; 
		}
	}

	void soa_setup(const std::string& filename) {
		std::ifstream infile(filename); 
		std::string line; 
		if (!infile) {
			std::cerr << "File not found" << "\n"; 
		}
		
		infile >> snapshot_interval >> endtime >> dt; 

		number_of_bodies = -1;							// comment out if we are only using SoA
		while (getline(infile, line)) {
			number_of_bodies += 1; 
		}

		infile.clear(); 
		infile >> snapshot_interval >> endtime >> dt;
		double x{};
		double y{};
		double z{};
		double velocity_x{};
		double velocity_y{}; 
		double velocity_z{}; 
		double mass{}; 
		bodies_soa = BodySoA(number_of_bodies); 
		size_t index{}; 
		while (infile >> x >> y >> z
			>> velocity_x >> velocity_y >> velocity_z
			>> mass) {
			if (mass <= 0) {
				std::cerr << "Invalid mass encountered" << "\n"; 
			}
			bodies_soa.x[index] = x;
			bodies_soa.y[index] = y;
			bodies_soa.z[index] = z;
			bodies_soa.velocity_x[index] = velocity_x;
			bodies_soa.velocity_y[index] = velocity_y;
			bodies_soa.velocity_z[index] = velocity_z;
			bodies_soa.mass[index] = mass;

			index += 1;
		}
		//bodies_soa = std::move(soa);		// might be error prone here
	}

	void aosoa_setup(const std::string& filename) {
		std::ifstream infile(filename); 
		if (!infile) {
			std::cerr << "File not found" << "\n"; 
		}
		infile >> snapshot_interval >> endtime >> dt; 
		BodyAoSoA aosoa; 
		for (size_t i{}; i < block_size; ++i) {
			// we can only insert block_size at a time into the array
			infile >> aosoa.x[i] >> aosoa.y[i] >> aosoa.z[i]
				>> aosoa.velocity_x[i] >> aosoa.velocity_y[i] >> aosoa.velocity_z[i]
				>> aosoa.mass[i]; 
			if (aosoa.mass[i] <= 0) {
				std::cerr << "Invalid mass encountered" << "\n"; 
			}
			bodies_aosoa.push_back(aosoa); 
			number_of_bodies += 1;
		}
	}
	
    explicit NBodySimulation(const std::string& filename)
	    : snapshot_interval(0.0), endtime(0.0), dt(0.0), bodies() {
		aos_setup(filename); 
		soa_setup(filename); 
		aosoa_setup(filename); 
	}
};

#endif // NBODY_SIMULATION_H

