#ifndef IO_H
#define IO_H

#include "NBodySimulation.h"
#include <type_traits>

enum class Layout {
	AOS = 0, 
	SOA, 
	AOSOA, 
};

template <typename T> 
concept Enum = std::is_enum<T>::value;

template <Layout layout>
struct vtk_writer {
	static void write_vtk_snapshot(NBodySimulation nbs, int snapshot_number); 
};

template <Enum T> 
void write_vtk_snapshot(NBodySimulation nbs, int snapshot_number);						

void openPVDFile();

void addSnapshotToPVD(int snapshot_number);

void closePVDFile();

#endif // IO_H

