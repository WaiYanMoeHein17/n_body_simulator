#ifndef IO_H
#define IO_H

#include "NBodySimulation.h"

enum class BodyType {
	AOS = 0, 
	SOA, 
	AOSOA, 
};

void writeVTKSnapshot(NBodySimulation nbs, BodyType body, int snapshotNumber);

void openPVDFile();
void addSnapshotToPVD(int snapshotNumber);
void closePVDFile();

#endif // IO_H

