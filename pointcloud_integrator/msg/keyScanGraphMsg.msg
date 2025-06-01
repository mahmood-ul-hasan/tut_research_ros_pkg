# data as serialization of sim(3)'s: (int id, float[7] scanToWorld)
uint32 numScans
uint8[] scanData


# constraints (int from, int to, float err)
uint32 numConstraints
uint8[] constraintsData
