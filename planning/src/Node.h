#pragma once

float fZero = 0.0;
float inf = 1 / fZero;

class Node {
private:
	float g_;
	float rhs_;
	bool memory_;

public:
	Node() : g_(inf), rhs_(inf) {}
	float getG() { return g_; }
	float getRHS() { return rhs_; }
	bool getMemory() { return memory_; }

	void setG(float g) { g_ = g; }
	void setRHS(float rhs) { rhs_ = rhs; }
	void setMemory(bool memory) { memory_ = memory; }	
};