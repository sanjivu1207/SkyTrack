#pragma once


#include <random>
#include <vector>

struct Target {
    int id = 0;
    double x = 0.0, y = 0.0;    
    double vx = 0.0, vy = 0.0;  
    double turnRate = 0.0;      
};

struct RadarConfig {
    double maxRange = 50'000.0;   
    double rangeSigma = 50.0;     
    double bearingSigma = 0.002;  
    double probDetection = 0.9;   
    double clutterPerSweep = 5.0; 
};

struct Detection {
    double x = 0.0, y = 0.0;    
    double range = 0.0;         
    double bearing = 0.0;       
    int truthId = -1;           
};

class Simulator {
public:
    explicit Simulator(const RadarConfig& cfg, unsigned seed = 42);


    void addRandomTargets(int n);
    void addTarget(const Target& t) { targets_.push_back(t); }


    void step(double dt);


    std::vector<Detection> sweep();

    const std::vector<Target>& targets() const { return targets_; }
    const RadarConfig& config() const { return cfg_; }
    bool inCoverage(const Target& t) const;

private:
    RadarConfig cfg_;
    std::mt19937 rng_;
    std::vector<Target> targets_;
    int nextId_ = 0;
};
